const autoStatus = document.getElementById("auto-status");
const mapScale = document.getElementById("map-scale");
const mapView = document.getElementById("map-view");
const roomCount = document.getElementById("room-count");
const roomList = document.getElementById("room-list");
const mappingState = document.getElementById("mapping-state");
const cameraState = document.getElementById("camera-state");
const cameraFeed = document.getElementById("camera-feed");
const teachState = document.getElementById("teach-state");
const teachRouteList = document.getElementById("teach-route-list");

const roomNameInput = document.getElementById("room-name");
const startMappingButton = document.getElementById("start-mapping");
const backAtDockButton = document.getElementById("back-at-dock");
const saveRoomButton = document.getElementById("save-room");
const cancelMappingButton = document.getElementById("cancel-mapping");
const teachRouteNameInput = document.getElementById("teach-route-name");
const startTeachButton = document.getElementById("start-teach");
const finishTeachButton = document.getElementById("finish-teach");
const cancelTeachButton = document.getElementById("cancel-teach");

const startCleanButton = document.getElementById("start-clean");
const dockNowButton = document.getElementById("dock-now");
const autoStopButton = document.getElementById("auto-stop");

const speedSlider = document.getElementById("mapping-speed");
const speedValue = document.getElementById("mapping-speed-value");

const throttleValue = document.getElementById("mapping-throttle-value");
const steeringValue = document.getElementById("mapping-steering-value");
const leftWheelValue = document.getElementById("mapping-left-wheel-value");
const rightWheelValue = document.getElementById("mapping-right-wheel-value");


let maximumSpeed = Number(speedSlider.value);

let targetThrottle = 0;
let targetSteering = 0;
let smoothThrottle = 0;
let smoothSteering = 0;

let mappingActive = false;
let mappingClosed = false;
let teachActive = false;
let currentCleaningState = "idle";
let currentTeachReplayState = "idle";

let requestRunning = false;
let pendingRequest = false;
let lastTransmissionTime = 0;
let selectedRoomIds = new Set();
let cameraFeedLoaded = false;
let currentTeachState = {routes: []};
let currentTeachReplayData = {state: "idle"};
let labelingRouteId = null;
let landmarkNameDraft = "";
let landmarkSelectedKeyframeIndex = null;
let landmarkZoom = 1;
let landmarkDraftBox = {
    x: 0.5,
    y: 0.5,
    width: 0.28,
    height: 0.28
};

const SEND_INTERVAL_MS = 60;
const SMOOTHING = 0.22;
const DEAD_ZONE = 0.06;
const AUTONOMOUS_DRIVING_ENABLED = true;

const SVG_NS = "http://www.w3.org/2000/svg";
const ROOM_COLORS = [
    ["#38bdf8", "rgba(56, 189, 248, 0.18)"],
    ["#f97316", "rgba(249, 115, 22, 0.16)"],
    ["#22c55e", "rgba(34, 197, 94, 0.16)"],
    ["#eab308", "rgba(234, 179, 8, 0.15)"],
    ["#a78bfa", "rgba(167, 139, 250, 0.16)"]
];


function applyDeadZone(value) {
    if (Math.abs(value) < DEAD_ZONE) {
        return 0;
    }

    const sign = Math.sign(value);
    const adjusted =
        (Math.abs(value) - DEAD_ZONE)
        / (1 - DEAD_ZONE);

    return sign * adjusted;
}


function createJoystick(
    elementId,
    allowedAxis,
    onChange
) {
    const base = document.getElementById(elementId);
    const knob = base.querySelector(".joystick-knob");

    let activePointerId = null;


    function updatePosition(event) {
        const rect = base.getBoundingClientRect();

        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        const radiusX =
            rect.width / 2 - knob.offsetWidth / 2;

        const radiusY =
            rect.height / 2 - knob.offsetHeight / 2;

        let offsetX = event.clientX - centerX;
        let offsetY = event.clientY - centerY;

        offsetX = Math.max(
            -radiusX,
            Math.min(radiusX, offsetX)
        );

        offsetY = Math.max(
            -radiusY,
            Math.min(radiusY, offsetY)
        );

        if (allowedAxis === "vertical") {
            offsetX = 0;

            knob.style.transform =
                `translate(
                    calc(-50% + ${offsetX}px),
                    calc(-50% + ${offsetY}px)
                )`;

            const normalized =
                applyDeadZone(-offsetY / radiusY);

            onChange(normalized);
        }

        if (allowedAxis === "horizontal") {
            offsetY = 0;

            knob.style.transform =
                `translate(
                    calc(-50% + ${offsetX}px),
                    calc(-50% + ${offsetY}px)
                )`;

            const normalized =
                applyDeadZone(offsetX / radiusX);

            onChange(normalized);
        }
    }


    function reset() {
        activePointerId = null;

        knob.style.transform =
            "translate(-50%, -50%)";

        onChange(0);
    }


    base.addEventListener("pointerdown", (event) => {
        if (!driveControlsActive()) {
            return;
        }

        event.preventDefault();

        activePointerId = event.pointerId;
        base.setPointerCapture(event.pointerId);

        updatePosition(event);
    });


    base.addEventListener("pointermove", (event) => {
        if (event.pointerId !== activePointerId) {
            return;
        }

        event.preventDefault();
        updatePosition(event);
    });


    base.addEventListener("pointerup", (event) => {
        if (event.pointerId === activePointerId) {
            reset();
        }
    });


    base.addEventListener("pointercancel", reset);
    base.addEventListener("lostpointercapture", reset);

    return reset;
}


const resetThrottle = createJoystick(
    "mapping-throttle-joystick",
    "vertical",
    (value) => {
        targetThrottle = value;

        throttleValue.textContent =
            Math.round(value * 100) + "%";
    }
);


const resetSteering = createJoystick(
    "mapping-steering-joystick",
    "horizontal",
    (value) => {
        targetSteering = value;

        steeringValue.textContent =
            Math.round(value * 100) + "%";
    }
);


function calculateWheelSpeeds() {
    if (!driveControlsActive()) {
        return {
            left: 0,
            right: 0
        };
    }

    const throttle =
        smoothThrottle * maximumSpeed;

    const steering =
        smoothSteering * maximumSpeed;

    let left = throttle + steering;
    let right = throttle - steering;

    const largestMagnitude = Math.max(
        maximumSpeed,
        Math.abs(left),
        Math.abs(right)
    );

    if (largestMagnitude > maximumSpeed) {
        const scale =
            maximumSpeed / largestMagnitude;

        left *= scale;
        right *= scale;
    }

    return {
        left: Math.round(left),
        right: Math.round(right)
    };
}


async function readJson(response) {
    try {
        return await response.json();
    } catch (error) {
        return {};
    }
}


async function postJson(path, body) {
    const response = await fetch(path, {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(body || {})
    });

    const data = await readJson(response);

    if (!response.ok || !data.ok) {
        throw new Error(data.error || "Request failed");
    }

    return data;
}


function sendSafetyStop() {
    resetDriveControls();

    if (navigator.sendBeacon) {
        const body = new Blob(
            [JSON.stringify({})],
            {
                type: "application/json"
            }
        );

        navigator.sendBeacon("/safety/stop", body);
        return;
    }

    fetch("/safety/stop", {
        method: "POST",
        keepalive: true
    }).catch((error) => {
        console.error(error);
    });
}


async function deleteJson(path) {
    const response = await fetch(path, {
        method: "DELETE"
    });

    const data = await readJson(response);

    if (!response.ok || !data.ok) {
        throw new Error(data.error || "Request failed");
    }

    return data;
}


async function sendWheelSpeeds(left, right) {
    if (!driveControlsActive() && (left !== 0 || right !== 0)) {
        return;
    }

    if (requestRunning) {
        pendingRequest = true;
        return;
    }

    requestRunning = true;

    try {
        const response = await fetch("/drive", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                left,
                right
            })
        });

        if (!response.ok) {
            throw new Error(
                `Drive request failed: ${response.status}`
            );
        }
    } catch (error) {
        autoStatus.textContent = "Connection error";
        console.error(error);
    } finally {
        requestRunning = false;

        if (pendingRequest) {
            pendingRequest = false;

            const speeds = calculateWheelSpeeds();

            sendWheelSpeeds(
                speeds.left,
                speeds.right
            );
        }
    }
}


function controlLoop(timestamp) {
    smoothThrottle +=
        (targetThrottle - smoothThrottle)
        * SMOOTHING;

    smoothSteering +=
        (targetSteering - smoothSteering)
        * SMOOTHING;

    if (Math.abs(smoothThrottle) < 0.005) {
        smoothThrottle = 0;
    }

    if (Math.abs(smoothSteering) < 0.005) {
        smoothSteering = 0;
    }

    const speeds = calculateWheelSpeeds();

    leftWheelValue.textContent = speeds.left;
    rightWheelValue.textContent = speeds.right;

    if (
        timestamp - lastTransmissionTime
        >= SEND_INTERVAL_MS
    ) {
        lastTransmissionTime = timestamp;

        if (driveControlsActive()) {
            sendWheelSpeeds(
                speeds.left,
                speeds.right
            );
        }
    }

    requestAnimationFrame(controlLoop);
}


requestAnimationFrame(controlLoop);


function driveControlsActive() {
    return mappingActive || teachActive;
}


function resetDriveControls() {
    targetThrottle = 0;
    targetSteering = 0;
    smoothThrottle = 0;
    smoothSteering = 0;

    resetThrottle();
    resetSteering();

    throttleValue.textContent = "0%";
    steeringValue.textContent = "0%";
    leftWheelValue.textContent = "0";
    rightWheelValue.textContent = "0";
}


async function emergencyStop() {
    resetDriveControls();

    try {
        await fetch("/safety/stop", {
            method: "POST"
        });

        autoStatus.textContent = "Stopped";
        await refreshState();
    } catch (error) {
        autoStatus.textContent = "Stop request failed";
        console.error(error);
    }
}


async function sendCleaningHeartbeat() {
    if (currentCleaningState !== "running") {
        return;
    }

    try {
        await postJson("/clean/heartbeat");
    } catch (error) {
        autoStatus.textContent = "Heartbeat lost; stopping";
        sendSafetyStop();
        console.error(error);
    }
}


async function sendTeachReplayHeartbeat() {
    if (currentTeachReplayState !== "running") {
        return;
    }

    try {
        await postJson("/teach-replay/heartbeat");
    } catch (error) {
        autoStatus.textContent = "Replay heartbeat lost; stopping";
        sendSafetyStop();
        console.error(error);
    }
}


function createSvgElement(name, attributes) {
    const element = document.createElementNS(SVG_NS, name);

    Object.entries(attributes).forEach(([key, value]) => {
        element.setAttribute(key, value);
    });

    return element;
}


function pointString(points) {
    return points
        .map((point) => `${point[0]},${-point[1]}`)
        .join(" ");
}


function roomAreaLabel(areaMm2) {
    const squareMeters = areaMm2 / 1_000_000;
    return squareMeters.toFixed(2) + " m2";
}


function routeDistanceLabel(distanceMm) {
    return (distanceMm / 1000).toFixed(2) + " m";
}


function renderMap(data) {
    const rooms = data.rooms || [];
    const mapping = data.mapping || {};
    const cleaning = data.cleaning || {};
    const teach = data.teach || {};
    const teachReplay = data.teach_replay || {};
    const routes = data.routes || {};
    const activePath = mapping.active
        ? mapping.path || []
        : mapping.draft_points || [];
    const activeTeachPoints = teach.active
        ? teach.active.points || []
        : [];

    const allPoints = [[0, 0]];

    rooms.forEach((room) => {
        room.points.forEach((point) => allPoints.push(point));
    });

    activePath.forEach((point) => allPoints.push(point));
    activeTeachPoints.forEach((point) => allPoints.push(point));

    if (teachReplay.pose && teachReplay.state === "running") {
        allPoints.push([
            teachReplay.pose.x || 0,
            teachReplay.pose.y || 0
        ]);
    }

    (teach.routes || []).forEach((route) => {
        (route.points || []).forEach((point) => allPoints.push(point));
    });

    Object.entries(routes).forEach(([roomId, route]) => {
        if (selectedRoomIds.has(roomId)) {
            route.forEach((point) => allPoints.push(point));
        }
    });

    let minX = Math.min(...allPoints.map((point) => point[0]));
    let maxX = Math.max(...allPoints.map((point) => point[0]));
    let minY = Math.min(...allPoints.map((point) => -point[1]));
    let maxY = Math.max(...allPoints.map((point) => -point[1]));

    if (maxX - minX < 1000) {
        minX = -500;
        maxX = 500;
    }

    if (maxY - minY < 1000) {
        minY = -500;
        maxY = 500;
    }

    const padding = Math.max(maxX - minX, maxY - minY) * 0.08;
    minX -= padding;
    maxX += padding;
    minY -= padding;
    maxY += padding;

    mapView.setAttribute(
        "viewBox",
        `${minX} ${minY} ${maxX - minX} ${maxY - minY}`
    );

    mapView.innerHTML = "";

    rooms.forEach((room, index) => {
        const colors = ROOM_COLORS[index % ROOM_COLORS.length];
        const polygon = createSvgElement("polygon", {
            points: pointString(room.points),
            fill: colors[1],
            stroke: colors[0],
            "stroke-width": "26",
            "stroke-linejoin": "round"
        });

        mapView.appendChild(polygon);
    });

    Object.entries(routes).forEach(([roomId, route]) => {
        if (!selectedRoomIds.has(roomId) || route.length < 2) {
            return;
        }

        const routeLine = createSvgElement("polyline", {
            points: pointString(route),
            fill: "none",
            stroke: "#ef4444",
            "stroke-width": "20",
            "stroke-linecap": "round",
            "stroke-linejoin": "round",
            "stroke-dasharray": "52 32",
            opacity: "0.92"
        });

        mapView.appendChild(routeLine);
    });

    if (activePath.length > 1) {
        const path = createSvgElement(
            mapping.active ? "polyline" : "polygon",
            {
                points: pointString(activePath),
                fill: mapping.active
                    ? "none"
                    : "rgba(245, 158, 11, 0.14)",
                stroke: "#f59e0b",
                "stroke-width": "24",
                "stroke-linecap": "round",
                "stroke-linejoin": "round"
            }
        );

        mapView.appendChild(path);
    }

    (teach.routes || []).forEach((route) => {
        if (!route.points || route.points.length < 2) {
            return;
        }

        const taughtPath = createSvgElement("polyline", {
            points: pointString(route.points),
            fill: "none",
            stroke: "#a78bfa",
            "stroke-width": "16",
            "stroke-linecap": "round",
            "stroke-linejoin": "round",
            opacity: "0.72"
        });

        mapView.appendChild(taughtPath);
    });

    if (activeTeachPoints.length > 1) {
        const activeTeachPath = createSvgElement("polyline", {
            points: pointString(activeTeachPoints),
            fill: "none",
            stroke: "#f97316",
            "stroke-width": "20",
            "stroke-linecap": "round",
            "stroke-linejoin": "round"
        });

        mapView.appendChild(activeTeachPath);
    }

    if (mapping.active && mapping.pose) {
        const pose = createSvgElement("circle", {
            cx: mapping.pose.x,
            cy: -mapping.pose.y,
            r: "48",
            fill: "#f8fafc",
            stroke: "#111827",
            "stroke-width": "16"
        });

        mapView.appendChild(pose);
    }

    if (cleaning.pose && cleaning.state === "running") {
        const cleanerPose = createSvgElement("circle", {
            cx: cleaning.pose.x,
            cy: -cleaning.pose.y,
            r: "44",
            fill: "#ef4444",
            stroke: "#111827",
            "stroke-width": "14"
        });

        mapView.appendChild(cleanerPose);
    }

    if (teachReplay.pose && teachReplay.state === "running") {
        const replayPose = createSvgElement("circle", {
            cx: teachReplay.pose.x,
            cy: -teachReplay.pose.y,
            r: "44",
            fill: "#a78bfa",
            stroke: "#111827",
            "stroke-width": "14"
        });

        mapView.appendChild(replayPose);
    }

    const dockRing = createSvgElement("circle", {
        cx: "0",
        cy: "0",
        r: "58",
        fill: "#22c55e",
        stroke: "#ecfccb",
        "stroke-width": "16"
    });

    const dockLineHorizontal = createSvgElement("line", {
        x1: "-90",
        y1: "0",
        x2: "90",
        y2: "0",
        stroke: "#ecfccb",
        "stroke-width": "16",
        "stroke-linecap": "round"
    });

    const dockLineVertical = createSvgElement("line", {
        x1: "0",
        y1: "-90",
        x2: "0",
        y2: "90",
        stroke: "#ecfccb",
        "stroke-width": "16",
        "stroke-linecap": "round"
    });

    mapView.appendChild(dockRing);
    mapView.appendChild(dockLineHorizontal);
    mapView.appendChild(dockLineVertical);

    const mapWidthMeters = (maxX - minX) / 1000;
    mapScale.textContent =
        mapWidthMeters.toFixed(1) + " m span";
}


async function startTeachReplay(route) {
    const confirmed = window.confirm(
        `Replay route "${route.name}" now? Keep this page open and be ready to press STOP.`
    );

    if (!confirmed) {
        autoStatus.textContent = "Route replay cancelled";
        return;
    }

    try {
        await postJson(`/teach-routes/${route.id}/go`);
        await refreshState();
        autoStatus.textContent = "Route replay started";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
}


async function stopTeachReplay() {
    try {
        await postJson("/teach-replay/stop");
        await refreshState();
        autoStatus.textContent = "Route replay stopped";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
}


async function saveLandmark(route, keyframe) {
    clampLandmarkDraftBox();

    try {
        await postJson(`/teach-routes/${route.id}/landmarks`, {
            name: landmarkNameDraft,
            keyframe_index: keyframe.index,
            x: landmarkDraftBox.x,
            y: landmarkDraftBox.y,
            width: landmarkDraftBox.width,
            height: landmarkDraftBox.height
        });
        landmarkNameDraft = "";
        await refreshState();
        autoStatus.textContent = "Landmark saved";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
}


function clampLandmarkDraftBox() {
    landmarkDraftBox.width = Math.max(
        0.08,
        Math.min(0.9, landmarkDraftBox.width)
    );
    landmarkDraftBox.height = Math.max(
        0.08,
        Math.min(0.9, landmarkDraftBox.height)
    );
    landmarkDraftBox.x = Math.max(
        landmarkDraftBox.width / 2,
        Math.min(1 - landmarkDraftBox.width / 2, landmarkDraftBox.x)
    );
    landmarkDraftBox.y = Math.max(
        landmarkDraftBox.height / 2,
        Math.min(1 - landmarkDraftBox.height / 2, landmarkDraftBox.y)
    );
}


function setLandmarkBoxCenterFromPointer(image, event) {
    const rect = image.getBoundingClientRect();

    landmarkDraftBox.x = Math.max(
        0,
        Math.min(1, (event.clientX - rect.left) / rect.width)
    );
    landmarkDraftBox.y = Math.max(
        0,
        Math.min(1, (event.clientY - rect.top) / rect.height)
    );
    clampLandmarkDraftBox();
}


function createLandmarkEditor(route) {
    const editor = document.createElement("div");
    editor.className = "landmark-editor";

    if (!route.keyframes || route.keyframes.length === 0) {
        const empty = document.createElement("p");
        empty.className = "room-empty";
        empty.textContent =
            "No camera frames were saved for this route. Teach the route again while Camera says Live.";
        editor.appendChild(empty);
        return editor;
    }

    const selectedKeyframe = route.keyframes.find((keyframe) => {
        return keyframe.index === landmarkSelectedKeyframeIndex;
    }) || route.keyframes[0];

    landmarkSelectedKeyframeIndex = selectedKeyframe.index;
    clampLandmarkDraftBox();

    const nameInput = document.createElement("input");
    nameInput.className = "room-name-input landmark-name-input";
    nameInput.type = "text";
    nameInput.placeholder = "Landmark name";
    nameInput.autocomplete = "off";
    nameInput.value = landmarkNameDraft;
    nameInput.addEventListener("input", () => {
        landmarkNameDraft = nameInput.value;
    });
    editor.appendChild(nameInput);

    const toolbar = document.createElement("div");
    toolbar.className = "landmark-editor-toolbar";

    const zoomControl = document.createElement("label");
    zoomControl.textContent = "Zoom";

    const zoomInput = document.createElement("input");
    zoomInput.type = "range";
    zoomInput.min = "1";
    zoomInput.max = "3";
    zoomInput.step = "0.1";
    zoomInput.value = String(landmarkZoom);
    zoomInput.addEventListener("input", () => {
        landmarkZoom = Number(zoomInput.value);
        renderTeachRoutes(currentTeachState, currentTeachReplayData);
    });
    zoomControl.appendChild(zoomInput);

    const widthControl = document.createElement("label");
    widthControl.textContent = "Width";

    const widthInput = document.createElement("input");
    widthInput.type = "range";
    widthInput.min = "0.08";
    widthInput.max = "0.9";
    widthInput.step = "0.01";
    widthInput.value = String(landmarkDraftBox.width);
    widthInput.addEventListener("input", () => {
        landmarkDraftBox.width = Number(widthInput.value);
        clampLandmarkDraftBox();
        renderTeachRoutes(currentTeachState, currentTeachReplayData);
    });
    widthControl.appendChild(widthInput);

    const heightControl = document.createElement("label");
    heightControl.textContent = "Height";

    const heightInput = document.createElement("input");
    heightInput.type = "range";
    heightInput.min = "0.08";
    heightInput.max = "0.9";
    heightInput.step = "0.01";
    heightInput.value = String(landmarkDraftBox.height);
    heightInput.addEventListener("input", () => {
        landmarkDraftBox.height = Number(heightInput.value);
        clampLandmarkDraftBox();
        renderTeachRoutes(currentTeachState, currentTeachReplayData);
    });
    heightControl.appendChild(heightInput);

    const saveButton = document.createElement("button");
    saveButton.type = "button";
    saveButton.className = "primary-button";
    saveButton.textContent = "Save";
    saveButton.addEventListener("click", async () => {
        await saveLandmark(route, selectedKeyframe);
    });

    toolbar.appendChild(zoomControl);
    toolbar.appendChild(widthControl);
    toolbar.appendChild(heightControl);
    toolbar.appendChild(saveButton);
    editor.appendChild(toolbar);

    const selectedFrame = document.createElement("div");
    selectedFrame.className = "landmark-selected-frame";

    const viewport = document.createElement("div");
    viewport.className = "landmark-frame-viewport";

    const stage = document.createElement("div");
    stage.className = "landmark-image-stage";
    stage.style.width = `${landmarkZoom * 100}%`;

    const selectedImage = document.createElement("img");
    selectedImage.className = "landmark-main-image";
    selectedImage.src = selectedKeyframe.url;
    selectedImage.alt = route.name;

    const selectionBox = document.createElement("span");
    selectionBox.className = "landmark-selection-box";
    selectionBox.style.left =
        `${(landmarkDraftBox.x - landmarkDraftBox.width / 2) * 100}%`;
    selectionBox.style.top =
        `${(landmarkDraftBox.y - landmarkDraftBox.height / 2) * 100}%`;
    selectionBox.style.width = `${landmarkDraftBox.width * 100}%`;
    selectionBox.style.height = `${landmarkDraftBox.height * 100}%`;

    let dragging = false;

    function moveBox(event) {
        setLandmarkBoxCenterFromPointer(selectedImage, event);
        selectionBox.style.left =
            `${(landmarkDraftBox.x - landmarkDraftBox.width / 2) * 100}%`;
        selectionBox.style.top =
            `${(landmarkDraftBox.y - landmarkDraftBox.height / 2) * 100}%`;
    }

    stage.addEventListener("pointerdown", (event) => {
        event.preventDefault();
        dragging = true;
        stage.setPointerCapture(event.pointerId);
        moveBox(event);
    });

    stage.addEventListener("pointermove", (event) => {
        if (!dragging) {
            return;
        }

        event.preventDefault();
        moveBox(event);
    });

    stage.addEventListener("pointerup", (event) => {
        dragging = false;

        try {
            stage.releasePointerCapture(event.pointerId);
        } catch (error) {
            console.error(error);
        }
    });

    stage.addEventListener("pointercancel", () => {
        dragging = false;
    });

    stage.appendChild(selectedImage);
    stage.appendChild(selectionBox);
    viewport.appendChild(stage);
    selectedFrame.appendChild(viewport);
    editor.appendChild(selectedFrame);

    const landmarks = route.landmarks || [];

    if (landmarks.length > 0) {
        const landmarkList = document.createElement("div");
        landmarkList.className = "landmark-list";

        landmarks.forEach((landmark) => {
            const item = document.createElement("span");
            item.className = "landmark-item";

            const image = document.createElement("img");
            image.src = landmark.patch_url;
            image.alt = landmark.name;

            const name = document.createElement("strong");
            name.textContent = landmark.name;

            const deleteButton = document.createElement("button");
            deleteButton.type = "button";
            deleteButton.className = "delete-room";
            deleteButton.textContent = "Delete";
            deleteButton.addEventListener("click", async () => {
                try {
                    await deleteJson(
                        `/teach-routes/${route.id}/landmarks/${landmark.id}`
                    );
                    await refreshState();
                    autoStatus.textContent = "Landmark deleted";
                } catch (error) {
                    autoStatus.textContent = error.message;
                    console.error(error);
                }
            });

            item.appendChild(image);
            item.appendChild(name);
            item.appendChild(deleteButton);
            landmarkList.appendChild(item);
        });

        editor.appendChild(landmarkList);
    }

    const keyframes = document.createElement("div");
    keyframes.className = "landmark-keyframes";

    route.keyframes.forEach((keyframe) => {
        const frame = document.createElement("button");
        frame.type = "button";
        frame.className = "landmark-frame";
        frame.classList.toggle(
            "selected",
            keyframe.index === landmarkSelectedKeyframeIndex
        );

        const image = document.createElement("img");
        image.src = keyframe.url;
        image.alt = route.name;
        frame.addEventListener("click", () => {
            landmarkSelectedKeyframeIndex = keyframe.index;
            landmarkDraftBox = {
                x: 0.5,
                y: 0.5,
                width: landmarkDraftBox.width,
                height: landmarkDraftBox.height
            };
            renderTeachRoutes(currentTeachState, currentTeachReplayData);
        });

        const label = document.createElement("small");
        label.textContent =
            Number(keyframe.timestamp || 0).toFixed(1) + "s";

        frame.appendChild(image);
        frame.appendChild(label);
        keyframes.appendChild(frame);
    });

    editor.appendChild(keyframes);
    return editor;
}


function renderTeachRoutes(teach, teachReplay) {
    const routes = teach.routes || [];
    const replayRunning =
        teachReplay && teachReplay.state === "running";

    teachRouteList.innerHTML = "";

    if (routes.length === 0) {
        const empty = document.createElement("p");
        empty.className = "room-empty";
        empty.textContent = "No teach routes";
        teachRouteList.appendChild(empty);
        return;
    }

    routes.forEach((route) => {
        const replayingThis =
            replayRunning && teachReplay.route_id === route.id;

        const item = document.createElement("div");
        item.className = "teach-route-item";

        const details = document.createElement("span");
        details.className = "teach-route-details";

        const name = document.createElement("strong");
        name.textContent = route.name;

        const summary = document.createElement("small");
        summary.textContent =
            routeDistanceLabel(route.distance_mm || 0)
            + " - "
            + (route.keyframe_count || 0)
            + " frames - "
            + (route.landmark_count || 0)
            + " landmarks";

        details.appendChild(name);
        details.appendChild(summary);

        if (route.keyframes && route.keyframes.length > 0) {
            const keyframes = document.createElement("span");
            keyframes.className = "teach-keyframes";

            route.keyframes.slice(0, 3).forEach((keyframe) => {
                const image = document.createElement("img");
                image.src = keyframe.url;
                image.alt = route.name;
                keyframes.appendChild(image);
            });

            details.appendChild(keyframes);
        }

        const actions = document.createElement("span");
        actions.className = "teach-route-actions";

        const labelButton = document.createElement("button");
        labelButton.type = "button";
        labelButton.className = "label-route";
        labelButton.textContent = labelingRouteId === route.id
            ? "Close"
            : "Label";
        labelButton.disabled = replayRunning || teachActive;
        labelButton.addEventListener("click", () => {
            if (labelingRouteId === route.id) {
                labelingRouteId = null;
            } else {
                labelingRouteId = route.id;
                landmarkSelectedKeyframeIndex =
                    route.keyframes && route.keyframes.length > 0
                        ? route.keyframes[0].index
                        : null;
                landmarkZoom = 1;
                landmarkDraftBox = {
                    x: 0.5,
                    y: 0.5,
                    width: 0.28,
                    height: 0.28
                };
            }

            renderTeachRoutes(teach, teachReplay);
        });

        const goButton = document.createElement("button");
        goButton.type = "button";
        goButton.className = replayingThis
            ? "stop-route"
            : "go-route";
        goButton.textContent = replayingThis
            ? "Stop"
            : "Go";
        goButton.disabled =
            teachActive
            || mappingActive
            || currentCleaningState === "running"
            || currentCleaningState === "docking"
            || (replayRunning && !replayingThis);
        goButton.addEventListener("click", async () => {
            if (replayingThis) {
                await stopTeachReplay();
                return;
            }

            await startTeachReplay(route);
        });

        const deleteButton = document.createElement("button");
        deleteButton.type = "button";
        deleteButton.className = "delete-room";
        deleteButton.textContent = "Delete";
        deleteButton.disabled = replayRunning || teachActive;
        deleteButton.addEventListener("click", async () => {
            try {
                await deleteJson(`/teach-routes/${route.id}`);
                if (labelingRouteId === route.id) {
                    labelingRouteId = null;
                }
                await refreshState();
                autoStatus.textContent = "Teach route deleted";
            } catch (error) {
                autoStatus.textContent = error.message;
                console.error(error);
            }
        });

        actions.appendChild(labelButton);
        actions.appendChild(goButton);
        actions.appendChild(deleteButton);

        item.appendChild(details);
        item.appendChild(actions);

        if (labelingRouteId === route.id) {
            item.appendChild(createLandmarkEditor(route));
        }

        teachRouteList.appendChild(item);
    });
}


function renderRoomList(rooms) {
    roomCount.textContent =
        rooms.length + (rooms.length === 1 ? " saved" : " saved");

    roomList.innerHTML = "";

    if (rooms.length === 0) {
        const empty = document.createElement("p");
        empty.className = "room-empty";
        empty.textContent = "No rooms saved";
        roomList.appendChild(empty);
        return;
    }

    rooms.forEach((room) => {
        const item = document.createElement("label");
        item.className = "room-item";

        const checkbox = document.createElement("input");
        checkbox.type = "checkbox";
        checkbox.value = room.id;
        checkbox.checked = selectedRoomIds.has(room.id);

        checkbox.addEventListener("change", () => {
            if (checkbox.checked) {
                selectedRoomIds.clear();
                selectedRoomIds.add(room.id);

                roomList
                    .querySelectorAll("input[type='checkbox']")
                    .forEach((input) => {
                        input.checked = input.value === room.id;
                    });
            } else {
                selectedRoomIds.delete(room.id);
            }
        });

        const details = document.createElement("span");
        details.className = "room-details";

        const name = document.createElement("strong");
        name.textContent = room.name;

        const area = document.createElement("small");
        area.textContent = roomAreaLabel(room.area_mm2);

        details.appendChild(name);
        details.appendChild(area);

        const deleteButton = document.createElement("button");
        deleteButton.type = "button";
        deleteButton.className = "delete-room";
        deleteButton.textContent = "Delete";
        deleteButton.addEventListener("click", async (event) => {
            event.preventDefault();
            event.stopPropagation();

            try {
                await deleteJson(`/rooms/${room.id}`);
                selectedRoomIds.delete(room.id);
                await refreshState();
                autoStatus.textContent = "Room deleted";
            } catch (error) {
                autoStatus.textContent = error.message;
                console.error(error);
            }
        });

        item.appendChild(checkbox);
        item.appendChild(details);
        item.appendChild(deleteButton);
        roomList.appendChild(item);
    });
}


function applyState(data) {
    const mapping = data.mapping || {};
    const cleaning = data.cleaning || {};
    const teach = data.teach || {};
    const teachReplay = data.teach_replay || {};
    const camera = data.camera || {};
    const rooms = data.rooms || [];
    const wasMappingActive = mappingActive;
    const wasTeachActive = teachActive;
    const wasTeachReplayRunning =
        currentTeachReplayState === "running";

    mappingActive = Boolean(mapping.active);
    mappingClosed = Boolean(mapping.closed);
    teachActive = Boolean(teach.active);
    currentCleaningState = cleaning.state || "idle";
    currentTeachReplayState = teachReplay.state || "idle";
    currentTeachState = teach;
    currentTeachReplayData = teachReplay;

    const teachReplayRunning =
        currentTeachReplayState === "running";

    if (
        (wasMappingActive && !mappingActive)
        || (wasTeachActive && !teachActive)
        || (wasTeachReplayRunning && !teachReplayRunning)
    ) {
        resetDriveControls();
    }

    document.body.classList.toggle(
        "mapping-locked",
        !driveControlsActive()
    );

    mappingState.textContent = mappingActive
        ? "Recording"
        : mappingClosed
            ? "Ready To Save"
            : "Idle";

    if (teach.active) {
        teachState.textContent =
            "Recording - "
            + teach.active.sample_count
            + " samples - "
            + teach.active.keyframe_count
            + " frames";
    } else {
        teachState.textContent = "Idle";
    }

    if (camera.ok) {
        cameraState.textContent = "Live";
        cameraState.title = "";

        if (!cameraFeedLoaded) {
            cameraFeed.src = "/camera/stream?t=" + Date.now();
            cameraFeedLoaded = true;
        }
    } else {
        cameraState.textContent = camera.error
            ? "Offline - " + camera.error
            : "Offline";
        cameraState.title = camera.error || "";
        cameraFeed.removeAttribute("src");
        cameraFeedLoaded = false;
    }

    if (teachReplayRunning) {
        const progress =
            Math.round((teachReplay.progress || 0) * 100);
        const vision = teachReplay.vision || {};
        const visionText = vision.enabled
            ? " | "
                + vision.message
                + (
                    vision.landmark_name
                        ? ": " + vision.landmark_name
                        : ""
                )
                + (
                    vision.score
                        ? " " + Math.round(vision.score * 100) + "%"
                        : ""
                )
            : "";

        autoStatus.textContent =
            "Replaying route: "
            + (teachReplay.route_name || "Route")
            + " - "
            + progress
            + "%"
            + visionText;
    } else if (cleaning.room) {
        autoStatus.textContent =
            cleaning.message + ": " + cleaning.room;
    } else if (mapping.warning) {
        autoStatus.textContent = mapping.warning;
    } else if (
        teachReplay.message
        && teachReplay.message !== "Ready"
        && teachReplay.route_name
    ) {
        autoStatus.textContent =
            teachReplay.message + ": " + teachReplay.route_name;
    } else if (
        teachReplay.state === "error"
        && teachReplay.message
    ) {
        autoStatus.textContent = teachReplay.message;
    } else {
        autoStatus.textContent = cleaning.message || "Ready";
    }

    startMappingButton.disabled =
        mappingActive
        || teachActive
        || teachReplayRunning
        || currentCleaningState === "running";

    backAtDockButton.disabled = !mappingActive;
    saveRoomButton.disabled = !mappingClosed;
    cancelMappingButton.disabled = !(mappingActive || mappingClosed);

    startTeachButton.disabled =
        teachActive
        || mappingActive
        || teachReplayRunning
        || currentCleaningState === "running";

    finishTeachButton.disabled = !teachActive;
    cancelTeachButton.disabled = !teachActive;

    startCleanButton.disabled =
        !AUTONOMOUS_DRIVING_ENABLED
        || rooms.length === 0
        || selectedRoomIds.size === 0
        || teachActive
        || teachReplayRunning
        || currentCleaningState === "running";

    startCleanButton.textContent = AUTONOMOUS_DRIVING_ENABLED
        ? "Clean Selected"
        : "Clean Disabled";

    dockNowButton.disabled =
        mappingActive || teachActive || teachReplayRunning;

    renderMap(data);
    renderRoomList(rooms);
    renderTeachRoutes(teach, teachReplay);
}


async function refreshState() {
    const response = await fetch("/autonomous/state");
    const data = await readJson(response);

    if (!response.ok || !data.ok) {
        throw new Error(data.error || "Could not load state");
    }

    applyState(data);
}


startMappingButton.addEventListener("click", async () => {
    try {
        await postJson("/mapping/start");
        await refreshState();
        autoStatus.textContent = "Mapping active";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


startTeachButton.addEventListener("click", async () => {
    try {
        await postJson("/teach/start", {
            name: teachRouteNameInput.value
        });
        await refreshState();
        autoStatus.textContent = "Teaching route";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


finishTeachButton.addEventListener("click", async () => {
    try {
        resetDriveControls();
        await postJson("/teach/finish", {
            name: teachRouteNameInput.value
        });
        teachRouteNameInput.value = "";
        await refreshState();
        autoStatus.textContent = "Teach route saved";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


cancelTeachButton.addEventListener("click", async () => {
    try {
        resetDriveControls();
        await postJson("/teach/cancel");
        await refreshState();
        autoStatus.textContent = "Teach route cancelled";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


backAtDockButton.addEventListener("click", async () => {
    try {
        resetDriveControls();
        await postJson("/mapping/back-at-dock");
        await refreshState();
        autoStatus.textContent = "Dock reset";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


saveRoomButton.addEventListener("click", async () => {
    try {
        await postJson("/mapping/save", {
            name: roomNameInput.value
        });

        roomNameInput.value = "";
        await refreshState();
        autoStatus.textContent = "Room saved";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


cancelMappingButton.addEventListener("click", async () => {
    try {
        resetDriveControls();
        await postJson("/mapping/cancel");
        await refreshState();
        autoStatus.textContent = "Mapping cancelled";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


startCleanButton.addEventListener("click", async () => {
    if (!AUTONOMOUS_DRIVING_ENABLED) {
        autoStatus.textContent =
            "Autonomous driving needs localization first";
        return;
    }

    const roomCount = selectedRoomIds.size;

    const confirmed = window.confirm(
        `Start autonomous cleaning for ${roomCount} selected room(s)? Keep this page open.`
    );

    if (!confirmed) {
        autoStatus.textContent = "Cleaning cancelled";
        return;
    }

    try {
        const data = await postJson("/clean/start", {
            rooms: Array.from(selectedRoomIds)
        });

        autoStatus.textContent = data.cleaning.message;
        await refreshState();
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


dockNowButton.addEventListener("click", async () => {
    const confirmed = window.confirm(
        "Send the Roomba built-in dock command? It can move on its own."
    );

    if (!confirmed) {
        autoStatus.textContent = "Dock cancelled";
        return;
    }

    try {
        resetDriveControls();
        const data = await postJson("/dock", {
            confirm: true
        });
        autoStatus.textContent = data.cleaning.message;
        await refreshState();
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


autoStopButton.addEventListener(
    "click",
    emergencyStop
);


speedSlider.addEventListener("input", () => {
    maximumSpeed = Number(speedSlider.value);
    speedValue.textContent = maximumSpeed;
});


speedSlider.addEventListener("change", async () => {
    try {
        await postJson("/speed", {
            speed: maximumSpeed
        });

        autoStatus.textContent =
            `Maximum speed: ${maximumSpeed} mm/s`;
    } catch (error) {
        autoStatus.textContent = "Could not update speed";
        console.error(error);
    }
});


window.addEventListener("blur", () => {
    if (
        mappingActive
        || teachActive
        || currentTeachReplayState === "running"
        || currentCleaningState === "running"
    ) {
        emergencyStop();
    }
});


window.addEventListener("pagehide", () => {
    if (
        mappingActive
        || teachActive
        || currentTeachReplayState === "running"
        || currentCleaningState === "running"
        || currentCleaningState === "docking"
    ) {
        sendSafetyStop();
    }
});


document.addEventListener(
    "visibilitychange",
    () => {
        if (
            document.hidden
            && (
                mappingActive
                || teachActive
                || currentTeachReplayState === "running"
                || currentCleaningState === "running"
                || currentCleaningState === "docking"
            )
        ) {
            emergencyStop();
        }
    }
);


setInterval(() => {
    sendCleaningHeartbeat();
    sendTeachReplayHeartbeat();

    refreshState().catch((error) => {
        autoStatus.textContent = error.message;
        console.error(error);
    });
}, 1000);


refreshState().catch((error) => {
    autoStatus.textContent = error.message;
    console.error(error);
});

const autoStatus = document.getElementById("auto-status");
const mapScale = document.getElementById("map-scale");
const mapView = document.getElementById("map-view");
const roomCount = document.getElementById("room-count");
const roomList = document.getElementById("room-list");
const mappingState = document.getElementById("mapping-state");

const roomNameInput = document.getElementById("room-name");
const startMappingButton = document.getElementById("start-mapping");
const backAtDockButton = document.getElementById("back-at-dock");
const saveRoomButton = document.getElementById("save-room");
const cancelMappingButton = document.getElementById("cancel-mapping");

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
let currentCleaningState = "idle";

let requestRunning = false;
let pendingRequest = false;
let lastTransmissionTime = 0;
let selectedRoomIds = new Set();

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
        if (!mappingActive) {
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
    if (!mappingActive) {
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
    if (!mappingActive && (left !== 0 || right !== 0)) {
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

        if (mappingActive) {
            sendWheelSpeeds(
                speeds.left,
                speeds.right
            );
        }
    }

    requestAnimationFrame(controlLoop);
}


requestAnimationFrame(controlLoop);


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


function renderMap(data) {
    const rooms = data.rooms || [];
    const mapping = data.mapping || {};
    const activePath = mapping.active
        ? mapping.path || []
        : mapping.draft_points || [];

    const allPoints = [[0, 0]];

    rooms.forEach((room) => {
        room.points.forEach((point) => allPoints.push(point));
    });

    activePath.forEach((point) => allPoints.push(point));

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
                selectedRoomIds.add(room.id);
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
    const rooms = data.rooms || [];
    const wasMappingActive = mappingActive;

    mappingActive = Boolean(mapping.active);
    mappingClosed = Boolean(mapping.closed);
    currentCleaningState = cleaning.state || "idle";

    if (wasMappingActive && !mappingActive) {
        resetDriveControls();
    }

    document.body.classList.toggle(
        "mapping-locked",
        !mappingActive
    );

    mappingState.textContent = mappingActive
        ? "Recording"
        : mappingClosed
            ? "Ready To Save"
            : "Idle";

    if (cleaning.room) {
        autoStatus.textContent =
            cleaning.message + ": " + cleaning.room;
    } else if (mapping.warning) {
        autoStatus.textContent = mapping.warning;
    } else {
        autoStatus.textContent = cleaning.message || "Ready";
    }

    startMappingButton.disabled =
        mappingActive || currentCleaningState === "running";

    backAtDockButton.disabled = !mappingActive;
    saveRoomButton.disabled = !mappingClosed;
    cancelMappingButton.disabled = !(mappingActive || mappingClosed);

    startCleanButton.disabled =
        !AUTONOMOUS_DRIVING_ENABLED
        || rooms.length === 0
        || currentCleaningState === "running";

    startCleanButton.textContent = AUTONOMOUS_DRIVING_ENABLED
        ? "Clean Selected"
        : "Clean Disabled";

    dockNowButton.disabled = mappingActive;

    renderMap(data);
    renderRoomList(rooms);
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
        const data = await postJson("/mapping/start");
        applyState(data);
        autoStatus.textContent = "Mapping active";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


backAtDockButton.addEventListener("click", async () => {
    try {
        resetDriveControls();
        const data = await postJson("/mapping/back-at-dock");
        applyState(data);
        autoStatus.textContent = "Dock reset";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


saveRoomButton.addEventListener("click", async () => {
    try {
        const data = await postJson("/mapping/save", {
            name: roomNameInput.value
        });

        roomNameInput.value = "";
        applyState(data);
        autoStatus.textContent = "Room saved";
    } catch (error) {
        autoStatus.textContent = error.message;
        console.error(error);
    }
});


cancelMappingButton.addEventListener("click", async () => {
    try {
        resetDriveControls();
        const data = await postJson("/mapping/cancel");
        applyState(data);
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
    if (mappingActive || currentCleaningState === "running") {
        emergencyStop();
    }
});


window.addEventListener("pagehide", () => {
    if (
        mappingActive
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

    refreshState().catch((error) => {
        autoStatus.textContent = error.message;
        console.error(error);
    });
}, 1000);


refreshState().catch((error) => {
    autoStatus.textContent = error.message;
    console.error(error);
});

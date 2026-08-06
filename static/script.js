const statusText = document.getElementById("status");

const speedSlider = document.getElementById("speed");
const speedValue = document.getElementById("speed-value");

const stopButton = document.getElementById("stop");
const vacuumButton = document.getElementById("vacuum");

const throttleValue = document.getElementById("throttle-value");
const steeringValue = document.getElementById("steering-value");

const leftWheelValue =
    document.getElementById("left-wheel-value");

const rightWheelValue =
    document.getElementById("right-wheel-value");


let maximumSpeed = Number(speedSlider.value);

let targetThrottle = 0;
let targetSteering = 0;

let smoothThrottle = 0;
let smoothSteering = 0;

let vacuumOn = false;

let lastSentLeft = null;
let lastSentRight = null;

let requestRunning = false;
let pendingRequest = false;

let lastTransmissionTime = 0;

const SEND_INTERVAL_MS = 50;
const SMOOTHING = 0.22;
const DEAD_ZONE = 0.06;


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
    "throttle-joystick",
    "vertical",
    (value) => {
        targetThrottle = value;

        throttleValue.textContent =
            Math.round(value * 100) + "%";
    }
);


const resetSteering = createJoystick(
    "steering-joystick",
    "horizontal",
    (value) => {
        targetSteering = value;

        steeringValue.textContent =
            Math.round(value * 100) + "%";
    }
);


function calculateWheelSpeeds() {
    const throttle =
        smoothThrottle * maximumSpeed;

    const steering =
        smoothSteering * maximumSpeed;

    /*
        Positive steering means turn right.

        Right turn:
        left wheel speeds up
        right wheel slows down
    */
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


async function sendWheelSpeeds(left, right) {
    if (requestRunning) {
        pendingRequest = true;
        return;
    }

    if (
        left === lastSentLeft
        && right === lastSentRight
    ) {
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

        lastSentLeft = left;
        lastSentRight = right;

        statusText.textContent =
            left === 0 && right === 0
                ? "Stopped"
                : "Driving";
    } catch (error) {
        statusText.textContent = "Connection error";
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

        sendWheelSpeeds(
            speeds.left,
            speeds.right
        );
    }

    requestAnimationFrame(controlLoop);
}


requestAnimationFrame(controlLoop);


async function emergencyStop() {
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

    try {
        await fetch("/stop", {
            method: "POST",
            keepalive: true
        });

        lastSentLeft = 0;
        lastSentRight = 0;

        statusText.textContent = "Stopped";
    } catch (error) {
        statusText.textContent =
            "Emergency stop request failed";

        console.error(error);
    }
}


stopButton.addEventListener(
    "click",
    emergencyStop
);


speedSlider.addEventListener("input", () => {
    maximumSpeed = Number(speedSlider.value);
    speedValue.textContent = maximumSpeed;
});


speedSlider.addEventListener("change", async () => {
    try {
        const response = await fetch("/speed", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                speed: maximumSpeed
            })
        });

        if (!response.ok) {
            throw new Error("Speed update failed");
        }

        statusText.textContent =
            `Maximum speed: ${maximumSpeed} mm/s`;
    } catch (error) {
        statusText.textContent =
            "Could not update speed";

        console.error(error);
    }
});


vacuumButton.addEventListener("click", async () => {
    const nextState = !vacuumOn;

    try {
        const response = await fetch(
            nextState
                ? "/vacuum/on"
                : "/vacuum/off",
            {
                method: "POST"
            }
        );

        if (!response.ok) {
            throw new Error(
                "Cleaning request failed"
            );
        }

        vacuumOn = nextState;

        vacuumButton.textContent =
            vacuumOn
                ? "Cleaning OFF"
                : "Cleaning ON";

        statusText.textContent =
            vacuumOn
                ? "Cleaning motors running"
                : "Cleaning motors stopped";
    } catch (error) {
        statusText.textContent =
            "Cleaning command failed";

        console.error(error);
    }
});


window.addEventListener(
    "blur",
    emergencyStop
);


window.addEventListener(
    "pagehide",
    emergencyStop
);


document.addEventListener(
    "visibilitychange",
    () => {
        if (document.hidden) {
            emergencyStop();
        }
    }
);

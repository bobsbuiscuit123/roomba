from flask import Flask, jsonify, redirect, render_template, request, url_for

from roomba import RoombaController


app = Flask(__name__)
roomba = RoombaController(speed=300)
roomba_started = False


@app.route("/")
def start_page():
    return render_template("start.html")


@app.post("/start")
def start_roomba():
    global roomba_started

    roomba.start()
    roomba_started = True

    return jsonify({
        "ok": True,
        "redirect": url_for("controls"),
    })


@app.route("/controls")
def controls():
    if not roomba_started:
        return redirect(url_for("start_page"))

    return render_template("index.html")


@app.post("/drive")
def drive():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    data = request.get_json(silent=True) or {}

    try:
        left = int(data.get("left", 0))
        right = int(data.get("right", 0))
    except (TypeError, ValueError):
        return jsonify({
            "ok": False,
            "error": "Invalid wheel speeds",
        }), 400

    left = max(-roomba.speed, min(roomba.speed, left))
    right = max(-roomba.speed, min(roomba.speed, right))

    roomba.drive_wheels(
        right_speed=right,
        left_speed=left,
    )

    return jsonify({
        "ok": True,
        "left": left,
        "right": right,
    })


@app.post("/stop")
def stop():
    roomba.stop()
    return jsonify({"ok": True})


@app.post("/speed")
def speed():
    data = request.get_json(silent=True) or {}

    try:
        new_speed = int(data.get("speed", 300))
    except (TypeError, ValueError):
        return jsonify({
            "ok": False,
            "error": "Invalid speed",
        }), 400

    roomba.set_speed(new_speed)

    return jsonify({
        "ok": True,
        "speed": roomba.speed,
    })


@app.post("/vacuum/on")
def vacuum_on():
    roomba.vacuum_on()
    return jsonify({"ok": True})


@app.post("/vacuum/off")
def vacuum_off():
    roomba.vacuum_off()
    return jsonify({"ok": True})


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False)
    finally:
        roomba.close()

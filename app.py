from flask import Flask, jsonify, redirect, render_template, request, url_for

from autonomous import AutonomousCleaner, RoomMapStore
from roomba import RoombaController


app = Flask(__name__)
roomba = RoombaController(speed=300)
room_map = RoomMapStore()
cleaner = AutonomousCleaner(roomba, room_map)
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
        "redirect": url_for("modes"),
    })


@app.route("/modes")
def modes():
    if not roomba_started:
        return redirect(url_for("start_page"))

    return render_template("modes.html")


@app.route("/controls")
def controls():
    if not roomba_started:
        return redirect(url_for("start_page"))

    return render_template("index.html")


@app.route("/autonomous")
def autonomous():
    if not roomba_started:
        return redirect(url_for("start_page"))

    return render_template("autonomous.html")


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
    room_map.record_drive(left, right)

    return jsonify({
        "ok": True,
        "left": left,
        "right": right,
    })


@app.post("/stop")
def stop():
    room_map.record_drive(0, 0)
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


@app.get("/autonomous/state")
def autonomous_state():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    state = room_map.state()
    state["cleaning"] = cleaner.status()
    state["ok"] = True

    return jsonify(state)


@app.post("/mapping/start")
def mapping_start():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    if cleaner.status()["state"] == "running":
        return jsonify({
            "ok": False,
            "error": "Stop autonomous cleaning first",
        }), 409

    try:
        roomba.stop()
        state = room_map.start_mapping()
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    state["ok"] = True
    state["cleaning"] = cleaner.status()

    return jsonify(state)


@app.post("/mapping/back-at-dock")
def mapping_back_at_dock():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    try:
        room_map.record_drive(0, 0)
        roomba.stop()
        state = room_map.finish_mapping_at_dock()
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    state["ok"] = True
    state["cleaning"] = cleaner.status()

    return jsonify(state)


@app.post("/mapping/save")
def mapping_save():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    data = request.get_json(silent=True) or {}

    try:
        room = room_map.save_room(str(data.get("name", "")))
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    state = room_map.state()
    state["ok"] = True
    state["room"] = room
    state["cleaning"] = cleaner.status()

    return jsonify(state)


@app.post("/mapping/cancel")
def mapping_cancel():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    room_map.record_drive(0, 0)
    roomba.stop()
    state = room_map.cancel_mapping()
    state["ok"] = True
    state["cleaning"] = cleaner.status()

    return jsonify(state)


@app.delete("/rooms/<room_id>")
def room_delete(room_id):
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    try:
        room_map.delete_room(room_id)
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 404

    state = room_map.state()
    state["ok"] = True
    state["cleaning"] = cleaner.status()

    return jsonify(state)


@app.post("/clean/start")
def clean_start():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    data = request.get_json(silent=True) or {}
    room_ids = data.get("rooms", [])

    if not isinstance(room_ids, list):
        return jsonify({
            "ok": False,
            "error": "Invalid room list",
        }), 400

    if room_map.state()["mapping"]["active"]:
        return jsonify({
            "ok": False,
            "error": "Finish or cancel mapping first",
        }), 409

    try:
        status = cleaner.start([str(room_id) for room_id in room_ids])
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    return jsonify({
        "ok": True,
        "cleaning": status,
    })


@app.post("/clean/stop")
def clean_stop():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    status = cleaner.stop()

    return jsonify({
        "ok": True,
        "cleaning": status,
    })


@app.post("/dock")
def dock():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    status = cleaner.send_to_dock()

    return jsonify({
        "ok": True,
        "cleaning": status,
    })


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False)
    finally:
        roomba.close()

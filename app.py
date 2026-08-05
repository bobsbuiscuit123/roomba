from flask import (
    Flask,
    Response,
    jsonify,
    redirect,
    render_template,
    request,
    send_file,
    url_for,
)

from autonomous import AutonomousCleaner, RoomMapStore, build_cleaning_preview_route
from camera import CameraStream
from roomba import RoombaController
from teach_routes import TeachRouteReplayer, TeachRouteStore


app = Flask(__name__)
roomba = RoombaController(speed=300)
room_map = RoomMapStore()
cleaner = AutonomousCleaner(roomba, room_map)
camera = CameraStream()
teach_routes = TeachRouteStore()
teach_replayer = TeachRouteReplayer(roomba, teach_routes)
roomba_started = False


def is_motion_recording():
    return room_map.is_mapping_active() or teach_routes.is_active()


def read_recording_motion_delta():
    if not is_motion_recording():
        return None

    try:
        return roomba.read_distance_angle()
    except Exception:
        return None


def clear_motion_delta() -> None:
    try:
        roomba.read_distance_angle()
    except Exception:
        pass


def record_motion_drive(left: int, right: int) -> None:
    motor_delta = read_recording_motion_delta()
    keyframe_jpeg = None

    if teach_routes.needs_keyframe():
        keyframe_jpeg = camera.get_jpeg()

    room_map.record_drive(left, right, motor_delta)
    teach_routes.record_drive(left, right, motor_delta, keyframe_jpeg)


@app.route("/")
def start_page():
    return render_template("start.html")


@app.post("/start")
def start_roomba():
    global roomba_started

    cleaner.stop()
    teach_replayer.stop()
    room_map.cancel_mapping()
    teach_routes.cancel()
    roomba.start()
    roomba.stop()
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

    if (
        (
            cleaner.status()["state"] in {"running", "docking"}
            or teach_replayer.is_busy()
        )
        and (left != 0 or right != 0)
    ):
        return jsonify({
            "ok": False,
            "error": "The Roomba is using the wheels",
        }), 409

    left = max(-roomba.speed, min(roomba.speed, left))
    right = max(-roomba.speed, min(roomba.speed, right))

    roomba.drive_wheels(
        right_speed=right,
        left_speed=left,
    )
    record_motion_drive(left, right)

    return jsonify({
        "ok": True,
        "left": left,
        "right": right,
    })


@app.post("/stop")
def stop():
    teach_replayer.stop()
    roomba.stop()
    record_motion_drive(0, 0)
    cleaner.reset_to_idle()
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
    routes = {}
    route_errors = {}

    for room in state["rooms"]:
        try:
            routes[room["id"]] = build_cleaning_preview_route(room["points"])
        except ValueError as error:
            route_errors[room["id"]] = str(error)

    state["routes"] = routes
    state["route_errors"] = route_errors
    state["cleaning"] = cleaner.status()
    state["teach"] = teach_routes.state()
    state["teach_replay"] = teach_replayer.status()
    state["camera"] = camera.status()
    state["ok"] = True

    return jsonify(state)


@app.get("/camera/status")
def camera_status():
    return jsonify(camera.status())


@app.get("/camera/stream")
def camera_stream():
    status = camera.status()

    if not status["ok"]:
        return jsonify(status), 503

    return Response(
        camera.frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.post("/teach/start")
def teach_start():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    if room_map.state()["mapping"]["active"]:
        return jsonify({
            "ok": False,
            "error": "Finish or cancel mapping first",
        }), 409

    if cleaner.status()["state"] in {"running", "docking"}:
        return jsonify({
            "ok": False,
            "error": "Stop the current Roomba action first",
        }), 409

    if teach_replayer.is_busy():
        return jsonify({
            "ok": False,
            "error": "Stop route replay first",
        }), 409

    data = request.get_json(silent=True) or {}

    try:
        roomba.stop()
        clear_motion_delta()
        state = teach_routes.start(str(data.get("name", "")))
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    return jsonify({
        "ok": True,
        "teach": state,
        "camera": camera.status(),
    })


@app.post("/teach/finish")
def teach_finish():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    data = request.get_json(silent=True) or {}

    try:
        roomba.stop()
        record_motion_drive(0, 0)
        route = teach_routes.finish(str(data.get("name", "")))
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    return jsonify({
        "ok": True,
        "route": route,
        "teach": teach_routes.state(),
    })


@app.post("/teach/cancel")
def teach_cancel():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    roomba.stop()
    record_motion_drive(0, 0)

    return jsonify({
        "ok": True,
        "teach": teach_routes.cancel(),
    })


@app.delete("/teach-routes/<route_id>")
def teach_route_delete(route_id):
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    if teach_replayer.is_busy():
        return jsonify({
            "ok": False,
            "error": "Stop route replay before deleting routes",
        }), 409

    try:
        teach_routes.delete(route_id)
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 404

    return jsonify({
        "ok": True,
        "teach": teach_routes.state(),
    })


@app.get("/teach-routes/<route_id>/keyframes/<filename>")
def teach_route_keyframe(route_id, filename):
    try:
        path = teach_routes.keyframe_path(route_id, filename)
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 404

    return send_file(path, mimetype="image/jpeg")


@app.post("/teach-routes/<route_id>/go")
def teach_route_go(route_id):
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    if room_map.state()["mapping"]["active"]:
        return jsonify({
            "ok": False,
            "error": "Finish or cancel mapping first",
        }), 409

    if teach_routes.is_active():
        return jsonify({
            "ok": False,
            "error": "Finish or cancel teach route first",
        }), 409

    if cleaner.status()["state"] in {"running", "docking"}:
        return jsonify({
            "ok": False,
            "error": "Stop the current Roomba action first",
        }), 409

    try:
        roomba.stop()
        roomba.vacuum_off()
        status = teach_replayer.start(route_id)
    except ValueError as error:
        return jsonify({
            "ok": False,
            "error": str(error),
        }), 400

    return jsonify({
        "ok": True,
        "teach_replay": status,
    })


@app.post("/teach-replay/stop")
def teach_replay_stop():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    return jsonify({
        "ok": True,
        "teach_replay": teach_replayer.stop(),
    })


@app.post("/teach-replay/heartbeat")
def teach_replay_heartbeat():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    return jsonify({
        "ok": True,
        "teach_replay": teach_replayer.heartbeat(),
    })


@app.post("/mapping/start")
def mapping_start():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    if cleaner.status()["state"] in {"running", "docking"}:
        return jsonify({
            "ok": False,
            "error": "Stop the current Roomba action first",
        }), 409

    if teach_replayer.is_busy():
        return jsonify({
            "ok": False,
            "error": "Stop route replay first",
        }), 409

    if teach_routes.is_active():
        return jsonify({
            "ok": False,
            "error": "Finish or cancel teach route first",
        }), 409

    try:
        roomba.stop()
        clear_motion_delta()
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
        roomba.stop()
        state = room_map.finish_mapping_at_dock(
            read_recording_motion_delta()
        )
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

    roomba.stop()
    record_motion_drive(0, 0)
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

    if teach_routes.is_active():
        return jsonify({
            "ok": False,
            "error": "Finish or cancel teach route first",
        }), 409

    if teach_replayer.is_busy():
        return jsonify({
            "ok": False,
            "error": "Stop route replay first",
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


@app.post("/clean/heartbeat")
def clean_heartbeat():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    return jsonify({
        "ok": True,
        "cleaning": cleaner.heartbeat(),
    })


@app.post("/dock")
def dock():
    if not roomba_started:
        return jsonify({
            "ok": False,
            "error": "Start the Roomba first",
        }), 409

    data = request.get_json(silent=True) or {}

    if room_map.state()["mapping"]["active"]:
        roomba.stop()
        record_motion_drive(0, 0)

        return jsonify({
            "ok": False,
            "error": "Finish or cancel mapping before docking",
        }), 409

    if teach_routes.is_active():
        roomba.stop()
        record_motion_drive(0, 0)

        return jsonify({
            "ok": False,
            "error": "Finish or cancel teach route before docking",
        }), 409

    if teach_replayer.is_busy():
        return jsonify({
            "ok": False,
            "error": "Stop route replay before docking",
        }), 409

    if data.get("confirm") is not True:
        roomba.stop()
        cleaner.reset_to_idle()

        return jsonify({
            "ok": False,
            "error": "Docking must be confirmed",
        }), 400

    cleaner.stop()
    status = cleaner.send_to_dock()

    return jsonify({
        "ok": True,
        "cleaning": status,
    })


@app.post("/safety/stop")
def safety_stop():
    teach_replayer.stop()
    roomba.stop()
    record_motion_drive(0, 0)
    room_map.cancel_mapping()
    teach_routes.cancel()
    cleaner.stop()

    return jsonify({
        "ok": True,
        "cleaning": cleaner.status(),
    })


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False)
    finally:
        teach_replayer.stop()
        camera.close()
        roomba.close()

import copy
import json
import os
import shutil
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from autonomous import (
    _distance,
    _integrate_distance_angle,
    _integrate_pose,
    _pose_point,
    _round_point,
)


TEACH_ROUTES_ENV_VAR = "TEACH_ROUTES_PATH"
TEACH_DATA_DIR_NAME = ".roomba_web"
TEACH_ROUTES_FILE_NAME = "teach_routes.json"
TEACH_KEYFRAME_DIR_NAME = "teach_keyframes"
TEACH_KEYFRAME_INTERVAL_SECONDS = 1.0
TEACH_KEYFRAME_DISTANCE_MM = 250.0
TEACH_REPLAY_MAX_SAMPLE_SECONDS = 0.45
TEACH_REPLAY_MIN_SAMPLE_SECONDS = 0.02
TEACH_REPLAY_FINAL_SAMPLE_SECONDS = 0.12
TEACH_REPLAY_SLEEP_SLICE_SECONDS = 0.05
TEACH_REPLAY_HEARTBEAT_TIMEOUT_SECONDS = 2.5
TEACH_REPLAY_MAX_WHEEL_SPEED = 500


class TeachRouteReplayCancelled(Exception):
    pass


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _default_teach_routes_path() -> Path:
    configured_path = os.environ.get(TEACH_ROUTES_ENV_VAR)

    if configured_path:
        return Path(configured_path).expanduser()

    return Path.home() / TEACH_DATA_DIR_NAME / TEACH_ROUTES_FILE_NAME


class TeachRouteStore:
    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = (path or _default_teach_routes_path()).expanduser()
        self.keyframe_dir = self.path.parent / TEACH_KEYFRAME_DIR_NAME
        self.lock = threading.Lock()
        self.routes: list[dict[str, Any]] = []
        self.active: Optional[dict[str, Any]] = None
        self.load()

    def load(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.keyframe_dir.mkdir(parents=True, exist_ok=True)

        if not self.path.exists():
            return

        with self.path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        self.routes = data.get("routes", [])

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.keyframe_dir.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_name(self.path.name + ".tmp")
        data = {
            "routes": self.routes,
            "updated_at": _now_iso(),
        }

        with temp_path.open("w", encoding="utf-8") as file:
            json.dump(data, file, indent=2)
            file.write("\n")

        temp_path.replace(self.path)

    def start(self, name: str = "") -> dict[str, Any]:
        with self.lock:
            if self.active is not None:
                raise ValueError("A teach route is already recording")

            route_id = uuid.uuid4().hex[:10]
            self.active = {
                "id": route_id,
                "name": name.strip() or "Route " + str(len(self.routes) + 1),
                "active": True,
                "started_at": _now_iso(),
                "started_monotonic": time.monotonic(),
                "last_update": time.monotonic(),
                "last_left": 0.0,
                "last_right": 0.0,
                "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "points": [[0.0, 0.0]],
                "samples": [],
                "keyframes": [],
                "last_keyframe_at": 0.0,
                "last_keyframe_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "motor_updates": 0,
                "fallback_updates": 0,
            }

            return self.state_locked()

    def is_active(self) -> bool:
        with self.lock:
            return self.active is not None

    def needs_keyframe(self) -> bool:
        with self.lock:
            if self.active is None:
                return False

            now = time.monotonic()
            pose = self.active["pose"]
            last_pose = self.active["last_keyframe_pose"]

            return (
                now - self.active["last_keyframe_at"]
                >= TEACH_KEYFRAME_INTERVAL_SECONDS
                or _distance(_pose_point(pose), _pose_point(last_pose))
                >= TEACH_KEYFRAME_DISTANCE_MM
            )

    def _record_keyframe_locked(self, jpeg: bytes) -> None:
        if self.active is None:
            return

        route_id = self.active["id"]
        keyframe_index = len(self.active["keyframes"])
        route_dir = self.keyframe_dir / route_id
        route_dir.mkdir(parents=True, exist_ok=True)
        filename = f"{keyframe_index:04d}.jpg"
        path = route_dir / filename
        path.write_bytes(jpeg)
        pose = dict(self.active["pose"])
        keyframe = {
            "index": keyframe_index,
            "path": str(path),
            "url": f"/teach-routes/{route_id}/keyframes/{filename}",
            "timestamp": time.monotonic() - self.active["started_monotonic"],
            "pose": {
                "x": round(pose["x"], 1),
                "y": round(pose["y"], 1),
                "theta": round(pose["theta"], 4),
            },
        }
        self.active["keyframes"].append(keyframe)
        self.active["last_keyframe_at"] = time.monotonic()
        self.active["last_keyframe_pose"] = pose

    def record_drive(
        self,
        left_speed: float,
        right_speed: float,
        motor_delta: Optional[tuple[int, int]] = None,
        keyframe_jpeg: Optional[bytes] = None,
    ) -> None:
        with self.lock:
            if self.active is None:
                return

            now = time.monotonic()
            last_update = self.active["last_update"] or now
            duration = min(max(now - last_update, 0.0), 0.4)

            if motor_delta is None:
                _integrate_pose(
                    self.active["pose"],
                    self.active["last_left"],
                    self.active["last_right"],
                    duration,
                )
                self.active["fallback_updates"] += 1
                distance_mm = None
                angle_degrees = None
            else:
                distance_mm, angle_degrees = motor_delta
                _integrate_distance_angle(
                    self.active["pose"],
                    distance_mm,
                    angle_degrees,
                )
                self.active["motor_updates"] += 1

            point = _pose_point(self.active["pose"])

            if _distance(self.active["points"][-1], point) >= 35:
                self.active["points"].append(_round_point(point))

            sample = {
                "t": round(now - self.active["started_monotonic"], 3),
                "left": float(left_speed),
                "right": float(right_speed),
                "distance_mm": distance_mm,
                "angle_degrees": angle_degrees,
                "pose": {
                    "x": round(self.active["pose"]["x"], 1),
                    "y": round(self.active["pose"]["y"], 1),
                    "theta": round(self.active["pose"]["theta"], 4),
                },
            }
            self.active["samples"].append(sample)
            self.active["last_update"] = now
            self.active["last_left"] = float(left_speed)
            self.active["last_right"] = float(right_speed)

            if keyframe_jpeg is not None:
                self._record_keyframe_locked(keyframe_jpeg)

    def finish(self, name: str = "") -> dict[str, Any]:
        with self.lock:
            if self.active is None:
                raise ValueError("No teach route is recording")

            if name.strip():
                self.active["name"] = name.strip()

            if len(self.active["samples"]) < 2:
                raise ValueError("Drive the route before saving")

            route = dict(self.active)
            route.pop("active", None)
            route.pop("started_monotonic", None)
            route.pop("last_update", None)
            route.pop("last_left", None)
            route.pop("last_right", None)
            route.pop("last_keyframe_at", None)
            route.pop("last_keyframe_pose", None)
            route["finished_at"] = _now_iso()
            route["duration_seconds"] = round(
                time.monotonic() - self.active["started_monotonic"],
                2,
            )
            route["sample_count"] = len(route["samples"])
            route["keyframe_count"] = len(route["keyframes"])
            route["point_count"] = len(route["points"])

            total_distance = 0.0

            for first, second in zip(route["points"], route["points"][1:]):
                total_distance += _distance(first, second)

            route["distance_mm"] = round(total_distance, 1)
            self.routes.append(route)
            self.active = None
            self.save()
            return route

    def cancel(self) -> dict[str, Any]:
        with self.lock:
            self.active = None
            return self.state_locked()

    def delete(self, route_id: str) -> None:
        with self.lock:
            original_count = len(self.routes)
            self.routes = [
                route for route in self.routes if route["id"] != route_id
            ]

            if len(self.routes) == original_count:
                raise ValueError("Teach route was not found")

            shutil.rmtree(
                self.keyframe_dir / Path(route_id).name,
                ignore_errors=True,
            )
            self.save()

    def get_route(self, route_id: str) -> dict[str, Any]:
        with self.lock:
            for route in self.routes:
                if route["id"] == route_id:
                    return copy.deepcopy(route)

        raise ValueError("Teach route was not found")

    def state_locked(self) -> dict[str, Any]:
        active = None

        if self.active is not None:
            active = {
                "id": self.active["id"],
                "name": self.active["name"],
                "started_at": self.active["started_at"],
                "pose": {
                    "x": round(self.active["pose"]["x"], 1),
                    "y": round(self.active["pose"]["y"], 1),
                    "theta": round(self.active["pose"]["theta"], 4),
                },
                "points": list(self.active["points"]),
                "sample_count": len(self.active["samples"]),
                "keyframe_count": len(self.active["keyframes"]),
                "motor_updates": self.active["motor_updates"],
                "fallback_updates": self.active["fallback_updates"],
            }

        return {
            "active": active,
            "routes": [
                {
                    "id": route["id"],
                    "name": route["name"],
                    "created_at": route["started_at"],
                    "duration_seconds": route.get("duration_seconds", 0),
                    "distance_mm": route.get("distance_mm", 0),
                    "sample_count": route.get("sample_count", 0),
                    "keyframe_count": route.get("keyframe_count", 0),
                    "points": route.get("points", []),
                    "keyframes": route.get("keyframes", [])[:3],
                }
                for route in self.routes
            ],
        }

    def state(self) -> dict[str, Any]:
        with self.lock:
            return self.state_locked()

    def keyframe_path(self, route_id: str, filename: str) -> Path:
        safe_route_id = Path(route_id).name
        safe_filename = Path(filename).name
        path = (self.keyframe_dir / safe_route_id / safe_filename).resolve()
        keyframe_root = self.keyframe_dir.resolve()

        if (
            keyframe_root not in path.parents
            or not path.exists()
            or not path.is_file()
        ):
            raise ValueError("Keyframe was not found")

        return path


class TeachRouteReplayer:
    def __init__(self, roomba: Any, route_store: TeachRouteStore) -> None:
        self.roomba = roomba
        self.route_store = route_store
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.last_heartbeat = time.monotonic()
        self.status_data: dict[str, Any] = {
            "state": "idle",
            "message": "Ready",
            "route_id": None,
            "route_name": None,
            "progress": 0.0,
            "sample_index": 0,
            "sample_count": 0,
            "pose": None,
        }

    def is_busy(self) -> bool:
        with self.lock:
            return self.status_data["state"] == "running"

    def status(self) -> dict[str, Any]:
        with self.lock:
            return copy.deepcopy(self.status_data)

    def heartbeat(self) -> dict[str, Any]:
        with self.lock:
            self.last_heartbeat = time.monotonic()
            return copy.deepcopy(self.status_data)

    def _check_heartbeat(self) -> None:
        if (
            time.monotonic() - self.last_heartbeat
            > TEACH_REPLAY_HEARTBEAT_TIMEOUT_SECONDS
        ):
            raise RuntimeError("Route replay heartbeat lost")

    def _join_worker(self) -> None:
        thread = self.thread

        if (
            thread
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.0)

    def _set_status(
        self,
        state: str,
        message: str,
        route: Optional[dict[str, Any]] = None,
        progress: float = 0.0,
        sample_index: int = 0,
        sample_count: int = 0,
        pose: Optional[dict[str, Any]] = None,
    ) -> None:
        route_id = None
        route_name = None

        if route is not None:
            route_id = route.get("id")
            route_name = route.get("name")

        with self.lock:
            self.status_data = {
                "state": state,
                "message": message,
                "route_id": route_id,
                "route_name": route_name,
                "progress": round(max(0.0, min(1.0, progress)), 3),
                "sample_index": sample_index,
                "sample_count": sample_count,
                "pose": copy.deepcopy(pose),
            }

    def _sample_speeds(self, sample: dict[str, Any]) -> tuple[int, int]:
        def clamp(speed: int) -> int:
            return max(
                -TEACH_REPLAY_MAX_WHEEL_SPEED,
                min(TEACH_REPLAY_MAX_WHEEL_SPEED, speed),
            )

        try:
            left = int(round(float(sample.get("left", 0))))
            right = int(round(float(sample.get("right", 0))))
        except (TypeError, ValueError):
            left = 0
            right = 0

        return (
            clamp(left),
            clamp(right),
        )

    def _sample_wait(
        self,
        sample: dict[str, Any],
        next_sample: Optional[dict[str, Any]],
    ) -> float:
        if next_sample is None:
            return TEACH_REPLAY_FINAL_SAMPLE_SECONDS

        try:
            duration = float(next_sample.get("t", 0)) - float(
                sample.get("t", 0)
            )
        except (TypeError, ValueError):
            duration = TEACH_REPLAY_MIN_SAMPLE_SECONDS

        return min(
            TEACH_REPLAY_MAX_SAMPLE_SECONDS,
            max(TEACH_REPLAY_MIN_SAMPLE_SECONDS, duration),
        )

    def _sleep_with_cancel(self, duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)

        while True:
            if self.cancel_event.is_set():
                raise TeachRouteReplayCancelled()

            self._check_heartbeat()
            remaining = deadline - time.monotonic()

            if remaining <= 0:
                return

            time.sleep(min(remaining, TEACH_REPLAY_SLEEP_SLICE_SECONDS))

    def start(self, route_id: str) -> dict[str, Any]:
        route = self.route_store.get_route(route_id)
        samples = route.get("samples", [])

        if len(samples) < 2:
            raise ValueError("Teach route does not have enough samples")

        with self.lock:
            if self.status_data["state"] == "running":
                raise ValueError("A teach route is already replaying")

            self.cancel_event.clear()
            self.last_heartbeat = time.monotonic()
            self.thread = threading.Thread(
                target=self._run,
                args=(route,),
                daemon=True,
            )
            self.status_data = {
                "state": "running",
                "message": "Starting route replay",
                "route_id": route["id"],
                "route_name": route["name"],
                "progress": 0.0,
                "sample_index": 0,
                "sample_count": len(samples),
                "pose": samples[0].get("pose"),
            }
            self.thread.start()

            return copy.deepcopy(self.status_data)

    def stop(self) -> dict[str, Any]:
        self.cancel_event.set()

        try:
            self.roomba.stop()
            self._join_worker()
            self.roomba.stop()
        finally:
            if self.status()["state"] == "running":
                self._set_status("idle", "Stopped")

        return self.status()

    def _run(self, route: dict[str, Any]) -> None:
        samples = route.get("samples", [])
        sample_count = len(samples)

        try:
            self.roomba.vacuum_off()

            for index, sample in enumerate(samples):
                if self.cancel_event.is_set():
                    raise TeachRouteReplayCancelled()

                self._check_heartbeat()
                left, right = self._sample_speeds(sample)
                pose = sample.get("pose")

                self.roomba.drive_wheels(
                    right_speed=right,
                    left_speed=left,
                )
                self._set_status(
                    "running",
                    "Replaying route",
                    route,
                    progress=(index + 1) / sample_count,
                    sample_index=index + 1,
                    sample_count=sample_count,
                    pose=pose,
                )

                next_sample = (
                    samples[index + 1]
                    if index + 1 < sample_count
                    else None
                )
                self._sleep_with_cancel(
                    self._sample_wait(sample, next_sample)
                )

            self.roomba.stop()
            self._set_status(
                "idle",
                "Arrived at route end",
                route,
                progress=1.0,
                sample_index=sample_count,
                sample_count=sample_count,
                pose=samples[-1].get("pose"),
            )
        except TeachRouteReplayCancelled:
            self.roomba.stop()
            self._set_status("idle", "Stopped", route)
        except Exception as error:
            try:
                self.roomba.stop()
            finally:
                self._set_status(
                    "error",
                    f"Route replay failed: {error}",
                    route,
                )
        finally:
            with self.lock:
                if self.thread is threading.current_thread():
                    self.thread = None

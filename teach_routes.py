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
TEACH_LANDMARK_DIR_NAME = "landmarks"
TEACH_KEYFRAME_INTERVAL_SECONDS = 1.0
TEACH_KEYFRAME_DISTANCE_MM = 250.0
TEACH_LANDMARK_PATCH_FRACTION = 0.28
TEACH_LANDMARK_MIN_PATCH_PIXELS = 48
TEACH_REPLAY_MAX_SAMPLE_SECONDS = 0.45
TEACH_REPLAY_MIN_SAMPLE_SECONDS = 0.02
TEACH_REPLAY_FINAL_SAMPLE_SECONDS = 0.12
TEACH_REPLAY_SLEEP_SLICE_SECONDS = 0.05
TEACH_REPLAY_HEARTBEAT_TIMEOUT_SECONDS = 2.5
TEACH_REPLAY_MAX_WHEEL_SPEED = 500
VISION_MATCH_INTERVAL_SECONDS = 0.25
VISION_LANDMARK_EARLY_SECONDS = 0.35
VISION_LANDMARK_LATE_SECONDS = 2.0
VISION_MATCH_MIN_SCORE = 0.45
VISION_CORRECTION_GAIN = 180.0
VISION_CORRECTION_MAX_SPEED = 55
VISION_REACQUIRE_TIMEOUT_SECONDS = 30.0
VISION_REACQUIRE_SWEEP_SPEED = 55
VISION_REACQUIRE_SWEEP_SECONDS = 0.18
VISION_REACQUIRE_SETTLE_SECONDS = 0.08
VISION_REVERSE_AFTER_SCAN_COUNT = 4
VISION_REVERSE_SPEED_SCALE = 0.65
VISION_REVERSE_MAX_SAMPLE_SECONDS = 0.18
VISION_REVERSE_STEP_SECONDS = 0.75
BUMPER_CHECK_INTERVAL_SECONDS = 0.08
OBSTACLE_REVERSE_SPEED = -130
OBSTACLE_TURN_SPEED = 125
OBSTACLE_ARC_FAST_SPEED = 135
OBSTACLE_ARC_SLOW_SPEED = 55
OBSTACLE_REVERSE_SECONDS = 0.35
OBSTACLE_TURN_SECONDS = 0.35
OBSTACLE_ARC_SECONDS = 0.55


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
            try:
                self._route_index_locked(route_id)
            except ValueError:
                raise ValueError("Teach route was not found")

            self.routes = [
                route for route in self.routes if route["id"] != route_id
            ]
            shutil.rmtree(
                self.keyframe_dir / Path(route_id).name,
                ignore_errors=True,
            )
            self.save()

    def _route_index_locked(self, route_id: str) -> int:
        for index, route in enumerate(self.routes):
            if route["id"] == route_id:
                return index

        raise ValueError("Teach route was not found")

    def _crop_landmark_patch(
        self,
        keyframe_path: Path,
        patch_path: Path,
        x_fraction: float,
        y_fraction: float,
        width_fraction: float,
        height_fraction: float,
    ) -> dict[str, int]:
        try:
            import cv2
        except Exception as error:
            raise ValueError("OpenCV is required for landmark labels") from error

        image = cv2.imread(str(keyframe_path))

        if image is None:
            raise ValueError("Could not read keyframe image")

        height, width = image.shape[:2]
        patch_width = int(round(width * width_fraction))
        patch_height = int(round(height * height_fraction))
        patch_width = max(TEACH_LANDMARK_MIN_PATCH_PIXELS, patch_width)
        patch_height = max(TEACH_LANDMARK_MIN_PATCH_PIXELS, patch_height)
        patch_width = min(patch_width, width)
        patch_height = min(patch_height, height)
        center_x = int(round(x_fraction * width))
        center_y = int(round(y_fraction * height))
        left = max(0, min(width - patch_width, center_x - patch_width // 2))
        top = max(0, min(height - patch_height, center_y - patch_height // 2))
        crop = image[top:top + patch_height, left:left + patch_width]

        patch_path.parent.mkdir(parents=True, exist_ok=True)

        if not cv2.imwrite(str(patch_path), crop):
            raise ValueError("Could not save landmark crop")

        return {
            "image_width": width,
            "image_height": height,
            "patch_x": left,
            "patch_y": top,
            "patch_width": patch_width,
            "patch_height": patch_height,
        }

    def add_landmark(
        self,
        route_id: str,
        keyframe_index: int,
        x_fraction: float,
        y_fraction: float,
        name: str = "",
        patch_fraction: float = TEACH_LANDMARK_PATCH_FRACTION,
        width_fraction: Optional[float] = None,
        height_fraction: Optional[float] = None,
    ) -> dict[str, Any]:
        try:
            x_fraction = float(x_fraction)
            y_fraction = float(y_fraction)
            patch_fraction = float(patch_fraction)
            width_fraction = (
                float(width_fraction)
                if width_fraction is not None
                else patch_fraction
            )
            height_fraction = (
                float(height_fraction)
                if height_fraction is not None
                else patch_fraction
            )
        except (TypeError, ValueError) as error:
            raise ValueError("Invalid landmark position") from error

        if not 0 <= x_fraction <= 1 or not 0 <= y_fraction <= 1:
            raise ValueError("Landmark position must be inside the image")

        width_fraction = max(0.08, min(0.9, width_fraction))
        height_fraction = max(0.08, min(0.9, height_fraction))

        with self.lock:
            route_index = self._route_index_locked(route_id)
            route = self.routes[route_index]
            keyframes = route.get("keyframes", [])
            keyframe = next(
                (
                    frame
                    for frame in keyframes
                    if int(frame.get("index", -1)) == int(keyframe_index)
                ),
                None,
            )

            if keyframe is None:
                raise ValueError("Keyframe was not found")

            keyframe_path = Path(keyframe.get("path", ""))

            if not keyframe_path.exists():
                raise ValueError("Keyframe image is missing")

            landmark_id = uuid.uuid4().hex[:10]
            safe_route_id = Path(route_id).name
            filename = f"{landmark_id}.jpg"
            patch_path = (
                self.keyframe_dir
                / safe_route_id
                / TEACH_LANDMARK_DIR_NAME
                / filename
            )
            crop_info = self._crop_landmark_patch(
                keyframe_path,
                patch_path,
                float(x_fraction),
                float(y_fraction),
                width_fraction,
                height_fraction,
            )
            landmarks = route.setdefault("landmarks", [])
            landmark = {
                "id": landmark_id,
                "name": name.strip() or f"Landmark {len(landmarks) + 1}",
                "created_at": _now_iso(),
                "keyframe_index": int(keyframe_index),
                "keyframe_url": keyframe.get("url", ""),
                "timestamp": keyframe.get("timestamp", 0),
                "pose": keyframe.get("pose"),
                "x": round(float(x_fraction), 4),
                "y": round(float(y_fraction), 4),
                "width": round(width_fraction, 4),
                "height": round(height_fraction, 4),
                "patch_url": (
                    f"/teach-routes/{safe_route_id}/landmarks/{filename}"
                ),
                "patch_path": str(patch_path),
                **crop_info,
            }
            landmarks.append(landmark)
            route["landmark_count"] = len(landmarks)
            self.save()
            return copy.deepcopy(landmark)

    def delete_landmark(self, route_id: str, landmark_id: str) -> None:
        with self.lock:
            route_index = self._route_index_locked(route_id)
            route = self.routes[route_index]
            landmarks = route.get("landmarks", [])
            kept_landmarks = [
                landmark
                for landmark in landmarks
                if landmark.get("id") != landmark_id
            ]

            if len(kept_landmarks) == len(landmarks):
                raise ValueError("Landmark was not found")

            removed = next(
                landmark
                for landmark in landmarks
                if landmark.get("id") == landmark_id
            )
            patch_path = Path(removed.get("patch_path", ""))

            if patch_path.exists() and patch_path.is_file():
                patch_path.unlink()

            route["landmarks"] = kept_landmarks
            route["landmark_count"] = len(kept_landmarks)
            self.save()

    def get_route(self, route_id: str) -> dict[str, Any]:
        with self.lock:
            return copy.deepcopy(self.routes[self._route_index_locked(route_id)])

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
                    "landmark_count": route.get(
                        "landmark_count",
                        len(route.get("landmarks", [])),
                    ),
                    "points": route.get("points", []),
                    "keyframes": route.get("keyframes", []),
                    "landmarks": route.get("landmarks", []),
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

    def landmark_path(self, route_id: str, filename: str) -> Path:
        safe_route_id = Path(route_id).name
        safe_filename = Path(filename).name
        path = (
            self.keyframe_dir
            / safe_route_id
            / TEACH_LANDMARK_DIR_NAME
            / safe_filename
        ).resolve()
        landmark_root = (
            self.keyframe_dir
            / safe_route_id
            / TEACH_LANDMARK_DIR_NAME
        ).resolve()

        if (
            landmark_root not in path.parents
            or not path.exists()
            or not path.is_file()
        ):
            raise ValueError("Landmark image was not found")

        return path


class TeachRouteReplayer:
    def __init__(
        self,
        roomba: Any,
        route_store: TeachRouteStore,
        camera: Optional[Any] = None,
    ) -> None:
        self.roomba = roomba
        self.route_store = route_store
        self.camera = camera
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.last_heartbeat = time.monotonic()
        self.last_vision_check = 0.0
        self.active_landmark_id: Optional[str] = None
        self.current_vision_correction = 0
        self.template_cache: dict[str, Any] = {}
        self.last_bumper_check = 0.0
        self.obstacle_count = 0
        self.obstacle_data: dict[str, Any] = {
            "active": False,
            "count": 0,
            "message": "",
            "bump_left": False,
            "bump_right": False,
        }
        self.vision_data: dict[str, Any] = self._vision_status(
            False,
            "No route replay",
        )
        self.status_data: dict[str, Any] = {
            "state": "idle",
            "message": "Ready",
            "route_id": None,
            "route_name": None,
            "progress": 0.0,
            "sample_index": 0,
            "sample_count": 0,
            "pose": None,
            "vision": copy.deepcopy(self.vision_data),
            "obstacle": copy.deepcopy(self.obstacle_data),
        }

    def _vision_status(
        self,
        enabled: bool,
        message: str,
        landmark: Optional[dict[str, Any]] = None,
        score: float = 0.0,
        offset: float = 0.0,
        correction: int = 0,
        seen: bool = False,
    ) -> dict[str, Any]:
        return {
            "enabled": enabled,
            "message": message,
            "landmark_id": landmark.get("id") if landmark else None,
            "landmark_name": landmark.get("name") if landmark else None,
            "score": round(float(score), 3),
            "offset": round(float(offset), 3),
            "correction": int(correction),
            "seen": bool(seen),
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
        vision: Optional[dict[str, Any]] = None,
        obstacle: Optional[dict[str, Any]] = None,
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
                "vision": copy.deepcopy(vision or self.vision_data),
                "obstacle": copy.deepcopy(obstacle or self.obstacle_data),
            }

    def _clamp_wheel_speed(self, speed: int) -> int:
        return max(
            -TEACH_REPLAY_MAX_WHEEL_SPEED,
            min(TEACH_REPLAY_MAX_WHEEL_SPEED, int(speed)),
        )

    def _sample_speeds(self, sample: dict[str, Any]) -> tuple[int, int]:
        try:
            left = int(round(float(sample.get("left", 0))))
            right = int(round(float(sample.get("right", 0))))
        except (TypeError, ValueError):
            left = 0
            right = 0

        return (
            self._clamp_wheel_speed(left),
            self._clamp_wheel_speed(right),
        )

    def _select_landmark(
        self,
        route: dict[str, Any],
        sample: dict[str, Any],
    ) -> Optional[dict[str, Any]]:
        landmarks = route.get("landmarks", [])

        if not landmarks:
            return None

        try:
            sample_time = float(sample.get("t", 0))
        except (TypeError, ValueError):
            sample_time = 0.0

        candidate_landmarks = []

        for landmark in landmarks:
            try:
                landmark_time = float(landmark.get("timestamp", 0))
            except (TypeError, ValueError):
                landmark_time = 0.0

            if (
                sample_time >= landmark_time - VISION_LANDMARK_EARLY_SECONDS
                and sample_time <= landmark_time + VISION_LANDMARK_LATE_SECONDS
            ):
                candidate_landmarks.append((landmark_time, landmark))

        if not candidate_landmarks:
            return None

        return min(
            candidate_landmarks,
            key=lambda item: abs(item[0] - sample_time),
        )[1]

    def _select_recovery_landmark(
        self,
        route: dict[str, Any],
        sample_index: int,
    ) -> Optional[dict[str, Any]]:
        landmarks = route.get("landmarks", [])
        samples = route.get("samples", [])

        if not landmarks or not samples:
            return None

        safe_index = max(0, min(sample_index, len(samples) - 1))

        try:
            sample_time = float(samples[safe_index].get("t", 0))
        except (TypeError, ValueError):
            sample_time = 0.0

        previous_landmarks = []
        all_landmarks = []

        for landmark in landmarks:
            try:
                landmark_time = float(landmark.get("timestamp", 0))
            except (TypeError, ValueError):
                landmark_time = 0.0

            all_landmarks.append((
                abs(landmark_time - sample_time),
                landmark_time,
                landmark,
            ))

            if landmark_time <= sample_time + VISION_LANDMARK_LATE_SECONDS:
                previous_landmarks.append((landmark_time, landmark))

        if previous_landmarks:
            return max(previous_landmarks, key=lambda item: item[0])[1]

        if not all_landmarks:
            return None

        return min(all_landmarks, key=lambda item: item[0])[2]

    def _load_cv2(self):
        if self.camera is not None and hasattr(self.camera, "_load_cv2"):
            cv2 = self.camera._load_cv2()

            if cv2 is not None:
                return cv2

        try:
            import cv2
        except Exception:
            return None

        return cv2

    def _decode_camera_frame(self):
        if self.camera is None:
            return None

        if hasattr(self.camera, "get_frame"):
            frame = self.camera.get_frame()

            if frame is not None:
                return frame

        cv2 = self._load_cv2()

        if cv2 is None:
            return None

        jpeg = self.camera.get_jpeg()

        if jpeg is None:
            return None

        try:
            import numpy as np
        except Exception:
            return None

        image_data = np.frombuffer(jpeg, dtype=np.uint8)
        return cv2.imdecode(image_data, cv2.IMREAD_COLOR)

    def _load_landmark_template(
        self,
        landmark: dict[str, Any],
    ) -> Optional[Any]:
        cv2 = self._load_cv2()

        if cv2 is None:
            return None

        patch_path = str(landmark.get("patch_path", ""))

        if not patch_path:
            return None

        if patch_path in self.template_cache:
            return self.template_cache[patch_path]

        template = cv2.imread(patch_path, cv2.IMREAD_GRAYSCALE)

        if template is None:
            return None

        self.template_cache[patch_path] = template
        return template

    def _match_landmark(
        self,
        frame: Any,
        landmark: dict[str, Any],
    ) -> Optional[dict[str, Any]]:
        cv2 = self._load_cv2()
        template = self._load_landmark_template(landmark)

        if cv2 is None or template is None or frame is None:
            return None

        frame_height, frame_width = frame.shape[:2]
        template_height, template_width = template.shape[:2]

        if (
            template_width >= frame_width
            or template_height >= frame_height
        ):
            return None

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(
            frame_gray,
            template,
            cv2.TM_CCOEFF_NORMED,
        )
        _, score, _, max_location = cv2.minMaxLoc(result)
        observed_x = (max_location[0] + template_width / 2) / frame_width
        observed_y = (max_location[1] + template_height / 2) / frame_height
        expected_x = float(landmark.get("x", 0.5))
        expected_y = float(landmark.get("y", 0.5))

        return {
            "score": float(score),
            "observed_x": observed_x,
            "observed_y": observed_y,
            "expected_x": expected_x,
            "expected_y": expected_y,
            "offset_x": observed_x - expected_x,
            "offset_y": observed_y - expected_y,
        }

    def _match_live_landmark(
        self,
        landmark: dict[str, Any],
    ) -> Optional[dict[str, Any]]:
        frame = self._decode_camera_frame()
        return self._match_landmark(frame, landmark)

    def _set_correction_from_match(
        self,
        landmark: dict[str, Any],
        match: dict[str, Any],
        message: str = "Correcting from landmark",
    ) -> None:
        correction = int(round(
            match["offset_x"] * VISION_CORRECTION_GAIN
        ))
        correction = max(
            -VISION_CORRECTION_MAX_SPEED,
            min(VISION_CORRECTION_MAX_SPEED, correction),
        )
        self.current_vision_correction = correction
        self.vision_data = self._vision_status(
            True,
            message,
            landmark,
            score=match["score"],
            offset=match["offset_x"],
            correction=correction,
            seen=True,
        )

    def _match_is_locked(self, match: Optional[dict[str, Any]]) -> bool:
        return bool(match and match["score"] >= VISION_MATCH_MIN_SCORE)

    def _sample_time_delta(
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

        return max(TEACH_REPLAY_MIN_SAMPLE_SECONDS, duration)

    def _reverse_route_step(
        self,
        route: dict[str, Any],
        reverse_index: int,
    ) -> int:
        samples = route.get("samples", [])

        if reverse_index <= 0 or not samples:
            return reverse_index

        self.current_vision_correction = 0
        self.vision_data = self._vision_status(
            True,
            "Backtracking to landmark",
        )
        self._set_status(
            "running",
            "Backtracking to landmark",
            route,
        )

        spent = 0.0
        index = min(reverse_index, len(samples) - 1)

        while index >= 0 and spent < VISION_REVERSE_STEP_SECONDS:
            if self.cancel_event.is_set():
                raise TeachRouteReplayCancelled()

            self._check_heartbeat()
            sample = samples[index]
            next_sample = (
                samples[index + 1]
                if index + 1 < len(samples)
                else None
            )
            left, right = self._sample_speeds(sample)
            duration = min(
                VISION_REVERSE_MAX_SAMPLE_SECONDS,
                self._sample_time_delta(sample, next_sample),
                VISION_REVERSE_STEP_SECONDS - spent,
            )

            index -= 1

            if duration <= 0 or (left == 0 and right == 0):
                continue

            self._drive_for(
                int(round(-left * VISION_REVERSE_SPEED_SCALE)),
                int(round(-right * VISION_REVERSE_SPEED_SCALE)),
                duration,
            )
            spent += duration

        self.roomba.stop()
        self._motion_sleep(VISION_REACQUIRE_SETTLE_SECONDS)
        return max(0, index)

    def _search_for_landmark(
        self,
        route: dict[str, Any],
        landmark: dict[str, Any],
        current_sample_index: int,
    ) -> int:
        deadline = time.monotonic() + VISION_REACQUIRE_TIMEOUT_SECONDS
        direction = 1
        best_score = 0.0
        scan_count = 0
        replay_index = current_sample_index
        reverse_index = min(
            max(0, current_sample_index - 1),
            max(0, len(route.get("samples", [])) - 1),
        )

        self.current_vision_correction = 0
        self.vision_data = self._vision_status(
            True,
            "Searching for landmark",
            landmark,
        )
        self._set_status(
            "running",
            "Searching for landmark",
            route,
        )
        self.roomba.stop()
        self._motion_sleep(VISION_REACQUIRE_SETTLE_SECONDS)

        while time.monotonic() < deadline:
            if self.cancel_event.is_set():
                raise TeachRouteReplayCancelled()

            self._check_heartbeat()
            obstacle_index = self._check_obstacle(
                route,
                reverse_index,
                recover=False,
            )

            if obstacle_index is not None:
                reverse_index = obstacle_index
                replay_index = min(replay_index, obstacle_index)
                continue

            match = self._match_live_landmark(landmark)

            if match is not None:
                best_score = max(best_score, float(match["score"]))

            if self._match_is_locked(match):
                self.roomba.stop()
                self._set_correction_from_match(
                    landmark,
                    match,
                    "Landmark reacquired",
                )
                return replay_index

            self.vision_data = self._vision_status(
                True,
                "Scanning for landmark",
                landmark,
                score=best_score,
                offset=match["offset_x"] if match else 0.0,
            )
            self._set_status(
                "running",
                "Scanning for landmark",
                route,
            )

            self._drive_for(
                -VISION_REACQUIRE_SWEEP_SPEED * direction,
                VISION_REACQUIRE_SWEEP_SPEED * direction,
                VISION_REACQUIRE_SWEEP_SECONDS,
            )
            self.roomba.stop()
            self._motion_sleep(VISION_REACQUIRE_SETTLE_SECONDS)
            direction *= -1
            scan_count += 1

            if (
                scan_count % VISION_REVERSE_AFTER_SCAN_COUNT == 0
                and reverse_index > 0
            ):
                reverse_index = self._reverse_route_step(
                    route,
                    reverse_index,
                )
                replay_index = min(replay_index, reverse_index)

        self.current_vision_correction = 0
        self.vision_data = self._vision_status(
            True,
            "Landmark lost",
            landmark,
            score=best_score,
        )
        raise RuntimeError(
            "Landmark lost during route replay: "
            + str(landmark.get("name", "landmark"))
        )

    def _apply_vision_correction(
        self,
        route: dict[str, Any],
        sample: dict[str, Any],
        sample_index: int,
        left: int,
        right: int,
    ) -> tuple[int, int, Optional[int]]:
        landmarks = route.get("landmarks", [])

        if not landmarks:
            self.current_vision_correction = 0
            self.vision_data = self._vision_status(
                False,
                "No landmarks",
            )
            return left, right, None

        if self.camera is None:
            self.current_vision_correction = 0
            self.vision_data = self._vision_status(
                False,
                "No camera",
            )
            return left, right, None

        if left < 35 or right < 35:
            self.current_vision_correction = 0
            self.vision_data = self._vision_status(
                True,
                "Vision paused for turn",
            )
            return left, right, None

        landmark = self._select_landmark(route, sample)

        if landmark is None:
            self.active_landmark_id = None
            self.current_vision_correction = 0
            self.vision_data = self._vision_status(
                True,
                "Waiting for landmark",
            )
            return left, right, None

        now = time.monotonic()
        landmark_id = str(landmark.get("id", ""))
        should_check_vision = (
            landmark_id != self.active_landmark_id
            or now - self.last_vision_check >= VISION_MATCH_INTERVAL_SECONDS
        )

        if should_check_vision:
            self.active_landmark_id = landmark_id
            self.last_vision_check = now
            match = self._match_live_landmark(landmark)

            if match is None:
                self.vision_data = self._vision_status(
                    True,
                    "Vision unavailable",
                    landmark,
                )
                replay_index = self._search_for_landmark(
                    route,
                    landmark,
                    sample_index,
                )

                if replay_index < sample_index:
                    return left, right, replay_index
            elif match["score"] < VISION_MATCH_MIN_SCORE:
                self.vision_data = self._vision_status(
                    True,
                    "Landmark not locked",
                    landmark,
                    score=match["score"],
                    offset=match["offset_x"],
                )
                replay_index = self._search_for_landmark(
                    route,
                    landmark,
                    sample_index,
                )

                if replay_index < sample_index:
                    return left, right, replay_index
            else:
                self._set_correction_from_match(
                    landmark,
                    match,
                )

        correction = self.current_vision_correction

        return (
            self._clamp_wheel_speed(left + correction),
            self._clamp_wheel_speed(right - correction),
            None,
        )

    def _motion_sleep(self, duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)

        while True:
            if self.cancel_event.is_set():
                raise TeachRouteReplayCancelled()

            self._check_heartbeat()
            remaining = deadline - time.monotonic()

            if remaining <= 0:
                return

            time.sleep(min(remaining, TEACH_REPLAY_SLEEP_SLICE_SECONDS))

    def _read_bumper_state(self) -> Optional[dict[str, Any]]:
        if not hasattr(self.roomba, "read_bumps_wheel_drops"):
            return None

        try:
            return self.roomba.read_bumps_wheel_drops()
        except Exception:
            return None

    def _drive_for(
        self,
        left: int,
        right: int,
        duration: float,
    ) -> None:
        self.roomba.drive_wheels(
            right_speed=self._clamp_wheel_speed(right),
            left_speed=self._clamp_wheel_speed(left),
        )
        self._motion_sleep(duration)

    def _set_obstacle_status(
        self,
        active: bool,
        message: str,
        bump_state: Optional[dict[str, Any]] = None,
    ) -> None:
        self.obstacle_data = {
            "active": active,
            "count": self.obstacle_count,
            "message": message,
            "bump_left": bool(
                bump_state and bump_state.get("bump_left")
            ),
            "bump_right": bool(
                bump_state and bump_state.get("bump_right")
            ),
        }

    def _backtrack_after_bump(
        self,
        route: dict[str, Any],
        bump_state: dict[str, Any],
        current_sample_index: int,
        message: str,
    ) -> int:
        samples = route.get("samples", [])
        sample_count = len(samples)
        safe_index = 0

        if sample_count:
            safe_index = max(
                0,
                min(current_sample_index, sample_count - 1),
            )

        self.current_vision_correction = 0

        if bump_state.get("wheel_drop"):
            raise RuntimeError("Wheel drop detected during route replay")

        self._set_obstacle_status(True, message, bump_state)
        self._set_status(
            "running",
            message,
            route,
            sample_index=safe_index,
            sample_count=sample_count,
            pose=samples[safe_index].get("pose") if samples else None,
        )
        self.roomba.stop()
        self._motion_sleep(0.08)

        reverse_index = self._reverse_route_step(route, safe_index)
        self.roomba.stop()
        self._motion_sleep(VISION_REACQUIRE_SETTLE_SECONDS)
        return reverse_index

    def _recover_from_obstacle(
        self,
        route: dict[str, Any],
        bump_state: dict[str, Any],
        current_sample_index: int,
    ) -> int:
        reverse_index = self._backtrack_after_bump(
            route,
            bump_state,
            current_sample_index,
            "Bump detected; backtracking",
        )
        landmark = self._select_recovery_landmark(route, reverse_index)

        if landmark is None:
            self._set_obstacle_status(
                False,
                "Bump backed up; no landmarks",
                bump_state,
            )
            return reverse_index

        replay_index = self._search_for_landmark(
            route,
            landmark,
            reverse_index,
        )
        self._set_obstacle_status(
            False,
            "Obstacle cleared; landmark reacquired",
            bump_state,
        )
        return min(reverse_index, replay_index)

    def _check_obstacle(
        self,
        route: dict[str, Any],
        current_sample_index: int = 0,
        recover: bool = True,
    ) -> Optional[int]:
        now = time.monotonic()

        if now - self.last_bumper_check < BUMPER_CHECK_INTERVAL_SECONDS:
            return None

        self.last_bumper_check = now
        bump_state = self._read_bumper_state()

        if not bump_state:
            return None

        if bump_state.get("wheel_drop"):
            raise RuntimeError("Wheel drop detected during route replay")

        if bump_state.get("bump"):
            self.obstacle_count += 1

            if recover:
                return self._recover_from_obstacle(
                    route,
                    bump_state,
                    current_sample_index,
                )

            return self._backtrack_after_bump(
                route,
                bump_state,
                current_sample_index,
                "Bump while searching; backtracking",
            )

        return None

    def _sample_wait(
        self,
        sample: dict[str, Any],
        next_sample: Optional[dict[str, Any]],
    ) -> float:
        return min(
            TEACH_REPLAY_MAX_SAMPLE_SECONDS,
            self._sample_time_delta(sample, next_sample),
        )

    def _sleep_with_cancel(
        self,
        duration: float,
        route: Optional[dict[str, Any]] = None,
        current_sample_index: int = 0,
    ) -> Optional[int]:
        deadline = time.monotonic() + max(0.0, duration)

        while True:
            if self.cancel_event.is_set():
                raise TeachRouteReplayCancelled()

            self._check_heartbeat()

            if route is not None:
                replay_index = self._check_obstacle(
                    route,
                    current_sample_index,
                )

                if replay_index is not None:
                    return replay_index

            remaining = deadline - time.monotonic()

            if remaining <= 0:
                return None

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
            self.last_vision_check = 0.0
            self.current_vision_correction = 0
            self.active_landmark_id = None
            self.last_bumper_check = 0.0
            self.obstacle_count = 0
            self.obstacle_data = {
                "active": False,
                "count": 0,
                "message": "",
                "bump_left": False,
                "bump_right": False,
            }
            self.template_cache = {}
            self.vision_data = self._vision_status(
                bool(route.get("landmarks", [])),
                (
                    "Vision ready"
                    if route.get("landmarks", [])
                    else "No landmarks"
                ),
            )
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
                "vision": copy.deepcopy(self.vision_data),
                "obstacle": copy.deepcopy(self.obstacle_data),
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
            index = 0

            while index < sample_count:
                if self.cancel_event.is_set():
                    raise TeachRouteReplayCancelled()

                self._check_heartbeat()
                replay_index = self._check_obstacle(route, index)

                if replay_index is not None:
                    index = max(0, min(replay_index, sample_count - 1))
                    continue

                sample = samples[index]
                left, right = self._sample_speeds(sample)
                left, right, replay_index = self._apply_vision_correction(
                    route,
                    sample,
                    index,
                    left,
                    right,
                )

                if replay_index is not None:
                    index = max(0, min(replay_index, sample_count - 1))
                    continue

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
                replay_index = self._sleep_with_cancel(
                    self._sample_wait(sample, next_sample),
                    route,
                    index,
                )

                if replay_index is not None:
                    index = max(0, min(replay_index, sample_count - 1))
                    continue

                index += 1

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

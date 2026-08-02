import json
import math
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


WHEEL_BASE_MM = 235.0
MIN_POINT_SPACING_MM = 80.0
MIN_ROOM_AREA_MM2 = 20_000.0


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi

    while angle < -math.pi:
        angle += 2 * math.pi

    return angle


def _distance(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _round_point(point: list[float]) -> list[float]:
    return [round(point[0], 1), round(point[1], 1)]


def _pose_point(pose: dict[str, float]) -> list[float]:
    return [pose["x"], pose["y"]]


def _polygon_area(points: list[list[float]]) -> float:
    if len(points) < 4:
        return 0.0

    area = 0.0

    for index in range(len(points) - 1):
        x1, y1 = points[index]
        x2, y2 = points[index + 1]
        area += x1 * y2 - x2 * y1

    return abs(area) / 2


def _bounds(points: list[list[float]]) -> dict[str, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]

    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
    }


def _boxes_overlap(
    first: dict[str, float],
    second: dict[str, float],
) -> bool:
    return not (
        first["max_x"] <= second["min_x"]
        or first["min_x"] >= second["max_x"]
        or first["max_y"] <= second["min_y"]
        or first["min_y"] >= second["max_y"]
    )


def _orientation(
    a: list[float],
    b: list[float],
    c: list[float],
) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (
        b[1] - a[1]
    ) * (c[0] - a[0])


def _on_segment(
    a: list[float],
    b: list[float],
    c: list[float],
) -> bool:
    return (
        min(a[0], c[0]) - 1e-7 <= b[0] <= max(a[0], c[0]) + 1e-7
        and min(a[1], c[1]) - 1e-7 <= b[1] <= max(a[1], c[1]) + 1e-7
        and abs(_orientation(a, b, c)) <= 1e-7
    )


def _same_point(a: list[float], b: list[float]) -> bool:
    return _distance(a, b) <= 1e-5


def _segments_cross_or_overlap(
    a: list[float],
    b: list[float],
    c: list[float],
    d: list[float],
) -> bool:
    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)

    if o1 * o2 < 0 and o3 * o4 < 0:
        return True

    endpoint_touch = (
        _same_point(a, c)
        or _same_point(a, d)
        or _same_point(b, c)
        or _same_point(b, d)
    )

    if endpoint_touch:
        return False

    return (
        (abs(o1) <= 1e-7 and _on_segment(a, c, b))
        or (abs(o2) <= 1e-7 and _on_segment(a, d, b))
        or (abs(o3) <= 1e-7 and _on_segment(c, a, d))
        or (abs(o4) <= 1e-7 and _on_segment(c, b, d))
    )


def _point_on_boundary(
    point: list[float],
    polygon: list[list[float]],
) -> bool:
    for index in range(len(polygon) - 1):
        if _on_segment(polygon[index], point, polygon[index + 1]):
            return True

    return False


def _point_in_polygon(
    point: list[float],
    polygon: list[list[float]],
) -> bool:
    if _point_on_boundary(point, polygon):
        return False

    inside = False
    x, y = point

    for index in range(len(polygon) - 1):
        x1, y1 = polygon[index]
        x2, y2 = polygon[index + 1]

        crosses_y = (y1 > y) != (y2 > y)

        if not crosses_y:
            continue

        x_intersection = x1 + (y - y1) * (x2 - x1) / (y2 - y1)

        if x_intersection > x:
            inside = not inside

    return inside


def _polygons_overlap(
    first: list[list[float]],
    second: list[list[float]],
) -> bool:
    if not _boxes_overlap(_bounds(first), _bounds(second)):
        return False

    for first_index in range(len(first) - 1):
        for second_index in range(len(second) - 1):
            if _segments_cross_or_overlap(
                first[first_index],
                first[first_index + 1],
                second[second_index],
                second[second_index + 1],
            ):
                return True

    if any(_point_in_polygon(point, second) for point in first[:-1]):
        return True

    return any(_point_in_polygon(point, first) for point in second[:-1])


def _self_intersects(points: list[list[float]]) -> bool:
    segment_count = len(points) - 1

    for first_index in range(segment_count):
        for second_index in range(first_index + 1, segment_count):
            adjacent = abs(first_index - second_index) == 1
            closing_pair = first_index == 0 and second_index == segment_count - 1

            if adjacent or closing_pair:
                continue

            if _segments_cross_or_overlap(
                points[first_index],
                points[first_index + 1],
                points[second_index],
                points[second_index + 1],
            ):
                return True

    return False


def _thin_path(points: list[list[float]]) -> list[list[float]]:
    if not points:
        return [[0.0, 0.0], [0.0, 0.0]]

    thinned = [_round_point(points[0])]

    for point in points[1:]:
        rounded = _round_point(point)

        if _distance(thinned[-1], rounded) >= MIN_POINT_SPACING_MM:
            thinned.append(rounded)

    if not _same_point(thinned[-1], [0.0, 0.0]):
        thinned.append([0.0, 0.0])

    if not _same_point(thinned[0], thinned[-1]):
        thinned.append(thinned[0])

    return thinned


def _integrate_pose(
    pose: dict[str, float],
    left_speed: float,
    right_speed: float,
    duration: float,
    wheel_base_mm: float = WHEEL_BASE_MM,
) -> None:
    if duration <= 0:
        return

    velocity = (left_speed + right_speed) / 2
    angular_velocity = (left_speed - right_speed) / wheel_base_mm
    mid_heading = pose["theta"] + angular_velocity * duration / 2

    pose["x"] += velocity * math.sin(mid_heading) * duration
    pose["y"] += velocity * math.cos(mid_heading) * duration
    pose["theta"] = _normalize_angle(
        pose["theta"] + angular_velocity * duration
    )


def _line_polygon_intersections(
    polygon: list[list[float]],
    y_value: float,
) -> list[float]:
    intersections: list[float] = []

    for index in range(len(polygon) - 1):
        x1, y1 = polygon[index]
        x2, y2 = polygon[index + 1]

        if y1 == y2:
            continue

        lower = min(y1, y2)
        upper = max(y1, y2)

        if lower <= y_value < upper:
            ratio = (y_value - y1) / (y2 - y1)
            intersections.append(x1 + ratio * (x2 - x1))

    intersections.sort()
    return intersections


def build_coverage_route(
    polygon: list[list[float]],
    spacing_mm: float = 320.0,
) -> list[list[float]]:
    room_bounds = _bounds(polygon)
    y_value = room_bounds["min_y"] + spacing_mm / 2
    route: list[list[float]] = []
    left_to_right = True

    while y_value <= room_bounds["max_y"]:
        intersections = _line_polygon_intersections(polygon, y_value)

        for index in range(0, len(intersections) - 1, 2):
            first = [intersections[index], y_value]
            second = [intersections[index + 1], y_value]

            if left_to_right:
                route.extend([first, second])
            else:
                route.extend([second, first])

            left_to_right = not left_to_right

        y_value += spacing_mm

    if route:
        return [_round_point(point) for point in route]

    x_center = (room_bounds["min_x"] + room_bounds["max_x"]) / 2
    y_center = (room_bounds["min_y"] + room_bounds["max_y"]) / 2
    return [[round(x_center, 1), round(y_center, 1)]]


class RoomMapStore:
    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = path or Path(__file__).with_name("room_map.json")
        self.lock = threading.Lock()
        self.rooms: list[dict[str, Any]] = []
        self.mapping: dict[str, Any] = self._empty_mapping()
        self.load()

    def _empty_mapping(self) -> dict[str, Any]:
        return {
            "active": False,
            "closed": False,
            "started_at": None,
            "last_update": None,
            "last_left": 0.0,
            "last_right": 0.0,
            "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "path": [[0.0, 0.0]],
            "draft_points": [],
            "warning": "",
        }

    def load(self) -> None:
        if not self.path.exists():
            return

        with self.path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        self.rooms = data.get("rooms", [])

    def save(self) -> None:
        data = {
            "dock": {"x": 0, "y": 0, "heading": 0},
            "rooms": self.rooms,
            "updated_at": _now_iso(),
        }

        with self.path.open("w", encoding="utf-8") as file:
            json.dump(data, file, indent=2)

    def start_mapping(self) -> dict[str, Any]:
        with self.lock:
            self.mapping = self._empty_mapping()
            self.mapping["active"] = True
            self.mapping["started_at"] = _now_iso()
            self.mapping["last_update"] = time.monotonic()

            return self.state_locked()

    def cancel_mapping(self) -> dict[str, Any]:
        with self.lock:
            self.mapping = self._empty_mapping()
            return self.state_locked()

    def record_drive(self, left_speed: float, right_speed: float) -> None:
        with self.lock:
            if not self.mapping["active"]:
                return

            now = time.monotonic()
            last_update = self.mapping["last_update"] or now
            duration = min(max(now - last_update, 0.0), 0.4)

            _integrate_pose(
                self.mapping["pose"],
                self.mapping["last_left"],
                self.mapping["last_right"],
                duration,
            )

            point = _pose_point(self.mapping["pose"])

            if _distance(self.mapping["path"][-1], point) >= 35:
                self.mapping["path"].append(_round_point(point))

            self.mapping["last_update"] = now
            self.mapping["last_left"] = float(left_speed)
            self.mapping["last_right"] = float(right_speed)

    def finish_mapping_at_dock(self) -> dict[str, Any]:
        with self.lock:
            if not self.mapping["active"]:
                raise ValueError("No mapping session is active")

            now = time.monotonic()
            duration = min(
                max(now - (self.mapping["last_update"] or now), 0.0),
                0.4,
            )

            _integrate_pose(
                self.mapping["pose"],
                self.mapping["last_left"],
                self.mapping["last_right"],
                duration,
            )

            drift = _distance(_pose_point(self.mapping["pose"]), [0.0, 0.0])
            path = list(self.mapping["path"])
            path.append(_pose_point(self.mapping["pose"]))
            path.append([0.0, 0.0])
            draft_points = _thin_path(path)

            if len(draft_points) < 4:
                raise ValueError("Drive around the room edge before saving")

            if _polygon_area(draft_points) < MIN_ROOM_AREA_MM2:
                raise ValueError("Mapped room is too small to save")

            if _self_intersects(draft_points):
                raise ValueError("Mapped room crosses itself")

            self.mapping["active"] = False
            self.mapping["closed"] = True
            self.mapping["last_left"] = 0.0
            self.mapping["last_right"] = 0.0
            self.mapping["draft_points"] = draft_points
            self.mapping["warning"] = (
                "Odometry drifted "
                + str(round(drift))
                + " mm before dock reset"
                if drift > 350
                else ""
            )
            self.mapping["pose"] = {"x": 0.0, "y": 0.0, "theta": 0.0}

            return self.state_locked()

    def save_room(self, name: str) -> dict[str, Any]:
        with self.lock:
            if not self.mapping["closed"]:
                raise ValueError("Finish the mapping at the dock first")

            points = self.mapping["draft_points"]

            if not points:
                raise ValueError("There is no room draft to save")

            for room in self.rooms:
                if _polygons_overlap(points, room["points"]):
                    raise ValueError(
                        "This room overlaps with " + room["name"]
                    )

            room = {
                "id": uuid.uuid4().hex[:10],
                "name": name.strip() or "Room " + str(len(self.rooms) + 1),
                "points": points,
                "area_mm2": round(_polygon_area(points), 1),
                "bounds": _bounds(points),
                "created_at": _now_iso(),
            }

            self.rooms.append(room)
            self.mapping = self._empty_mapping()
            self.save()

            return room

    def delete_room(self, room_id: str) -> None:
        with self.lock:
            original_count = len(self.rooms)
            self.rooms = [
                room for room in self.rooms if room["id"] != room_id
            ]

            if len(self.rooms) == original_count:
                raise ValueError("Room was not found")

            self.save()

    def get_rooms(self, room_ids: list[str]) -> list[dict[str, Any]]:
        with self.lock:
            room_lookup = {room["id"]: room for room in self.rooms}
            rooms = []

            for room_id in room_ids:
                if room_id not in room_lookup:
                    raise ValueError("Room was not found")

                rooms.append(room_lookup[room_id])

            return rooms

    def reset_pose_to_dock(self) -> None:
        with self.lock:
            self.mapping["pose"] = {"x": 0.0, "y": 0.0, "theta": 0.0}

    def state_locked(self) -> dict[str, Any]:
        return {
            "rooms": self.rooms,
            "mapping": {
                "active": self.mapping["active"],
                "closed": self.mapping["closed"],
                "started_at": self.mapping["started_at"],
                "pose": {
                    "x": round(self.mapping["pose"]["x"], 1),
                    "y": round(self.mapping["pose"]["y"], 1),
                    "theta": round(self.mapping["pose"]["theta"], 4),
                },
                "path": list(self.mapping["path"]),
                "draft_points": list(self.mapping["draft_points"]),
                "warning": self.mapping["warning"],
            },
        }

    def state(self) -> dict[str, Any]:
        with self.lock:
            return self.state_locked()


class AutonomousCleaner:
    def __init__(
        self,
        roomba: Any,
        room_store: RoomMapStore,
        wheel_base_mm: float = WHEEL_BASE_MM,
    ) -> None:
        self.roomba = roomba
        self.room_store = room_store
        self.wheel_base_mm = wheel_base_mm
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.status_data: dict[str, Any] = {
            "state": "idle",
            "message": "Ready",
            "room": None,
            "pose": self._public_pose(),
        }

    def _public_pose(self) -> dict[str, float]:
        return {
            "x": round(self.pose["x"], 1),
            "y": round(self.pose["y"], 1),
            "theta": round(self.pose["theta"], 4),
        }

    def is_busy(self) -> bool:
        with self.lock:
            return self.status_data["state"] in {"running", "docking"}

    def status(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.status_data)

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
        room: Optional[str] = None,
    ) -> None:
        with self.lock:
            self.status_data = {
                "state": state,
                "message": message,
                "room": room,
                "pose": self._public_pose(),
            }

    def start(self, room_ids: list[str]) -> dict[str, Any]:
        rooms = self.room_store.get_rooms(room_ids)

        if not rooms:
            raise ValueError("Select at least one room")

        with self.lock:
            if self.status_data["state"] == "running":
                raise ValueError("Autonomous cleaning is already running")

            self.cancel_event.clear()
            self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
            self.thread = threading.Thread(
                target=self._run,
                args=(rooms,),
                daemon=True,
            )
            self.status_data = {
                "state": "running",
                "message": "Starting",
                "room": None,
                "pose": self._public_pose(),
            }
            self.thread.start()

            return dict(self.status_data)

    def stop(self) -> dict[str, Any]:
        self.cancel_event.set()

        try:
            self.roomba.stop()
            self.roomba.vacuum_off()
            self._join_worker()
            self.roomba.stop()
        finally:
            self._set_status("idle", "Stopped", None)

        return self.status()

    def send_to_dock(self) -> dict[str, Any]:
        self.cancel_event.set()
        self.roomba.stop()
        self.roomba.vacuum_off()
        self._join_worker()
        self.roomba.seek_dock()
        self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.room_store.reset_pose_to_dock()
        self._set_status("docking", "Docking command sent", None)
        return self.status()

    def _run(self, rooms: list[dict[str, Any]]) -> None:
        try:
            self.roomba.vacuum_on()

            for room in rooms:
                self._set_status("running", "Cleaning", room["name"])
                route = build_coverage_route(room["points"])

                for point in route:
                    self._drive_to(point, room["name"])

                    if self.cancel_event.is_set():
                        raise RuntimeError("Cleaning cancelled")

                self._drive_to([0.0, 0.0], room["name"])

            self.roomba.vacuum_off()
            self._set_status("docking", "Returning to dock", None)
            self._drive_to([0.0, 0.0], None)
            self.roomba.seek_dock()
            self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
            self.room_store.reset_pose_to_dock()
            self._set_status("docking", "Docking command sent", None)
        except RuntimeError:
            self.roomba.stop()
            self.roomba.vacuum_off()
            self._set_status("idle", "Stopped", None)
        except Exception as error:
            self.roomba.stop()
            self.roomba.vacuum_off()
            self._set_status("error", str(error), None)

    def _drive_to(
        self,
        target: list[float],
        room_name: Optional[str],
    ) -> None:
        dx = target[0] - self.pose["x"]
        dy = target[1] - self.pose["y"]
        distance = math.hypot(dx, dy)

        if distance < 60:
            return

        desired_heading = math.atan2(dx, dy)
        turn_angle = _normalize_angle(desired_heading - self.pose["theta"])

        if abs(turn_angle) > 0.05:
            self._turn(turn_angle, room_name)

        self._drive_straight(distance, room_name)

    def _turn(self, angle: float, room_name: Optional[str]) -> None:
        speed = 95
        left = speed if angle > 0 else -speed
        right = -speed if angle > 0 else speed
        angular_speed = abs((left - right) / self.wheel_base_mm)
        duration = abs(angle) / angular_speed

        self._move_for(left, right, duration, room_name)

    def _drive_straight(
        self,
        distance: float,
        room_name: Optional[str],
    ) -> None:
        speed = 150
        duration = distance / speed
        self._move_for(speed, speed, duration, room_name)

    def _move_for(
        self,
        left: int,
        right: int,
        duration: float,
        room_name: Optional[str],
    ) -> None:
        start = time.monotonic()
        last_update = start

        while True:
            if self.cancel_event.is_set():
                raise RuntimeError("Cleaning cancelled")

            now = time.monotonic()
            elapsed = now - start

            if elapsed >= duration:
                break

            step = min(now - last_update, duration - elapsed)

            if step > 0:
                _integrate_pose(
                    self.pose,
                    left,
                    right,
                    step,
                    self.wheel_base_mm,
                )

            self._set_status("running", "Cleaning", room_name)

            if self.cancel_event.is_set():
                raise RuntimeError("Cleaning cancelled")

            self.roomba.drive_wheels(right_speed=right, left_speed=left)
            last_update = now
            time.sleep(0.08)

        remaining = max(duration - (last_update - start), 0.0)

        if remaining > 0:
            _integrate_pose(
                self.pose,
                left,
                right,
                remaining,
                self.wheel_base_mm,
            )

        self.roomba.stop()
        self._set_status("running", "Cleaning", room_name)

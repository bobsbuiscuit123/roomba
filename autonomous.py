import json
import math
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


WHEEL_BASE_MM = 235.0
MIN_POINT_SPACING_MM = 80.0
MIN_ROOM_AREA_MM2 = 20_000.0
COVERAGE_SPACING_MM = 180.0
COVERAGE_EDGE_MARGIN_MM = 100.0
DOCK_APPROACH_DISTANCE_MM = 420.0
UNDOCK_DISTANCE_MM = 55.0
MAX_CLEANING_ROOM_SPAN_MM = 8_000.0
MAX_CLEANING_ROOM_AREA_MM2 = 60_000_000.0
MAX_CLEANING_ROUTE_POINTS = 500
MAX_CLEANING_SEGMENT_MM = 6_000.0
MAX_CLEANING_DURATION_SECONDS = 20 * 60
AUTONOMOUS_HEARTBEAT_TIMEOUT_SECONDS = 2.5
AUTONOMOUS_COMMAND_INTERVAL_SECONDS = 0.08
AUTONOMOUS_STRAIGHT_SPEED_MM_S = 85
AUTONOMOUS_TURN_SPEED_MM_S = 70
AUTONOMOUS_STRAIGHT_CHUNK_MM = 120
AUTONOMOUS_TURN_CHUNK_RAD = 0.28
AUTONOMOUS_WAYPOINT_TOLERANCE_MM = 90
AUTONOMOUS_HEADING_TOLERANCE_RAD = 0.08
SENSORLESS_TIME_LIMIT_MULTIPLIER = 1.55
ROOM_MAP_ENV_VAR = "ROOM_MAP_PATH"
ROOM_MAP_DIR_NAME = ".roomba_web"
ROOM_MAP_FILE_NAME = "room_map.json"


def _default_room_map_path() -> Path:
    configured_path = os.environ.get(ROOM_MAP_ENV_VAR)

    if configured_path:
        return Path(configured_path).expanduser()

    return Path.home() / ROOM_MAP_DIR_NAME / ROOM_MAP_FILE_NAME


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


def _polygon_centroid(points: list[list[float]]) -> list[float]:
    if len(points) < 4:
        return [0.0, 0.0]

    area_accumulator = 0.0
    x_accumulator = 0.0
    y_accumulator = 0.0

    for index in range(len(points) - 1):
        x1, y1 = points[index]
        x2, y2 = points[index + 1]
        cross = x1 * y2 - x2 * y1
        area_accumulator += cross
        x_accumulator += (x1 + x2) * cross
        y_accumulator += (y1 + y2) * cross

    if abs(area_accumulator) <= 1e-7:
        xs = [point[0] for point in points[:-1]]
        ys = [point[1] for point in points[:-1]]
        return [sum(xs) / len(xs), sum(ys) / len(ys)]

    return [
        x_accumulator / (3 * area_accumulator),
        y_accumulator / (3 * area_accumulator),
    ]


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


def _point_in_or_on_polygon(
    point: list[float],
    polygon: list[list[float]],
) -> bool:
    return _point_on_boundary(point, polygon) or _point_in_polygon(
        point,
        polygon,
    )


def _segment_stays_in_polygon(
    start: list[float],
    end: list[float],
    polygon: list[list[float]],
    spacing_mm: float = 120.0,
) -> bool:
    segment_length = _distance(start, end)
    sample_count = max(2, math.ceil(segment_length / spacing_mm))

    for index in range(sample_count + 1):
        ratio = index / sample_count
        point = [
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        ]

        if not _point_in_or_on_polygon(point, polygon):
            return False

    return True


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


def _loop_close_path(points: list[list[float]]) -> list[list[float]]:
    if len(points) < 2:
        return [[0.0, 0.0], [0.0, 0.0]]

    drift = points[-1]
    last_index = len(points) - 1
    corrected = []

    for index, point in enumerate(points):
        correction_ratio = index / last_index
        corrected.append([
            point[0] - drift[0] * correction_ratio,
            point[1] - drift[1] * correction_ratio,
        ])

    corrected[0] = [0.0, 0.0]
    corrected[-1] = [0.0, 0.0]

    return corrected


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


def _integrate_distance_angle(
    pose: dict[str, float],
    distance_mm: float,
    angle_degrees: float,
) -> None:
    # Roomba OI reports counter-clockwise positive. This map uses clockwise positive.
    angle = -math.radians(angle_degrees)
    mid_heading = pose["theta"] + angle / 2

    pose["x"] += distance_mm * math.sin(mid_heading)
    pose["y"] += distance_mm * math.cos(mid_heading)
    pose["theta"] = _normalize_angle(pose["theta"] + angle)


def _sweep_axis_angle(polygon: list[list[float]]) -> float:
    points = polygon[:-1]

    if len(points) < 2:
        return 0.0

    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    sxx = 0.0
    syy = 0.0
    sxy = 0.0

    for point in points:
        dx = point[0] - center_x
        dy = point[1] - center_y
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    if abs(sxx - syy) <= 1e-7 and abs(sxy) <= 1e-7:
        return 0.0

    return 0.5 * math.atan2(2 * sxy, sxx - syy)


def _to_sweep_space(
    point: list[float],
    angle: float,
) -> list[float]:
    cosine = math.cos(angle)
    sine = math.sin(angle)

    return [
        point[0] * cosine + point[1] * sine,
        -point[0] * sine + point[1] * cosine,
    ]


def _from_sweep_space(
    point: list[float],
    angle: float,
) -> list[float]:
    cosine = math.cos(angle)
    sine = math.sin(angle)

    return [
        point[0] * cosine - point[1] * sine,
        point[0] * sine + point[1] * cosine,
    ]


def _vertical_polygon_intersections(
    polygon: list[list[float]],
    x_value: float,
) -> list[float]:
    intersections: list[float] = []

    for index in range(len(polygon) - 1):
        x1, y1 = polygon[index]
        x2, y2 = polygon[index + 1]

        if x1 == x2:
            continue

        lower = min(x1, x2)
        upper = max(x1, x2)

        if lower <= x_value < upper:
            ratio = (x_value - x1) / (x2 - x1)
            intersections.append(y1 + ratio * (y2 - y1))

    intersections.sort()
    return intersections


def _build_coverage_segments(
    polygon: list[list[float]],
    spacing_mm: float = COVERAGE_SPACING_MM,
    edge_margin_mm: float = COVERAGE_EDGE_MARGIN_MM,
) -> list[tuple[list[float], list[float]]]:
    angle = _sweep_axis_angle(polygon)
    sweep_polygon = [
        _to_sweep_space(point, angle)
        for point in polygon
    ]
    room_bounds = _bounds(sweep_polygon)
    sweep_value = room_bounds["min_x"] + edge_margin_mm
    segments: list[tuple[list[float], list[float]]] = []

    while sweep_value <= room_bounds["max_x"] - edge_margin_mm:
        intersections = _vertical_polygon_intersections(
            sweep_polygon,
            sweep_value,
        )

        for index in range(0, len(intersections) - 1, 2):
            first_y = intersections[index]
            second_y = intersections[index + 1]
            span_width = second_y - first_y

            if span_width < edge_margin_mm * 1.5:
                continue

            if span_width > edge_margin_mm * 2:
                first = [sweep_value, first_y + edge_margin_mm]
                second = [sweep_value, second_y - edge_margin_mm]
            else:
                center = (first_y + second_y) / 2
                first = [sweep_value, center]
                second = [sweep_value, center]

            segment = (
                _round_point(_from_sweep_space(first, angle)),
                _round_point(_from_sweep_space(second, angle)),
            )

            if _segment_stays_in_polygon(segment[0], segment[1], polygon):
                segments.append(segment)

        sweep_value += spacing_mm

    return segments


def _dock_approach_point(
    distance_mm: float = DOCK_APPROACH_DISTANCE_MM,
) -> list[float]:
    return _round_point([0.0, -distance_mm])


def _undock_point(
    distance_mm: float = UNDOCK_DISTANCE_MM,
) -> list[float]:
    return _round_point([0.0, -distance_mm])


def _append_route_point(
    route: list[list[float]],
    point: list[float],
) -> None:
    rounded = _round_point(point)

    if not route or _distance(route[-1], rounded) > 5:
        route.append(rounded)


def _boustrophedon_candidate(
    segments: list[tuple[list[float], list[float]]],
    reverse_order: bool,
    flip_first: bool,
) -> list[list[float]]:
    ordered_segments = list(reversed(segments)) if reverse_order else segments
    route: list[list[float]] = []

    for index, segment in enumerate(ordered_segments):
        should_flip = bool(index % 2) != flip_first
        first, second = segment

        if should_flip:
            first, second = second, first

        _append_route_point(route, first)
        _append_route_point(route, second)

    return route


def _coverage_route_is_connected(
    route: list[list[float]],
    polygon: list[list[float]],
) -> bool:
    if not route:
        return False

    for start, end in zip(route, route[1:]):
        if not _segment_stays_in_polygon(start, end, polygon):
            return False

    return True


def build_coverage_route(
    polygon: list[list[float]],
    start: Optional[list[float]] = None,
    allow_start_transit_outside: bool = False,
) -> list[list[float]]:
    current = start or _polygon_centroid(polygon)
    segments = _build_coverage_segments(polygon)
    candidates: list[tuple[float, list[list[float]]]] = []

    for reverse_order in (False, True):
        for flip_first in (False, True):
            route = _boustrophedon_candidate(
                segments,
                reverse_order,
                flip_first,
            )

            if not route:
                continue

            if not allow_start_transit_outside:
                if not _segment_stays_in_polygon(current, route[0], polygon):
                    continue

            if not _coverage_route_is_connected(route, polygon):
                continue

            candidates.append((_distance(current, route[0]), route))

    if candidates:
        candidates.sort(key=lambda candidate: candidate[0])
        return candidates[0][1]

    if segments:
        raise ValueError("Cleaning route has disconnected areas")

    centroid = _polygon_centroid(polygon)
    rounded = _round_point(centroid)

    if _point_in_polygon(rounded, polygon):
        return [rounded]

    raise ValueError("Cleaning route has disconnected areas")


def build_safe_cleaning_route(
    polygon: list[list[float]],
) -> list[list[float]]:
    validate_cleaning_polygon(polygon)

    undock = _undock_point()
    dock_approach = _dock_approach_point()
    coverage_route = build_coverage_route(
        polygon,
        undock,
        allow_start_transit_outside=True,
    )

    if not coverage_route:
        raise ValueError("No cleaning route could be generated")

    for index, point in enumerate(coverage_route):
        rounded = _round_point(point)

        if not _point_in_or_on_polygon(rounded, polygon):
            raise ValueError("Cleaning route has points outside the room")

        if index == 0:
            continue

        previous = coverage_route[index - 1]

        if _distance(previous, rounded) > MAX_CLEANING_SEGMENT_MM:
            raise ValueError("Cleaning route has an unsafe long segment")

        if not _segment_stays_in_polygon(
            previous,
            rounded,
            polygon,
        ):
            raise ValueError("Cleaning route would leave the mapped room")

    route = [undock] + coverage_route + [dock_approach]

    for start, end in zip([[0.0, 0.0]] + route, route + [[0.0, 0.0]]):
        if _distance(start, end) > MAX_CLEANING_SEGMENT_MM:
            raise ValueError("Cleaning route has an unsafe long segment")

    if len(route) > MAX_CLEANING_ROUTE_POINTS:
        raise ValueError("Cleaning route is too large")

    return route


def build_cleaning_preview_route(
    polygon: list[list[float]],
) -> list[list[float]]:
    route = build_safe_cleaning_route(polygon)

    return (
        [[0.0, 0.0]]
        + route
        + [[0.0, 0.0]]
    )


def validate_cleaning_polygon(polygon: list[list[float]]) -> None:
    if len(polygon) < 4:
        raise ValueError("Room map is incomplete")

    if not _same_point(polygon[0], polygon[-1]):
        raise ValueError("Room map is not closed")

    if _self_intersects(polygon):
        raise ValueError("Room map crosses itself")

    area = _polygon_area(polygon)

    if area < MIN_ROOM_AREA_MM2:
        raise ValueError("Room map is too small")

    if area > MAX_CLEANING_ROOM_AREA_MM2:
        raise ValueError("Room map is too large")

    room_bounds = _bounds(polygon)

    if (
        room_bounds["max_x"] - room_bounds["min_x"]
        > MAX_CLEANING_ROOM_SPAN_MM
        or room_bounds["max_y"] - room_bounds["min_y"]
        > MAX_CLEANING_ROOM_SPAN_MM
    ):
        raise ValueError("Room map span is too large")


class RoomMapStore:
    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = (path or _default_room_map_path()).expanduser()
        self.lock = threading.Lock()
        self.rooms: list[dict[str, Any]] = []
        self.mapping: dict[str, Any] = self._empty_mapping()
        self._prepare_storage()
        self.load()

    def _empty_mapping(self) -> dict[str, Any]:
        return {
            "active": False,
            "closed": False,
            "started_at": None,
            "last_update": None,
            "last_left": 0.0,
            "last_right": 0.0,
            "motor_updates": 0,
            "fallback_updates": 0,
            "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "path": [[0.0, 0.0]],
            "draft_points": [],
            "warning": "",
        }

    def _prepare_storage(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        legacy_path = Path(__file__).with_name(ROOM_MAP_FILE_NAME)

        if self.path.resolve() == legacy_path.resolve():
            return

        if self.path.exists() or not legacy_path.exists():
            return

        self.path.write_text(
            legacy_path.read_text(encoding="utf-8"),
            encoding="utf-8",
        )

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

        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_name(self.path.name + ".tmp")

        with temp_path.open("w", encoding="utf-8") as file:
            json.dump(data, file, indent=2)
            file.write("\n")

        temp_path.replace(self.path)

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

    def is_mapping_active(self) -> bool:
        with self.lock:
            return bool(self.mapping["active"])

    def _record_motion_locked(
        self,
        motor_delta: Optional[tuple[int, int]],
    ) -> None:
        now = time.monotonic()
        last_update = self.mapping["last_update"] or now
        duration = min(max(now - last_update, 0.0), 0.4)

        if motor_delta is None:
            _integrate_pose(
                self.mapping["pose"],
                self.mapping["last_left"],
                self.mapping["last_right"],
                duration,
            )
            self.mapping["fallback_updates"] += 1
        else:
            distance_mm, angle_degrees = motor_delta
            _integrate_distance_angle(
                self.mapping["pose"],
                distance_mm,
                angle_degrees,
            )
            self.mapping["motor_updates"] += 1

        point = _pose_point(self.mapping["pose"])

        if _distance(self.mapping["path"][-1], point) >= 35:
            self.mapping["path"].append(_round_point(point))

        self.mapping["last_update"] = now

    def record_drive(
        self,
        left_speed: float,
        right_speed: float,
        motor_delta: Optional[tuple[int, int]] = None,
    ) -> None:
        with self.lock:
            if not self.mapping["active"]:
                return

            self._record_motion_locked(motor_delta)
            self.mapping["last_left"] = float(left_speed)
            self.mapping["last_right"] = float(right_speed)

    def finish_mapping_at_dock(
        self,
        motor_delta: Optional[tuple[int, int]] = None,
    ) -> dict[str, Any]:
        with self.lock:
            if not self.mapping["active"]:
                raise ValueError("No mapping session is active")

            self._record_motion_locked(motor_delta)

            drift = _distance(_pose_point(self.mapping["pose"]), [0.0, 0.0])
            path = list(self.mapping["path"])
            path.append(_pose_point(self.mapping["pose"]))
            draft_points = _thin_path(_loop_close_path(path))

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
                "motion_source": (
                    "motor"
                    if self.mapping["motor_updates"] > 0
                    else "joystick"
                ),
                "motor_updates": self.mapping["motor_updates"],
                "fallback_updates": self.mapping["fallback_updates"],
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
        self.last_heartbeat = time.monotonic()
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

    def heartbeat(self) -> dict[str, Any]:
        with self.lock:
            self.last_heartbeat = time.monotonic()
            return dict(self.status_data)

    def _check_heartbeat(self) -> None:
        if (
            time.monotonic() - self.last_heartbeat
            > AUTONOMOUS_HEARTBEAT_TIMEOUT_SECONDS
        ):
            raise RuntimeError("Autonomous heartbeat lost")

    def start(self, room_ids: list[str]) -> dict[str, Any]:
        if not room_ids:
            raise ValueError("Select at least one room")

        if len(room_ids) > 1:
            raise ValueError("Clean one room at a time from the dock")

        rooms = self.room_store.get_rooms(room_ids)
        room_routes = []

        for room in rooms:
            room_routes.append((room, build_safe_cleaning_route(room["points"])))

        with self.lock:
            if self.status_data["state"] in {"running", "docking"}:
                raise ValueError(
                    "Stop the current Roomba action before cleaning"
                )

            self.cancel_event.clear()
            self.last_heartbeat = time.monotonic()
            self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
            self.thread = threading.Thread(
                target=self._run,
                args=(room_routes,),
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

    def reset_to_idle(self) -> dict[str, Any]:
        self.cancel_event.set()
        self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.room_store.reset_pose_to_dock()
        self._set_status("idle", "Ready", None)
        return self.status()

    def _run(
        self,
        room_routes: list[tuple[dict[str, Any], list[list[float]]]],
    ) -> None:
        started_at = time.monotonic()

        try:
            self.roomba.vacuum_off()

            for room, route in room_routes:
                self._set_status("running", "Cleaning", room["name"])
                undock_point = route[0]
                cleaning_points = route[1:-1]
                dock_approach = route[-1]

                self._back_off_dock(undock_point, room["name"])

                self._check_heartbeat()
                self._check_runtime(started_at)
                self._drive_to(
                    cleaning_points[0],
                    room["points"],
                    room["name"],
                    enforce_room=False,
                    status_message="Going to room",
                )
                self.pose["x"] = cleaning_points[0][0]
                self.pose["y"] = cleaning_points[0][1]

                for point in cleaning_points[1:]:
                    self._check_heartbeat()
                    self._check_runtime(started_at)
                    self._drive_to(
                        point,
                        room["points"],
                        room["name"],
                        enforce_room=True,
                        status_message="Cleaning",
                    )

                    if self.cancel_event.is_set():
                        raise RuntimeError("Cleaning cancelled")

                self._check_heartbeat()
                self._check_runtime(started_at)
                self._drive_to(
                    dock_approach,
                    room["points"],
                    room["name"],
                    enforce_room=False,
                    status_message="Returning to dock",
                )
                self.pose["x"] = dock_approach[0]
                self.pose["y"] = dock_approach[1]
                self._face_point([0.0, 0.0], room["name"])
                self.pose["x"] = dock_approach[0]
                self.pose["y"] = dock_approach[1]

            self.roomba.vacuum_off()
            self._set_status("docking", "Returning to dock", None)
            self.roomba.stop()
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

    def _check_runtime(self, started_at: float) -> None:
        if time.monotonic() - started_at > MAX_CLEANING_DURATION_SECONDS:
            raise RuntimeError("Autonomous runtime limit reached")

    def _drive_to(
        self,
        target: list[float],
        polygon: list[list[float]],
        room_name: Optional[str],
        enforce_room: bool = True,
        status_message: str = "Cleaning",
    ) -> None:
        while True:
            self._check_heartbeat()

            dx = target[0] - self.pose["x"]
            dy = target[1] - self.pose["y"]
            distance = math.hypot(dx, dy)

            if distance < AUTONOMOUS_WAYPOINT_TOLERANCE_MM:
                return

            desired_heading = math.atan2(dx, dy)
            turn_angle = _normalize_angle(
                desired_heading - self.pose["theta"]
            )

            if abs(turn_angle) > AUTONOMOUS_HEADING_TOLERANCE_RAD:
                turn_chunk = math.copysign(
                    min(abs(turn_angle), AUTONOMOUS_TURN_CHUNK_RAD),
                    turn_angle,
                )
                self._turn(turn_chunk, room_name, status_message)
                continue

            step_distance = min(distance, AUTONOMOUS_STRAIGHT_CHUNK_MM)
            projected = [
                self.pose["x"] + step_distance * math.sin(self.pose["theta"]),
                self.pose["y"] + step_distance * math.cos(self.pose["theta"]),
            ]

            current_point = [self.pose["x"], self.pose["y"]]

            if enforce_room and not _segment_stays_in_polygon(
                current_point,
                projected,
                polygon,
            ):
                raise RuntimeError("Stopping before leaving mapped room")

            self._drive_straight(step_distance, room_name, status_message)

            if self.cancel_event.is_set():
                raise RuntimeError("Cleaning cancelled")

    def _back_off_dock(
        self,
        undock_point: list[float],
        room_name: Optional[str],
    ) -> None:
        if abs(undock_point[0]) > 1 or undock_point[1] >= 0:
            raise RuntimeError("Cleaning route does not start behind the dock")

        self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._set_status("running", "Backing off dock", room_name)

        reverse_distance = abs(undock_point[1])
        speed = AUTONOMOUS_STRAIGHT_SPEED_MM_S

        self._move_for(
            -speed,
            -speed,
            reverse_distance / speed,
            room_name,
            target_distance=reverse_distance,
            status_message="Backing off dock",
        )

        self.pose["x"] = undock_point[0]
        self.pose["y"] = undock_point[1]
        self.pose["theta"] = 0.0
        self._set_status("running", "Cleaning", room_name)

    def _face_point(
        self,
        target: list[float],
        room_name: Optional[str],
    ) -> None:
        self._set_status("running", "Facing dock", room_name)

        while True:
            self._check_heartbeat()

            dx = target[0] - self.pose["x"]
            dy = target[1] - self.pose["y"]

            if math.hypot(dx, dy) <= 1:
                return

            desired_heading = math.atan2(dx, dy)
            turn_angle = _normalize_angle(
                desired_heading - self.pose["theta"]
            )

            if abs(turn_angle) <= AUTONOMOUS_HEADING_TOLERANCE_RAD:
                self.pose["theta"] = desired_heading
                self._set_status("running", "Dock handoff", room_name)
                return

            turn_chunk = math.copysign(
                min(abs(turn_angle), AUTONOMOUS_TURN_CHUNK_RAD),
                turn_angle,
            )
            self._turn(turn_chunk, room_name, "Facing dock")

            if self.cancel_event.is_set():
                raise RuntimeError("Cleaning cancelled")

    def _turn(
        self,
        angle: float,
        room_name: Optional[str],
        status_message: str = "Cleaning",
    ) -> None:
        speed = AUTONOMOUS_TURN_SPEED_MM_S
        left = speed if angle > 0 else -speed
        right = -speed if angle > 0 else speed
        angular_speed = abs((left - right) / self.wheel_base_mm)
        duration = abs(angle) / angular_speed

        self._move_for(
            left,
            right,
            duration,
            room_name,
            target_angle=abs(angle),
            status_message=status_message,
        )

    def _drive_straight(
        self,
        distance: float,
        room_name: Optional[str],
        status_message: str = "Cleaning",
    ) -> None:
        speed = AUTONOMOUS_STRAIGHT_SPEED_MM_S
        duration = distance / speed
        self._move_for(
            speed,
            speed,
            duration,
            room_name,
            target_distance=distance,
            status_message=status_message,
        )

    def _move_for(
        self,
        left: int,
        right: int,
        duration: float,
        room_name: Optional[str],
        target_distance: Optional[float] = None,
        target_angle: Optional[float] = None,
        status_message: str = "Cleaning",
    ) -> None:
        start = time.monotonic()
        last_update = start
        measured_distance = 0.0
        measured_angle = 0.0
        sensor_ok = self._clear_motion_sensors()
        max_duration = duration * SENSORLESS_TIME_LIMIT_MULTIPLIER + 0.35

        while True:
            if self.cancel_event.is_set():
                raise RuntimeError("Cleaning cancelled")

            self._check_heartbeat()

            now = time.monotonic()
            elapsed = now - start

            if elapsed >= max_duration:
                break

            if target_distance is not None and measured_distance >= target_distance:
                break

            if target_angle is not None and measured_angle >= target_angle:
                break

            step = min(now - last_update, max_duration - elapsed)

            sensor_delta = self._read_motion_delta()

            if sensor_delta:
                sensor_ok = True
                distance_delta, angle_delta = sensor_delta
                _integrate_distance_angle(
                    self.pose,
                    distance_delta,
                    angle_delta,
                )
                measured_distance += abs(distance_delta)
                measured_angle += abs(math.radians(angle_delta))
            elif step > 0:
                _integrate_pose(
                    self.pose,
                    left,
                    right,
                    step,
                    self.wheel_base_mm,
                )

                if not sensor_ok:
                    if target_distance is not None:
                        measured_distance += abs((left + right) / 2) * step

                    if target_angle is not None:
                        measured_angle += (
                            abs((left - right) / self.wheel_base_mm) * step
                        )

            if target_distance is not None and measured_distance >= target_distance:
                break

            if target_angle is not None and measured_angle >= target_angle:
                break

            self._set_status("running", status_message, room_name)

            if self.cancel_event.is_set():
                raise RuntimeError("Cleaning cancelled")

            self.roomba.drive_wheels(right_speed=right, left_speed=left)
            last_update = now
            time.sleep(AUTONOMOUS_COMMAND_INTERVAL_SECONDS)

        self.roomba.stop()
        self._set_status("running", status_message, room_name)

    def _clear_motion_sensors(self) -> bool:
        try:
            self.roomba.read_distance_angle()
            return True
        except Exception:
            return False

    def _read_motion_delta(self) -> Optional[tuple[int, int]]:
        try:
            return self.roomba.read_distance_angle()
        except Exception:
            return None

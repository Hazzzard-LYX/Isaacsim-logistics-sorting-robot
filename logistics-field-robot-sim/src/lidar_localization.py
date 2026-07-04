#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
已知地图 + 2D 激光雷达定位原型。

当前脚本不依赖 Isaac Sim：先用已知场地几何做 ray casting，再用 scan matching
从一帧激光雷达距离数据估计机器人位姿 (x, y, yaw)。后续接入 Isaac LiDAR 时，
只需要把真实 ranges 和对应 angles 传给 KnownMapLocalizer.localize()。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


FIELD_LENGTH = 4.0
FIELD_WIDTH = 2.0

BOX_LENGTH = 0.28
BOX_WIDTH = 0.20
ROBOT_RADIUS = 0.22

OBSTACLE_RADIUS = 0.051
OBSTACLE_POSITIONS = [(-0.8, 0.0), (0.8, 0.0)]

PICKUP_SLOTS = [
    (-1.75, 0.6, "vertical"),
    (-1.75, 0.0, "vertical"),
    (-1.75, -0.6, "vertical"),
]

PLACE_SLOTS = [
    (1.76, 0.8, "horizontal"),
    (1.8, 0.4, "vertical"),
    (1.8, 0.0, "vertical"),
    (1.8, -0.4, "vertical"),
    (1.76, -0.8, "horizontal"),
]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


class KnownFieldMap:
    """场地二维地图：边界墙、矩形箱体和圆柱障碍物。"""

    def __init__(self) -> None:
        self.segments: list[Segment] = []
        self.circles: list[Circle] = []
        self._build_boundary()
        self._build_boxes()
        self._build_obstacles()

    def _build_boundary(self) -> None:
        half_l = FIELD_LENGTH / 2.0
        half_w = FIELD_WIDTH / 2.0
        self._add_rect(0.0, 0.0, FIELD_LENGTH, FIELD_WIDTH)
        # _add_rect 会生成矩形四条边；这里表示场地内边界。
        assert len(self.segments) == 4

    def _build_boxes(self) -> None:
        for cx, cy, orientation in [*PICKUP_SLOTS, *PLACE_SLOTS]:
            sx, sy = box_footprint(orientation)
            self._add_rect(cx, cy, sx, sy)

    def _build_obstacles(self) -> None:
        for ox, oy in OBSTACLE_POSITIONS:
            self.circles.append(Circle(ox, oy, OBSTACLE_RADIUS))

    def _add_rect(self, cx: float, cy: float, sx: float, sy: float) -> None:
        min_x = cx - sx / 2.0
        max_x = cx + sx / 2.0
        min_y = cy - sy / 2.0
        max_y = cy + sy / 2.0
        self.segments.extend(
            [
                Segment(min_x, min_y, max_x, min_y),
                Segment(max_x, min_y, max_x, max_y),
                Segment(max_x, max_y, min_x, max_y),
                Segment(min_x, max_y, min_x, min_y),
            ]
        )

    def ray_cast(self, x: float, y: float, angle: float, max_range: float) -> float:
        """返回从 (x, y) 沿 angle 方向打出的第一处命中距离。"""
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = max_range

        for segment in self.segments:
            hit = ray_segment_intersection(x, y, dx, dy, segment)
            if hit is not None and 0.0 <= hit < best:
                best = hit

        for circle in self.circles:
            hit = ray_circle_intersection(x, y, dx, dy, circle)
            if hit is not None and 0.0 <= hit < best:
                best = hit

        return best


def box_footprint(orientation: str) -> tuple[float, float]:
    """返回箱体在 xy 平面的尺寸，语义与 create_logistics_field.py 一致。"""
    if orientation == "horizontal":
        return BOX_LENGTH, BOX_WIDTH
    return BOX_WIDTH, BOX_LENGTH


def ray_segment_intersection(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    segment: Segment,
) -> float | None:
    sx = segment.x2 - segment.x1
    sy = segment.y2 - segment.y1
    denom = cross(dx, dy, sx, sy)
    if abs(denom) < 1e-9:
        return None

    qpx = segment.x1 - ox
    qpy = segment.y1 - oy
    t = cross(qpx, qpy, sx, sy) / denom
    u = cross(qpx, qpy, dx, dy) / denom

    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def ray_circle_intersection(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    circle: Circle,
) -> float | None:
    fx = ox - circle.x
    fy = oy - circle.y
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - circle.radius * circle.radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0.0:
        return None

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0
    hits = [t for t in (t1, t2) if t >= 0.0]
    return min(hits) if hits else None


def cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def simulate_scan(
    known_map: KnownFieldMap,
    pose: Pose2D,
    angles: Iterable[float],
    max_range: float,
) -> list[float]:
    return [
        known_map.ray_cast(pose.x, pose.y, pose.yaw + local_angle, max_range)
        for local_angle in angles
    ]


class KnownMapLocalizer:
    """用已知地图对单帧 2D LiDAR scan 做位姿匹配。"""

    def __init__(
        self,
        known_map: KnownFieldMap,
        angles: list[float],
        max_range: float = 6.0,
    ) -> None:
        self.known_map = known_map
        self.angles = angles
        self.max_range = max_range

    def localize(self, ranges: list[float], initial_guess: Pose2D) -> Pose2D:
        pose = initial_guess
        search_levels = [
            (0.30, math.radians(20.0), 0.05, math.radians(5.0)),
            (0.08, math.radians(6.0), 0.02, math.radians(1.5)),
            (0.025, math.radians(2.0), 0.005, math.radians(0.5)),
        ]

        for xy_radius, yaw_radius, xy_step, yaw_step in search_levels:
            pose = self._search_neighborhood(
                ranges,
                pose,
                xy_radius,
                yaw_radius,
                xy_step,
                yaw_step,
            )
        return pose

    def _search_neighborhood(
        self,
        ranges: list[float],
        center: Pose2D,
        xy_radius: float,
        yaw_radius: float,
        xy_step: float,
        yaw_step: float,
    ) -> Pose2D:
        best_pose = center
        best_score = float("inf")

        for x in frange(center.x - xy_radius, center.x + xy_radius, xy_step):
            for y in frange(center.y - xy_radius, center.y + xy_radius, xy_step):
                if not is_pose_inside_field(x, y):
                    continue
                for yaw in frange(center.yaw - yaw_radius, center.yaw + yaw_radius, yaw_step):
                    candidate = Pose2D(x, y, normalize_angle(yaw))
                    score = scan_error(self.known_map, self.angles, ranges, candidate, self.max_range)
                    if score < best_score:
                        best_score = score
                        best_pose = candidate

        return best_pose


def scan_error(
    known_map: KnownFieldMap,
    angles: list[float],
    observed_ranges: list[float],
    pose: Pose2D,
    max_range: float,
) -> float:
    predicted = simulate_scan(known_map, pose, angles, max_range)
    total = 0.0
    count = 0

    for observed, expected in zip(observed_ranges, predicted):
        if not math.isfinite(observed):
            continue
        obs = min(observed, max_range)
        diff = obs - expected
        total += min(diff * diff, 0.25)
        count += 1

    return total / max(count, 1)


def is_pose_inside_field(x: float, y: float) -> bool:
    half_l = FIELD_LENGTH / 2.0 - ROBOT_RADIUS
    half_w = FIELD_WIDTH / 2.0 - ROBOT_RADIUS
    return -half_l <= x <= half_l and -half_w <= y <= half_w


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    value = start
    epsilon = step * 0.5
    while value <= stop + epsilon:
        yield value
        value += step


def make_lidar_angles(num_beams: int = 180) -> list[float]:
    return [
        -math.pi + i * (2.0 * math.pi / num_beams)
        for i in range(num_beams)
    ]


def main() -> None:
    known_map = KnownFieldMap()
    angles = make_lidar_angles(180)
    localizer = KnownMapLocalizer(known_map, angles)

    true_pose = Pose2D(0.35, -0.22, math.radians(18.0))
    initial_guess = Pose2D(0.10, -0.05, math.radians(5.0))
    observed_ranges = simulate_scan(known_map, true_pose, angles, max_range=6.0)
    estimated_pose = localizer.localize(observed_ranges, initial_guess)

    print("true_pose:")
    print(f"  x={true_pose.x:.3f}, y={true_pose.y:.3f}, yaw={math.degrees(true_pose.yaw):.2f} deg")
    print("initial_guess:")
    print(f"  x={initial_guess.x:.3f}, y={initial_guess.y:.3f}, yaw={math.degrees(initial_guess.yaw):.2f} deg")
    print("estimated_pose:")
    print(f"  x={estimated_pose.x:.3f}, y={estimated_pose.y:.3f}, yaw={math.degrees(estimated_pose.yaw):.2f} deg")


if __name__ == "__main__":
    main()

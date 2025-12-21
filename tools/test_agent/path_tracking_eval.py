#!/usr/bin/env python3
import argparse
import math
import pathlib
import statistics
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener


Point = Tuple[float, float]


def _closest_point_on_segment(p: Point, a: Point, b: Point) -> Tuple[Point, float]:
    ax, ay = a
    bx, by = b
    px, py = p
    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return a, 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / ab2
    t = max(0.0, min(1.0, t))
    return (ax + t * abx, ay + t * aby), t


def _distance(p1: Point, p2: Point) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


@dataclass
class Settings:
    path_topic: str
    cmd_vel_topic: str
    map_frame: str
    base_frame: str
    report_path: pathlib.Path
    goal_tolerance_m: float
    stopped_speed_mps: float
    stable_time_s: float
    sample_hz: float


class PathTrackingEval(Node):
    def __init__(self, settings: Settings):
        super().__init__("path_tracking_eval")
        self.settings = settings

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._path_sub = self.create_subscription(Path, settings.path_topic, self._on_path, qos)

        self._cmd_vel: Twist = Twist()
        self._cmd_sub = self.create_subscription(Twist, settings.cmd_vel_topic, self._on_cmd, 10)

        self._path_points: List[Point] = []
        self._start_time = time.time()
        self._goal_reached_since: Optional[float] = None
        self._errors: List[float] = []
        self._speeds: List[float] = []
        self._dist_to_goal: List[float] = []
        self._last_pose_ok = False
        self._report_written = False

        period = 1.0 / max(1.0, settings.sample_hz)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Eval path={settings.path_topic}, cmd_vel={settings.cmd_vel_topic}, "
            f"frames={settings.map_frame}->{settings.base_frame}, report={settings.report_path}"
        )

    def _on_path(self, msg: Path) -> None:
        pts: List[Point] = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        self._path_points = pts

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_vel = msg

    def _get_robot_xy(self) -> Optional[Point]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self.settings.map_frame, self.settings.base_frame, rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def _compute_cross_track_error(self, pos: Point) -> Optional[float]:
        if len(self._path_points) < 2:
            return None
        best = float("inf")
        for i in range(len(self._path_points) - 1):
            a = self._path_points[i]
            b = self._path_points[i + 1]
            c, _ = _closest_point_on_segment(pos, a, b)
            best = min(best, _distance(pos, c))
        return best

    def _compute_dist_to_goal(self, pos: Point) -> Optional[float]:
        if not self._path_points:
            return None
        return _distance(pos, self._path_points[-1])

    def _tick(self) -> None:
        if len(self._path_points) < 2:
            return

        pos = self._get_robot_xy()
        if pos is None:
            self._last_pose_ok = False
            return
        self._last_pose_ok = True

        err = self._compute_cross_track_error(pos)
        dgoal = self._compute_dist_to_goal(pos)
        speed = abs(self._cmd_vel.linear.x)

        if err is not None:
            self._errors.append(err)
        if dgoal is not None:
            self._dist_to_goal.append(dgoal)
        self._speeds.append(speed)

        if dgoal is not None and dgoal <= self.settings.goal_tolerance_m and speed <= self.settings.stopped_speed_mps:
            if self._goal_reached_since is None:
                self._goal_reached_since = time.time()
            elif time.time() - self._goal_reached_since >= self.settings.stable_time_s:
                self.get_logger().info("Goal condition stable; writing report and shutting down")
                self._write_report(completed=True)
                rclpy.shutdown()
        else:
            self._goal_reached_since = None

    def _write_report(self, completed: bool) -> None:
        self._report_written = True
        self.settings.report_path.parent.mkdir(parents=True, exist_ok=True)
        total_s = time.time() - self._start_time

        errs = self._errors if self._errors else [float("nan")]
        speeds = self._speeds if self._speeds else [float("nan")]
        dgoal_last = self._dist_to_goal[-1] if self._dist_to_goal else float("nan")

        errs_sorted = sorted(self._errors) if self._errors else []
        p95 = errs_sorted[int(0.95 * (len(errs_sorted) - 1))] if len(errs_sorted) >= 2 else (errs_sorted[0] if errs_sorted else float("nan"))

        rms = math.sqrt(statistics.mean([e * e for e in errs])) if self._errors else float("nan")
        mx = max(errs) if self._errors else float("nan")
        avg_speed = statistics.mean(speeds) if self._speeds else float("nan")

        verdict = "PASS" if completed and not math.isnan(rms) else "INCOMPLETE"
        hints: List[str] = []
        if not completed:
            hints.append("Did not meet goal condition (distance + stop) within the run time.")
        if not math.isnan(mx) and mx > 0.30:
            hints.append("Max cross-track error > 0.30 m: try increasing lookahead or reducing desired_linear_vel.")
        if not math.isnan(rms) and rms > 0.15:
            hints.append("RMS error > 0.15 m: check TF frames, path density (interpolation step), and controller gains/limits.")

        md = []
        md.append("# Path Tracking Report")
        md.append("")
        md.append(f"- verdict: **{verdict}**")
        md.append(f"- completed: `{completed}`")
        md.append(f"- run_time_s: `{total_s:.2f}`")
        md.append(f"- last_distance_to_goal_m: `{dgoal_last:.3f}`")
        md.append("")
        md.append("## Metrics")
        md.append(f"- cross_track_error_rms_m: `{rms:.3f}`")
        md.append(f"- cross_track_error_p95_m: `{p95:.3f}`")
        md.append(f"- cross_track_error_max_m: `{mx:.3f}`")
        md.append(f"- average_speed_mps: `{avg_speed:.3f}`")
        md.append("")
        md.append("## Notes")
        if hints:
            for h in hints:
                md.append(f"- {h}")
        else:
            md.append("- No hints.")
        md.append("")
        md.append("## Inputs")
        md.append(f"- path_topic: `{self.settings.path_topic}`")
        md.append(f"- cmd_vel_topic: `{self.settings.cmd_vel_topic}`")
        md.append(f"- map_frame: `{self.settings.map_frame}`")
        md.append(f"- base_frame: `{self.settings.base_frame}`")

        self.settings.report_path.write_text("\n".join(md) + "\n")

    def destroy_node(self) -> bool:
        if not self._report_written:
            try:
                self._write_report(completed=False)
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--path-topic", default="waypoint_path")
    ap.add_argument("--cmd-vel-topic", default="/cmd_vel")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--report", default="tools/test_agent/sessions/latest/report.md")
    ap.add_argument("--goal-tolerance-m", type=float, default=0.25)
    ap.add_argument("--stopped-speed-mps", type=float, default=0.03)
    ap.add_argument("--stable-time-s", type=float, default=2.0)
    ap.add_argument("--sample-hz", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = PathTrackingEval(
        Settings(
            path_topic=args.path_topic,
            cmd_vel_topic=args.cmd_vel_topic,
            map_frame=args.map_frame,
            base_frame=args.base_frame,
            report_path=pathlib.Path(args.report),
            goal_tolerance_m=args.goal_tolerance_m,
            stopped_speed_mps=args.stopped_speed_mps,
            stable_time_s=args.stable_time_s,
            sample_hz=args.sample_hz,
        )
    )
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

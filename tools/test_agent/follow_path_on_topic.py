#!/usr/bin/env python3
import argparse
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path


@dataclass
class Settings:
    path_topic: str
    action_name: str
    controller_id: str
    goal_checker_id: str
    resend_on_update: bool
    min_poses: int
    startup_grace_s: float


class FollowPathOnTopic(Node):
    def __init__(self, settings: Settings):
        super().__init__("follow_path_on_topic")
        self.settings = settings

        self._last_path_stamp_ns: Optional[int] = None
        self._goal_sent = False
        self._start_time = time.time()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._sub = self.create_subscription(Path, settings.path_topic, self._on_path, qos)
        self._client = ActionClient(self, FollowPath, settings.action_name)

        self.get_logger().info(
            f"Listening path={settings.path_topic}, action={settings.action_name}, "
            f"controller_id={settings.controller_id}, goal_checker_id={settings.goal_checker_id}"
        )

    def _on_path(self, msg: Path) -> None:
        if len(msg.poses) < self.settings.min_poses:
            return

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._last_path_stamp_ns is None:
            self._last_path_stamp_ns = stamp_ns
        elif stamp_ns == self._last_path_stamp_ns:
            return

        # Grace period to let Nav2 come up.
        if time.time() - self._start_time < self.settings.startup_grace_s:
            self._last_path_stamp_ns = stamp_ns
            return

        if self._goal_sent and not self.settings.resend_on_update:
            self._last_path_stamp_ns = stamp_ns
            return

        if not self._client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("FollowPath action server not available yet")
            return

        goal = FollowPath.Goal()
        goal.path = msg
        goal.controller_id = self.settings.controller_id
        goal.goal_checker_id = self.settings.goal_checker_id

        self.get_logger().info(f"Sending FollowPath goal with {len(msg.poses)} poses")
        self._client.send_goal_async(goal)
        self._goal_sent = True
        self._last_path_stamp_ns = stamp_ns


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--path-topic", default="waypoint_path")
    ap.add_argument("--action-name", default="follow_path")
    ap.add_argument("--controller-id", default="FollowPath")
    ap.add_argument("--goal-checker-id", default="general_goal_checker")
    ap.add_argument("--resend-on-update", action="store_true")
    ap.add_argument("--min-poses", type=int, default=2)
    ap.add_argument("--startup-grace-s", type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = FollowPathOnTopic(
        Settings(
            path_topic=args.path_topic,
            action_name=args.action_name,
            controller_id=args.controller_id,
            goal_checker_id=args.goal_checker_id,
            resend_on_update=args.resend_on_update,
            min_poses=args.min_poses,
            startup_grace_s=args.startup_grace_s,
        )
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


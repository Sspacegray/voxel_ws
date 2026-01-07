#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from industrial_route_interfaces.srv import PlanRoute
from industrial_route_interfaces.action import ExecuteRoute


def yaw_to_quat(yaw: float):
    # geometry_msgs/Quaternion without importing tf
    import math
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PlannerExecutor(Node):
    def __init__(self):
        super().__init__("industrial_route_plan_and_execute")
        self.cli = self.create_client(PlanRoute, "/industrial_route/plan")
        self.ac = ActionClient(self, ExecuteRoute, "/industrial_route/execute")

    def wait_ready(self):
        self.get_logger().info("Waiting for /industrial_route/plan ...")
        self.cli.wait_for_service()
        self.get_logger().info("Waiting for /industrial_route/execute ...")
        self.ac.wait_for_server()

    def make_pose(self, x: float, y: float, yaw: float, frame_id="map"):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = yaw_to_quat(float(yaw))
        return p

    def plan(self, start: PoseStamped, goal: PoseStamped):
        req = PlanRoute.Request()
        req.start = start
        req.goal = goal
        req.use_start_pose = True
        req.use_goal_pose = True

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def execute_path(self, path):
        goal = ExecuteRoute.Goal()
        goal.use_path = True
        goal.path = path
        goal.rotate_in_place = True

        send_future = self.ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("ExecuteRoute goal rejected")
            return False

        self.get_logger().info("ExecuteRoute accepted")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        self.get_logger().info(f"ExecuteRoute done: success={res.success} code={res.error_code} msg={res.error_msg}")
        return bool(res.success)

    def on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f"progress={fb.progress:.2f} node={fb.current_node_id} edge={fb.current_edge_id}"
        )


def main():
    rclpy.init()
    node = PlannerExecutor()
    node.wait_ready()

    start = node.make_pose(0.2, 0.2, 0.0)
    goal = node.make_pose(1.8, 1.8, math.pi / 2.0)

    node.get_logger().info("Planning...")
    resp = node.plan(start, goal)
    if not resp.success:
        node.get_logger().error(f"Plan failed: {resp.error_code} {resp.error_msg}")
        rclpy.shutdown()
        return 1

    node.get_logger().info(f"Planned: nodes={len(resp.node_sequence)} path_poses={len(resp.dense_path.poses)} cost={resp.total_cost:.3f}")
    ok = node.execute_path(resp.dense_path)
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())


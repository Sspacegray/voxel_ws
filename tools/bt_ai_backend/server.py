#!/usr/bin/env python3
"""
bt_ai_backend - 本地任务生成服务（Stub / 零依赖）

用途：
- 为 bt_visual_editor 的 “AI任务” 提供一个可跑通的本地 HTTP 后端
- 你可以把这里替换成任意大模型/规则引擎，只要保持返回 JSON 结构即可
"""

from __future__ import annotations

import argparse
import json
import re
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any


def _json_dumps(obj: Any) -> bytes:
    return json.dumps(obj, ensure_ascii=False, indent=2).encode("utf-8")


def _extract_waypoints(prompt: str):
    """
    极简规则：
    - 尝试解析类似 A(1.2,0.3,90deg) / (1.2,0.3,90) 的点
    - 解析不到则给默认点
    """
    # (x, y, yaw) 允许 deg 后缀
    pattern = re.compile(
        r"(?:[A-Za-z0-9_]+)?\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*(?:,\s*([-+]?\d+(?:\.\d+)?)(?:\s*deg)?)?\s*\)"
    )
    points = []
    for m in pattern.finditer(prompt):
        x = float(m.group(1))
        y = float(m.group(2))
        yaw = float(m.group(3)) if m.group(3) is not None else 0.0
        points.append((x, y, yaw))

    if points:
        return points

    return [
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 90.0),
        (0.5, 2.0, -90.0),
    ]


def _extract_loops(prompt: str) -> int:
    m = re.search(r"(?:循环|loop|loops)\s*([0-9]+)", prompt, flags=re.IGNORECASE)
    if not m:
        return 1
    loops = int(m.group(1))
    return max(1, min(loops, 999))


def _extract_dwell(prompt: str) -> float:
    m = re.search(r"(?:停留|wait|dwell)\s*([0-9]+(?:\.[0-9]+)?)\s*(?:s|秒)?", prompt, flags=re.IGNORECASE)
    if not m:
        return 0.0
    dwell = float(m.group(1))
    return max(0.0, min(dwell, 3600.0))


def _make_waypoints_yaml(frame_id: str, points, dwell_sec: float) -> str:
    lines = []
    lines.append(f"frame_id: {frame_id}")
    lines.append("waypoints:")
    for i, (x, y, yaw_deg) in enumerate(points, start=1):
        lines.append(f"  - id: P{i}")
        lines.append(f"    x: {x:.3f}")
        lines.append(f"    y: {y:.3f}")
        lines.append(f"    yaw_deg: {yaw_deg:.3f}")
        lines.append(f"    dwell_sec: {dwell_sec:.3f}")
        lines.append("    actions: []")
    lines.append("")
    return "\n".join(lines)


def _make_bt_xml(template_id: str) -> str:
    # 仅示例：根据 template_id 返回一份可运行的 BT
    if template_id == "route_graph_strict_follow":
        return """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
        <RecoveryNode number_of_retries="1" name="ComputeRoute">
          <RateController hz="0.5">
            <Fallback>
              <ReactiveSequence>
                <Inverter><GoalUpdated/></Inverter>
                <IsPathValid path="{path}"/>
              </ReactiveSequence>
              <ComputeAndTrackRoute goal="{goal}" path="{path}" route="{route}" use_poses="true" error_code_id="{compute_route_error_code}"/>
            </Fallback>
          </RateController>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </RecoveryNode>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <RoundRobin name="RecoveryActions">
        <Sequence name="ClearingActions">
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </Sequence>
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.30" backup_speed="0.15"/>
      </RoundRobin>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""
    if template_id == "nav2_patrol_through_poses":
        return """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode name="PatrolRecovery" number_of_retries="6">
      <PipelineSequence name="PatrolPipeline">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
        <NavigateThroughPoses goals="{goals}" server_name="navigate_through_poses"/>
      </PipelineSequence>
      <RoundRobin name="RecoveryActions">
        <Sequence name="ClearingActions">
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </Sequence>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.30" backup_speed="0.15"/>
      </RoundRobin>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""
    # 默认：单点导航
    return """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigatePipeline">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
        <NavigateToPose goal="{goal}" server_name="navigate_to_pose"/>
      </PipelineSequence>
      <RoundRobin name="RecoveryActions">
        <Sequence name="ClearingActions">
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </Sequence>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.30" backup_speed="0.15"/>
      </RoundRobin>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, payload: Any):
        body = _json_dumps(payload)
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        if self.path != "/generate":
            self._send(404, {"error": "not found"})
            return

        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            req = json.loads(raw.decode("utf-8"))
        except Exception:
            self._send(400, {"error": "invalid json"})
            return

        prompt = str(req.get("prompt", "") or "")
        template_id = str(req.get("template_id", "") or "nav2_patrol_through_poses")

        frame_id = "map"
        loops = _extract_loops(prompt)
        dwell = _extract_dwell(prompt)
        points = _extract_waypoints(prompt)

        mission = {
            "name": "ai_generated_mission",
            "frame_id": frame_id,
            "template_id": template_id,
            "loops": loops,
            "error_policy": {
                "max_retries": 2,
                "on_failure": "skip",
                "report_topic": "/inspection/state",
            },
            "waypoints": [
                {
                    "id": f"P{i}",
                    "x": x,
                    "y": y,
                    "yaw_deg": yaw,
                    "dwell_sec": dwell,
                    "actions": [],
                }
                for i, (x, y, yaw) in enumerate(points, start=1)
            ],
        }

        payload = {
            "mission": mission,
            "bt_xml": _make_bt_xml(template_id),
            "waypoints_yaml": _make_waypoints_yaml(frame_id, points, dwell),
        }

        self._send(200, payload)

    def log_message(self, fmt: str, *args):
        # keep quiet
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    args = parser.parse_args()

    httpd = HTTPServer((args.host, args.port), Handler)
    print(f"[bt_ai_backend] listening on http://{args.host}:{args.port}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()


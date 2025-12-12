## Route Server 相关代码 Review（需修正）

1) `route_navigator.py`（ComputeAndTrackRoute 示例）字段错误：动作定义需要 `start_id/goal_id`（uint16）、`use_start/use_poses`，当前代码使用了不存在的 `RouteNodes` 类型并未填这些布尔字段，会直接编译/运行失败。应改为：
```python
goal = ComputeAndTrackRoute.Goal()
goal.start_id = 0
goal.goal_id = 5
goal.use_start = False
goal.use_poses = False
```
2) 节点 ID 类型：示例中传字符串 ID，Compute*Route 动作字段为 `uint16`，需要整数；否则在序列化时报类型错误。
3) FollowWaypoints 示例的朝向：当前始终 `orientation.w = 1.0`，若需要“到点先转正再前进”，应根据相邻节点方向设置 yaw（用 `atan2`），否则控制器无法提前对齐朝向。
4) Route Tool 说明：Humble 无预置面板，必须自行编译 `nav2_rviz_plugins`；在文档中应提示缺失插件时的处理（已在 route.md 修正，可保持）。
5) 运行前检查：在发送动作前增加 `self.route_client.wait_for_server()` 已有，但建议在 main 中先查 `/route_server` lifecycle 状态为 active，否则目标会被拒绝。

## nav2_params.yaml 重点问题（需调整）

1) `controller_server.general_goal_checker.yaw_goal_tolerance` 设为 3.14（几乎不管朝向），与“到点先转正再前进”需求相反，建议收紧到 0.1~0.2 rad，并在控制器侧开启旋转对齐逻辑。路径：`src/robot_navigation/src/params/nav2_params.yaml`.
2) MPPI 自定义参数 `minAngleDiff: 365.0` 与注释（角度差过大继续调头）不符，MPPI 期望弧度量纲，该值过大导致永不触发，建议 0.2~0.3 rad；若想用度数需先转换。路径同上。
3) `odom_topic: /Odometry` 与大多数驱动的 `odom`/`/odom` 不同，需确认实际话题，否则控制器无里程计导致停滞；如不匹配请统一到实际话题名。
4) `transform_tolerance: 2.0`（local/global costmap）过大，会掩盖 TF 延迟并拖慢碰撞响应，实机建议 0.3~0.5 s，并优先修复 TF 时序而非放大容差。
5) MPPI `batch_size: 1000`、`controller_frequency: 20 Hz`、`time_steps: 50` 对 Jetson/低功耗平台算力压力大（等效 1000×50 轨迹/周期），如出现周期超时或高 CPU，建议降低 batch_size=400~600 或调 controller_frequency=15 Hz，保持实时性。

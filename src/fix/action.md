# Route Server Arch(1) 严格路网追踪 Action

目标：让 `NavigateToPose` 在 Jazzy 下走 Route Server 输出的 dense path，Controller 严格追踪路网；自由空间全局规划不再介入（除非你显式启用 first/last‑mile）。

当前工程基础：
- route_server 已在参数中配置并随导航启动：`src/robot_navigation/src/params/nav2_params.yaml:595`（graph=`1126.geojson`）。
- 控制器是 MPPI，已有 PathAlign/PathFollow critic：`src/robot_navigation/src/params/nav2_params.yaml:169` 起。
- 现有 BT `navigate_on_route_graph_w_recovery.xml` 属于旧版节点体系，不适配 Jazzy。

---

## Step 0：前置检查

1) 启动导航（会带起 route_server）：
```bash
ros2 launch robot_navigation navigation.launch.py
```

2) 确认 route_server active + Action 存在：
```bash
ros2 lifecycle get /route_server            # 期望 active
ros2 action list | grep route              # 期望看到 /compute_route /compute_and_track_route
```

3) RViz 里加 MarkerArray 订阅 `/route_graph`，确认图与 `map` 对齐。

---

## Step 1：新建 Jazzy 兼容 BT（Arch 1）

新建文件：
`src/robot_navigation/src/behavior_trees/navigate_on_route_graph_arch1_jazzy.xml`

核心逻辑：ComputeAndTrackRoute（或 ComputeRoute）生成 `{path}` → FollowPath 追踪。

```xml
<root BTCPP_format="4" main_tree_to_execute="NavigateOnRouteGraphArch1">
  <BehaviorTree ID="NavigateOnRouteGraphArch1">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}"
                            default_controller="FollowPath"
                            topic_name="controller_selector"/>

        <RecoveryNode number_of_retries="1" name="ComputeRoute">
          <RateController hz="0.5">
            <Fallback>
              <ReactiveSequence>
                <Inverter><GoalUpdated/></Inverter>
                <IsPathValid path="{path}"/>
              </ReactiveSequence>

              <!-- 推荐：带 route operations / feedback -->
              <ComputeAndTrackRoute goal="{goal}"
                                    path="{path}"
                                    route="{route}"
                                    use_poses="true"
                                    error_code_id="{compute_route_error_code}"/>

              <!-- 若不需要 operations，可替换成 ComputeRoute -->
              <!--
              <ComputeRoute goal="{goal}" path="{path}" route="{route}"
                            use_poses="true"
                            error_code_id="{compute_route_error_code}"/>
              -->
            </Fallback>
          </RateController>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </RecoveryNode>

        <RecoveryNode number_of_retries="1" name="FollowPath">
          <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}"
                      smoother_id="SmoothPath"/>
          <FollowPath path="{path}" controller_id="{selected_controller}"
                      error_code_id="{follow_path_error_code}"/>
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>

      <RoundRobin name="RecoveryActions">
        <Sequence>
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </Sequence>
        <Wait wait_duration="5.0"/>
        <BackUp backup_dist="0.30" backup_speed="0.15"/>
      </RoundRobin>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

说明：
- 用 `PipelineSequence` 是因为 `ComputeAndTrackRoute` 会持续 RUNNING；Pipeline 允许它和 `FollowPath` 并行 tick。
- 不做 first/last‑mile，保证 route path 不被自由空间规划覆盖（KISS / Arch(1)）。

---

## Step 2：修改 bt_navigator 参数

文件：`src/robot_navigation/src/params/nav2_params.yaml`

1) 在 `bt_navigator.plugin_lib_names` 里加入 route BT 库（Jazzy 已提供）：
```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_and_track_route_bt_node
      - nav2_compute_route_bt_node    # 仅当 BT 里用 ComputeRoute 时需要
      # ...保留你原有列表
```

2) 指定默认 BT：
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "/home/suja/voxel_ws/src/robot_navigation/src/behavior_trees/navigate_on_route_graph_arch1_jazzy.xml"
```

---

## Step 3：Route Server 参数（让路径更“轨道化”）

同文件 `src/robot_navigation/src/params/nav2_params.yaml:595`，建议显式补齐核心参数：
```yaml
route_server:
  ros__parameters:
    path_density: 0.03        # 更密一点（原默认约 0.05）
    smooth_corners: false     # 关掉拐角平滑，避免抄近路
    smoothing_radius: 0.3     # 若 smooth_corners=true 时使用
    enable_nn_search: true
```

取舍：
- `smooth_corners=false` → 最严格贴路网，但拐角可能更硬，靠 Controller 处理。
- 若拐角跟踪抖动，再开平滑并减小 `smoothing_radius`。

---

## Step 4：Smoother + MPPI 强贴路径调参

1) ConstrainedSmoother 保持贴原路径  
文件：`src/robot_navigation/src/params/nav2_params.yaml:468`
```yaml
smoother_server:
  ros__parameters:
    SmoothPath:
      w_dist: 5.0        # 原来是 0.0，增加“贴原路径”约束
```

2) MPPI critic 权重偏向路径  
文件：`src/robot_navigation/src/params/nav2_params.yaml:169` 起
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      PathAlignCritic:
        cost_weight: 40.0     # 原 20.0，提高对齐强度
      PathFollowCritic:
        cost_weight: 15.0     # 原 5.0，提高沿路前进偏好
      PathAngleCritic:
        cost_weight: 8.0      # 原 5.0，拐角更愿意扭回来
      ObstaclesCritic:
        repulsion_weight: 0.15  # 原 0.2，轻微降低侧向躲避；collision/critical 权重保持不变
```

注意：这一步是在“严格贴路网”和“局部避障安全”之间做权衡。若现场障碍多、通道窄，`repulsion_weight` 不要再降。

---

## Step 5：编译与运行

```bash
colcon build --packages-select robot_navigation
source install/setup.zsh
ros2 launch robot_navigation navigation.launch.py
```

RViz 点 “2D Goal Pose”，应直接触发路网全局 path → MPPI 跟踪。

---

## Step 6：验证（rosbag + 指标）

录包：
```bash
ros2 bag record /tf /tf_static /Odometry /plan /route_graph \
  /local_costmap/costmap_raw /cmd_vel_nav
```

看指标：
- 横向偏差 RMS（对比 `/plan` 或你自发布的 route path）建议 < 0.1 m。
- 偏离发生时检查 `local_costmap` 是否有障碍推离；若是，则属于安全避障导致的“允许偏离”。

---

## 若出现 Gap（起点/终点离图太远）

这是文档里明确的 last‑mile 问题。两种补法：
1) 在 BT 里加 first/last‑mile：`ComputePathToPose` 连接到 route 起点/终点（Arch 4）。  
2) 继续用你已有脚本 `route_goal_bridge.py` 负责 first/last‑mile，再把中段交给 FollowPath（保持 strict on‑graph）。


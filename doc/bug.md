# wheel odom 无法读取的原因排查

## 现象
- `ros2 topic echo /diff_drive_controller/odom` 无数据，而 `/diff_drive_controller/cmd_vel_unstamped` 可以驱动底盘。
- 在其他工控机/老版本代码中可以看到轮速里程。

## 结论与定位
1. **diff_drive_controller 被配置为只发布 Twist，不发布位姿**  
   - `robot_base/config/controllers.yaml:17-37` 将 `odom_only_twist` 设为 `true` 并注释“使用完整的odom信息”。实际上在 `diff_drive_controller` 插件中，该参数为 “仅发布 twist（不计算 pose）”。因此当前控制器只会把角速度/线速度写进 `nav_msgs/Odometry.twist`，`pose` 始终为零，并且在没有订阅者的情况下 RCL 不会真正发送消息，看起来就像话题不存在。把该参数改为 `false` 或直接移除即可恢复完整里程。
2. **TF 也被关闭，导致即便强制发布 Odometry Pose 也没有 `odom->base_link` 链**  
   - 同一个配置里还把 `enable_odom_tf` 设为 `false`（`robot_base/config/controllers.yaml:24-27`）。如果外部又没有其它节点广播轮速 TF，那 diff_drive_controller 的 TF 分支完全被禁用。Fast-LIO 自身会发 `odom->base_link`，但是轮速链路这边没有任何 TF，所以 RViz/导航看不到“轮速里程”。
3. **导航栈已切换到 Fast-LIO `/Odometry`，不再订阅 `/diff_drive_controller/odom`**  
   - `robot_navigation/src/params/nav2_params.yaml:45-80、122-150、562-575` 中，BT Navigator、Controller Server、Velocity Smoother 均改成读取 `/Odometry`（Fast-LIO），与轮速话题完全脱钩；
   - `robot_navigation/src/launch/navigation.launch.py:127-136` 仅把 `cmd_vel_smoothed` 重映射到 `/diff_drive_controller/cmd_vel_unstamped`，但没有任何地方再监听 `/diff_drive_controller/odom`。

## 建议
1. 如果需要恢复轮速里程用于调试/冗余，请在 `robot_base/config/controllers.yaml` 中：
   - 删除 `odom_only_twist: true`，或者显式设置为 `false`；
   - 视情况把 `enable_odom_tf` 置 `true`；
   - 重新启动 `robot_base` bringup 后，通过 `ros2 topic echo /diff_drive_controller/odom` 验证。
2. 决定导航使用哪一路里程：
   - 如果继续使用 Fast-LIO，则保持 `/Odometry` 配置不动即可；
   - 如果希望在没有 Fast-LIO 的情况下 fallback 到轮速里程，需要给 `nav2_params.yaml` 增加一个模式切换（例如 `odom_topic` 来自 Launch 参数），并在 Launch 中正确 remap。
3. 无论使用哪一路里程，都要保证 TF 链完整（`odom -> base_link`）。如果依赖 Fast-LIO，请确保 Fast-LIO 在启动时就发布该 TF；如果切回轮速，就让 diff_drive_controller 打开 TF 发布。

# Final Plan: 恢复 wheel odom 实时更新

## 背景
- 当前 `/diff_drive_controller/odom` 中 pose 始终不变，doc/bug.md 早前分析指出 `robot_base/config/controllers.yaml` 将 `odom_only_twist` 设为 `true`，导致 diff_drive_controller 只发布 twist；同时 `enable_odom_tf=false`，因此 TF 链也缺失。
- FAST-LIO 侧 `send_odom_base_tf=true`，但如果只希望 wheel odom 反映下位机里程，就必须让 diff_drive_controller 重新积分并广播。

## 修改步骤
1. **恢复位姿积分与 TF**  
   - 编辑 `robot_base/config/controllers.yaml`：
     - 删除或改为 `odom_only_twist: false`；
     - 视需要把 `enable_odom_tf` 置 `true`，确保在无 FAST-LIO 时也有 `odom->base_footprint` TF。  
   - 这一步遵循 KISS/DRY，仅在单一配置处管理 odom 发布模式。
2. **重启 robot_base bringup**  
   - `ros2 launch robot_base bringup.launch.py`，确认 `controller_manager`、`robot_state_publisher` 正常启动。
3. **验证 wheel odom**  
   - `ros2 topic echo /diff_drive_controller/odom` 或在 RViz 查看 `odom` 轨迹；
   - 使用 `ros2 run tf2_tools view_frames.py` 确认 `odom -> base_footprint -> base_link` 链持续更新。
4. **与 FAST-LIO 的集成策略**  
   - 若仍以 FAST-LIO `/state_estimation` 作为导航主 odom，可保持 Nav2 remap；轮速 odom 则作为调试/冗余。
   - 若需在 FAST-LIO 不可用时切回轮速，考虑在 `nav2_params.yaml` 添加 `odom_topic` 可配置项，通过 Launch 选择 `/state_estimation` 或 `/diff_drive_controller/odom`。
5. **记录验证结论**  
   - 在 doc/bug.md 或新的调试记录中追加“已修改参数+测试结果”，便于日后回溯。

## 验证要求
- 机器人实际前后运动时，`nav_msgs/Odometry.pose.pose.position` 应随时间变化；
- TF 树中存在以 wheel odom 为源的 `odom` 帧，并能与 FAST-LIO 同时存在或通过参数开关选择。

## 影响面
- 仅影响 `robot_base` 控制器配置；对 FAST-LIO、导航栈无直接改动。
- 若同时启用 diff_drive_controller 与 FAST-LIO TF，需要在 TF 消费方明确选择，避免重复广播。

/**
 * 内置行为树模板（与当前 bt_visual_editor 的块/生成器兼容）
 *
 * 说明：
 * - 模板以 BT.CPP XML 表达，加载时会尝试转换为可编辑 Blockly 积木
 * - 模板里的黑板变量（如 {goal}/{path}/{goals}）需要运行时由 bt_navigator / 任务节点提供
 */

export const BT_TEMPLATES = [
  {
    id: 'nav2_official_nav_to_pose_replanning_recovery',
    name: '官方：单点导航 + 1Hz 重规划 + 恢复（NavigateToPose 默认）',
    description: '来自 nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml（ComputePathToPose + FollowPath + 清图/旋转/后退恢复）。',
    fileName: 'nav2_official_navigate_to_pose_w_replanning_and_recovery.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_nav_to_pose_goal_patience',
    name: '官方：单点导航 + 目标附近“耐心等待”策略',
    description: '来自 navigate_to_pose_w_replanning_goal_patience_and_recovery.xml：靠近目标时若路径显著变长，会 CancelControl 并等待障碍移开。',
    fileName: 'nav2_official_navigate_to_pose_w_replanning_goal_patience_and_recovery.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <ReactiveSequence name="MonitorAndFollowPath">
          <PathLongerOnApproach path="{path}" prox_len="3.0" length_factor="2.0">
            <RetryUntilSuccessful num_attempts="1">
              <SequenceStar name="CancelingControlAndWait">
                <CancelControl name="ControlCancel"/>
                <Wait wait_duration="5"/>
              </SequenceStar>
            </RetryUntilSuccessful>
          </PathLongerOnApproach>
          <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </RecoveryNode>
        </ReactiveSequence>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_replanning_only_goal_updated',
    name: '官方：仅目标更新时重规划（最简）',
    description: '来自 navigate_w_replanning_only_if_goal_is_updated.xml：GoalUpdatedController 包裹 ComputePathToPose。',
    fileName: 'nav2_official_navigate_w_replanning_only_if_goal_is_updated.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <GoalUpdatedController>
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </GoalUpdatedController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_replanning_only_if_path_invalid',
    name: '官方：仅路径失效时重规划（最简）',
    description: '来自 navigate_w_replanning_only_if_path_becomes_invalid.xml：IsPathValid + GlobalUpdatedGoal 共同判定是否需要重新规划。',
    fileName: 'nav2_official_navigate_w_replanning_only_if_path_becomes_invalid.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <Fallback>
          <ReactiveSequence>
            <Inverter><GlobalUpdatedGoal/></Inverter>
            <IsPathValid path="{path}"/>
          </ReactiveSequence>
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </Fallback>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_recovery_and_replanning_if_path_invalid',
    name: '官方：路径失效才重规划 + 完整恢复',
    description: '来自 navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml：带清图/旋转/等待/后退恢复，适合复杂场景。',
    fileName: 'nav2_official_navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence>
        <RateController hz="1.0" name="RateControllerComputePathToPose">
          <RecoveryNode number_of_retries="1" name="RecoveryComputePathToPose">
            <Fallback name="FallbackComputePathToPose">
              <ReactiveSequence name="CheckIfNewPathNeeded">
                <Inverter><GlobalUpdatedGoal/></Inverter>
                <IsPathValid path="{path}"/>
              </ReactiveSequence>
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </Fallback>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="RecoveryFollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="FallbackRecoveries">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin name="SpinRecovery" spin_dist="1.57"/>
          <Wait name="WaitRecovery" wait_duration="5"/>
          <BackUp name="BackUpRecovery" backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_consistent_replanning_timer_or_invalid',
    name: '官方：定时重规划（10s）或路径失效时重规划 + 恢复',
    description: '来自 nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml：PathExpiringTimer + IsPathValid 控制重规划。',
    fileName: 'nav2_official_nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="2.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <Fallback>
              <ReactiveSequence>
                <Inverter><PathExpiringTimer seconds="10" path="{path}"/></Inverter>
                <Inverter><GlobalUpdatedGoal/></Inverter>
                <IsPathValid path="{path}"/>
              </ReactiveSequence>
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </Fallback>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_replanning_time_1hz',
    name: '官方：1Hz 定时重规划（最简）',
    description: '来自 navigate_w_replanning_time.xml：RateController(1Hz) 周期性 ComputePathToPose。',
    fileName: 'nav2_official_navigate_w_replanning_time.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_replanning_distance_1m',
    name: '官方：每 1m 重规划（最简）',
    description: '来自 navigate_w_replanning_distance.xml：DistanceController(distance=1m) 触发重规划。',
    fileName: 'nav2_official_navigate_w_replanning_distance.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <DistanceController distance="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </DistanceController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_replanning_speed',
    name: '官方：随速度调频重规划（最简）',
    description: '来自 navigate_w_replanning_speed.xml：SpeedController 根据速度动态调整规划频率。',
    fileName: 'nav2_official_navigate_w_replanning_speed.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <SpeedController min_rate="0.1" max_rate="1.0" min_speed="0.0" max_speed="0.26">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </SpeedController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_official_follow_point',
    name: '官方：跟随动态目标（Follow Point）',
    description: '来自 follow_point.xml：GoalUpdater 更新目标，规划后截断路径并持续跟随（KeepRunningUntilFailure）。',
    fileName: 'nav2_official_follow_point.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <Sequence>
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
            <ComputePathToPose goal="{updated_goal}" path="{path}" planner_id="GridBased"/>
          </GoalUpdater>
          <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
        </Sequence>
      </RateController>
      <KeepRunningUntilFailure>
        <FollowPath path="{truncated_path}" controller_id="FollowPath"/>
      </KeepRunningUntilFailure>
    </PipelineSequence>
  </BehaviorTree>
</root>`
  },
  {
    id: 'nav2_patrol_through_poses',
    name: '官方：多点巡航 + 重规划 + 恢复（Through Poses）',
    description: '基于 Nav2 官方 NavigateThroughPoses 行为树：RemovePassedGoals + ComputePathThroughPoses + FollowPath（0.333Hz 重规划）+ 恢复动作。',
    fileName: 'template_patrol_through_poses.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="0.333">
          <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses">
            <ReactiveSequence>
              <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
              <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
            </ReactiveSequence>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>

        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>

      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>`
  },
  {
    id: 'route_graph_strict_follow',
    name: '路网：严格贴路网（ComputeAndTrackRoute + FollowPath）',
    description: '需要 route_server（ComputeAndTrackRoute）与 follow_path；0.5Hz 动态重规划，贴路网跟随。',
    fileName: 'template_route_graph_arch1.xml',
    xml: `<?xml version="1.0"?>
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
</root>`
  },
  {
    id: 'battery_guard_and_work',
    name: '策略：低电量回充，否则执行任务',
    description: '示例化“条件守卫”：低电量则 DockRobot，否则执行 NavigateToPose（{goal}）。',
    fileName: 'template_battery_guard.xml',
    xml: `<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveFallback name="BatteryGuard">
      <Sequence name="GoCharge">
        <IsBatteryLow min_battery="20" battery_topic="battery_status" is_voltage="false"/>
        <DockRobot dock_id="dock_1" navigate_to_staging_pose="true"/>
      </Sequence>
      <Sequence name="DoWork">
        <NavigateToPose goal="{goal}" server_name="navigate_to_pose"/>
      </Sequence>
    </ReactiveFallback>
  </BehaviorTree>
</root>`
  }
];

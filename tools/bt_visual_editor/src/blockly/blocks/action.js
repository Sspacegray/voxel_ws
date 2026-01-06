/**
 * 动作节点积木块定义
 * Action nodes for ROS2/Nav2
 * 包含中英文双语tooltip
 */

import * as Blockly from 'blockly';

// ========================================
// NavigateToPose - 导航到目标点
// ========================================
Blockly.Blocks['bt_navigate_to_pose'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('NavigateToPose');
        this.appendDummyInput()
            .appendField('goal 目标位姿')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'GOAL');
        this.appendDummyInput()
            .appendField('behavior_tree 行为树')
            .appendField(new Blockly.FieldTextInput(''), 'BEHAVIOR_TREE');
        this.appendDummyInput()
            .appendField('server_name 服务名')
            .appendField(new Blockly.FieldTextInput('navigate_to_pose'), 'SERVER_NAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(30);
        this.setTooltip('NavigateToPose (导航到目标点)\n' +
            '调用Nav2导航服务器，导航到指定目标位姿。\n' +
            '• goal: 目标位姿（黑板变量如{goal}）\n' +
            '• behavior_tree: 可选的子行为树文件\n' +
            '常用于：完整的点到点导航任务。');
    }
};

// ========================================
// NavigateThroughPoses - 巡航多点
// ========================================
Blockly.Blocks['bt_navigate_through_poses'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('NavigateThroughPoses');
        this.appendDummyInput()
            .appendField('goals 目标列表')
            .appendField(new Blockly.FieldTextInput('{goals}'), 'GOALS');
        this.appendDummyInput()
            .appendField('server_name 服务名')
            .appendField(new Blockly.FieldTextInput('navigate_through_poses'), 'SERVER_NAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(30);
        this.setTooltip('NavigateThroughPoses (多点巡航导航)\n' +
            '按顺序导航经过多个目标点。\n' +
            '• goals: 目标位姿列表（黑板变量）\n' +
            '常用于：巡逻、多点清扫等任务。');
    }
};

// ========================================
// ComputePathToPose - 计算路径
// ========================================
Blockly.Blocks['bt_compute_path_to_pose'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ComputePathToPose');
        this.appendDummyInput()
            .appendField('goal 目标位姿')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'GOAL');
        this.appendDummyInput()
            .appendField('path 输出路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'PATH');
        this.appendDummyInput()
            .appendField('planner_id 规划器')
            .appendField(new Blockly.FieldDropdown([
                ['GridBased', 'GridBased'],
                ['NavfnPlanner', 'NavfnPlanner'],
                ['SmacPlanner', 'SmacPlanner'],
                ['ThetaStarPlanner', 'ThetaStarPlanner']
            ]), 'PLANNER_ID');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(30);
        this.setTooltip('ComputePathToPose (计算路径)\n' +
            '调用规划器计算从当前位置到目标的路径。\n' +
            '• goal: 目标位姿\n' +
            '• path: 输出的路径（供FollowPath使用）\n' +
            '• planner_id: 使用的规划器\n' +
            '常用于：分离式导航（先规划再跟踪）。');
    }
};

// ========================================
// FollowPath - 跟随路径
// ========================================
Blockly.Blocks['bt_follow_path'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('FollowPath');
        this.appendDummyInput()
            .appendField('path 输入路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'PATH');
        this.appendDummyInput()
            .appendField('controller_id 控制器')
            .appendField(new Blockly.FieldDropdown([
                ['FollowPath', 'FollowPath'],
                ['DWB', 'DWB'],
                ['RPP', 'RPP'],
                ['TEB', 'TEB'],
                ['MPPI', 'MPPI']
            ]), 'CONTROLLER_ID');
        this.appendDummyInput()
            .appendField('error_code_id 错误码')
            .appendField(new Blockly.FieldTextInput('{follow_path_error_code}'), 'ERROR_CODE_ID');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(30);
        this.setTooltip('FollowPath (跟随路径)\n' +
            '调用控制器跟踪给定路径。\n' +
            '• path: 要跟踪的路径（来自ComputePathToPose）\n' +
            '• controller_id: 使用的控制器\n' +
            '• error_code_id: 错误码输出变量\n' +
            '常用于：配合ComputePathToPose进行导航。');
    }
};

// ========================================
// ComputeRoute - 计算路由
// ========================================
Blockly.Blocks['bt_compute_route'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ComputeRoute');
        this.appendDummyInput()
            .appendField('start_id 起点ID')
            .appendField(new Blockly.FieldTextInput('{start_id}'), 'START_ID');
        this.appendDummyInput()
            .appendField('goal_id 终点ID')
            .appendField(new Blockly.FieldTextInput('{goal_id}'), 'GOAL_ID');
        this.appendDummyInput()
            .appendField('route 路由')
            .appendField(new Blockly.FieldTextInput('{route}'), 'ROUTE');
        this.appendDummyInput()
            .appendField('path 路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'PATH');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(30);
        this.setTooltip('ComputeRoute (计算路由)\n' +
            '在路网图上计算从起点到终点的路由。\n' +
            '• start_id/goal_id: 路网节点ID\n' +
            '• route: 输出的路由信息\n' +
            '• path: 输出的路径\n' +
            '常用于：基于拓扑地图/路网的导航。');
    }
};

// ========================================
// ComputeAndTrackRoute - 计算并追踪路由（路网）
// ========================================
Blockly.Blocks['bt_compute_and_track_route'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ComputeAndTrackRoute');
        this.appendDummyInput()
            .appendField('goal 目标位姿')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'GOAL');
        this.appendDummyInput()
            .appendField('path 输出路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'PATH');
        this.appendDummyInput()
            .appendField('route 输出路由')
            .appendField(new Blockly.FieldTextInput('{route}'), 'ROUTE');
        this.appendDummyInput()
            .appendField('use_poses 使用位姿')
            .appendField(new Blockly.FieldCheckbox('TRUE'), 'USE_POSES');
        this.appendDummyInput()
            .appendField('error_code_id 错误码')
            .appendField(new Blockly.FieldTextInput('{compute_route_error_code}'), 'ERROR_CODE_ID');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(30);
        this.setTooltip('ComputeAndTrackRoute (计算并跟踪路由)\n' +
            '在路网上计算路由并跟踪执行。\n' +
            '• goal: 目标位姿\n' +
            '• use_poses: 是否使用精确位姿\n' +
            '常用于：Route Server路网导航。');
    }
};

// ========================================
// BackUp - 后退
// ========================================
Blockly.Blocks['bt_backup'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('BackUp');
        this.appendDummyInput()
            .appendField('backup_dist 后退距离(m)')
            .appendField(new Blockly.FieldNumber(0.3, 0.05, 2.0, 0.05), 'BACKUP_DIST');
        this.appendDummyInput()
            .appendField('backup_speed 后退速度(m/s)')
            .appendField(new Blockly.FieldNumber(0.15, 0.05, 0.5, 0.05), 'BACKUP_SPEED');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('BackUp (后退恢复动作)\n' +
            '让机器人向后移动指定距离。\n' +
            '• backup_dist: 后退距离（米）\n' +
            '• backup_speed: 后退速度（米/秒）\n' +
            '常用于：卡住时的恢复动作。');
    }
};

// ========================================
// Spin - 原地旋转
// ========================================
Blockly.Blocks['bt_spin'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Spin');
        this.appendDummyInput()
            .appendField('spin_dist 旋转角度(rad)')
            .appendField(new Blockly.FieldNumber(1.57, -6.28, 6.28, 0.1), 'SPIN_DIST');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('Spin (原地旋转恢复动作)\n' +
            '让机器人原地旋转指定角度。\n' +
            '• spin_dist: 旋转角度（弧度，1.57≈90°）\n' +
            '常用于：卡住时尝试找到出路。');
    }
};

// ========================================
// Wait - 等待
// ========================================
Blockly.Blocks['bt_wait'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Wait');
        this.appendDummyInput()
            .appendField('wait_duration 等待时间(s)')
            .appendField(new Blockly.FieldNumber(5.0, 0.1, 600, 0.1), 'WAIT_DURATION');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('Wait (等待动作)\n' +
            '暂停执行指定时间。\n' +
            '• wait_duration: 等待时长（秒）\n' +
            '常用于：等待障碍移开、定时任务等。');
    }
};

// ========================================
// ClearEntireCostmap - 清除代价地图
// ========================================
Blockly.Blocks['bt_clear_costmap'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ClearEntireCostmap');
        this.appendDummyInput()
            .appendField('service_name 服务选择')
            .appendField(new Blockly.FieldDropdown([
                ['global_costmap 全局', 'global_costmap/clear_entirely_global_costmap'],
                ['local_costmap 局部', 'local_costmap/clear_entirely_local_costmap']
            ]), 'SERVICE_NAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('ClearEntireCostmap (清除代价地图)\n' +
            '清除全部代价地图数据。\n' +
            '• global_costmap: 清除全局代价图\n' +
            '• local_costmap: 清除局部代价图\n' +
            '常用于：虚假障碍导致规划失败时的恢复。');
    }
};

// ========================================
// DockRobot - 对接充电
// ========================================
Blockly.Blocks['bt_dock_robot'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('DockRobot');
        this.appendDummyInput()
            .appendField('dock_id 充电桩ID')
            .appendField(new Blockly.FieldTextInput('dock_1'), 'DOCK_ID');
        this.appendDummyInput()
            .appendField('navigate_to_staging 导航到预位置')
            .appendField(new Blockly.FieldCheckbox('TRUE'), 'NAVIGATE_TO_STAGING');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('DockRobot (对接充电桩)\n' +
            '控制机器人对接到指定充电桩。\n' +
            '• dock_id: 充电桩标识\n' +
            '• navigate_to_staging: 是否先导航到预位置\n' +
            '常用于：低电量自动充电。');
    }
};

// ========================================
// UndockRobot - 离开充电桩
// ========================================
Blockly.Blocks['bt_undock_robot'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('UndockRobot');
        this.appendDummyInput()
            .appendField('dock_type 充电桩类型')
            .appendField(new Blockly.FieldTextInput(''), 'DOCK_TYPE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('UndockRobot (离开充电桩)\n' +
            '控制机器人从充电桩离开。\n' +
            '• dock_type: 可选，指定充电桩类型\n' +
            '常用于：充电完成后开始任务。');
    }
};

// ========================================
// SmoothPath - 平滑路径
// ========================================
Blockly.Blocks['bt_smooth_path'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('SmoothPath');
        this.appendDummyInput()
            .appendField('unsmoothed_path 原始路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'UNSMOOTHED_PATH');
        this.appendDummyInput()
            .appendField('smoothed_path 平滑后路径')
            .appendField(new Blockly.FieldTextInput('{smoothed_path}'), 'SMOOTHED_PATH');
        this.appendDummyInput()
            .appendField('smoother_id 平滑器')
            .appendField(new Blockly.FieldTextInput('SmoothPath'), 'SMOOTHER_ID');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(280);
        this.setTooltip('SmoothPath (路径平滑)\n' +
            '对规划的路径进行平滑处理。\n' +
            '• unsmoothed_path: 输入的原始路径\n' +
            '• smoothed_path: 输出的平滑路径\n' +
            '常用于：提升路径跟踪舒适度。');
    }
};

// ========================================
// TruncatePath - 截断路径
// ========================================
Blockly.Blocks['bt_truncate_path'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('TruncatePath');
        this.appendDummyInput()
            .appendField('input_path 输入路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'INPUT_PATH');
        this.appendDummyInput()
            .appendField('output_path 输出路径')
            .appendField(new Blockly.FieldTextInput('{truncated_path}'), 'OUTPUT_PATH');
        this.appendDummyInput()
            .appendField('distance 截断距离(m)')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 10, 0.1), 'DISTANCE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(280);
        this.setTooltip('TruncatePath (截断路径)\n' +
            '截掉路径末端指定距离。\n' +
            '• distance: 截断距离（米）\n' +
            '常用于：目标点前留出缓冲区。');
    }
};

// ========================================
// ReinitializeGlobalLocalization - 重新初始化定位
// ========================================
Blockly.Blocks['bt_reinitialize_localization'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ReinitializeGlobalLocalization');
        this.appendDummyInput()
            .appendField('service_name 服务名')
            .appendField(new Blockly.FieldTextInput('reinitialize_global_localization'), 'SERVICE_NAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('ReinitializeGlobalLocalization (重新初始化定位)\n' +
            '触发AMCL全局重定位。\n' +
            '在整个地图上重新撒粒子。\n' +
            '常用于：定位丢失时的恢复。');
    }
};

// ========================================
// ControllerSelector - 控制器选择
// ========================================
Blockly.Blocks['bt_controller_selector'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ControllerSelector');
        this.appendDummyInput()
            .appendField('default_controller 默认')
            .appendField(new Blockly.FieldDropdown([
                ['FollowPath', 'FollowPath'],
                ['DWB', 'DWB'],
                ['RPP', 'RPP'],
                ['TEB', 'TEB'],
                ['MPPI', 'MPPI']
            ]), 'DEFAULT_CONTROLLER');
        this.appendDummyInput()
            .appendField('selected_controller 选中')
            .appendField(new Blockly.FieldTextInput('{selected_controller}'), 'SELECTED_CONTROLLER');
        this.appendDummyInput()
            .appendField('topic_name 话题名')
            .appendField(new Blockly.FieldTextInput('controller_selector'), 'TOPIC_NAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(225);
        this.setTooltip('ControllerSelector (控制器选择器)\n' +
            '动态选择路径跟踪控制器。\n' +
            '• DWB: 动态窗口法\n' +
            '• RPP: 纯追踪法\n' +
            '• TEB: 时间弹性带\n' +
            '• MPPI: 模型预测控制');
    }
};

// ========================================
// PlannerSelector - 规划器选择
// ========================================
Blockly.Blocks['bt_planner_selector'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('PlannerSelector');
        this.appendDummyInput()
            .appendField('default_planner 默认')
            .appendField(new Blockly.FieldDropdown([
                ['GridBased', 'GridBased'],
                ['NavfnPlanner', 'NavfnPlanner'],
                ['SmacPlanner', 'SmacPlanner'],
                ['ThetaStarPlanner', 'ThetaStarPlanner']
            ]), 'DEFAULT_PLANNER');
        this.appendDummyInput()
            .appendField('selected_planner 选中')
            .appendField(new Blockly.FieldTextInput('{selected_planner}'), 'SELECTED_PLANNER');
        this.appendDummyInput()
            .appendField('topic_name 话题名')
            .appendField(new Blockly.FieldTextInput('planner_selector'), 'TOPIC_NAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(225);
        this.setTooltip('PlannerSelector (规划器选择器)\n' +
            '动态选择路径规划器。\n' +
            '• GridBased: 2D A*\n' +
            '• NavfnPlanner: Dijkstra/A*\n' +
            '• SmacPlanner: Hybrid A*\n' +
            '• ThetaStarPlanner: Theta*');
    }
};

export default Blockly.Blocks;

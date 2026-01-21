/**
 * 条件节点积木块定义
 * Condition nodes for behavior trees
 */

import * as Blockly from 'blockly';

// ========================================
// GoalReached - 到达目标检测
// ========================================
Blockly.Blocks['bt_goal_reached'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('GoalReached 到达目标');
        this.appendDummyInput()
            .appendField('goal 目标')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'GOAL');
        this.appendDummyInput()
            .appendField('robot_base_frame 机器人坐标系')
            .appendField(new Blockly.FieldTextInput('base_link'), 'ROBOT_BASE_FRAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('GoalReached (到达目标)\n' +
            '判断机器人是否已到达目标点。\n' +
            '• goal: 目标位姿（黑板变量如 {goal}）\n' +
            '• robot_base_frame: 机器人基坐标系（默认 base_link）');
    }
};

// ========================================
// GoalUpdated - 目标已更新检测
// ========================================
Blockly.Blocks['bt_goal_updated'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('GoalUpdated 目标更新');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('GoalUpdated (目标更新)\n' +
            '判断目标是否发生更新（例如 RViz 重新点目标）。');
    }
};

// ========================================
// IsBatteryLow - 低电量检测
// ========================================
Blockly.Blocks['bt_is_battery_low'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsBatteryLow 低电量');
        this.appendDummyInput()
            .appendField('min_battery 阈值(%)')
            .appendField(new Blockly.FieldNumber(20, 0, 100, 1), 'MIN_BATTERY');
        this.appendDummyInput()
            .appendField('battery_topic 电池话题')
            .appendField(new Blockly.FieldTextInput('battery_status'), 'BATTERY_TOPIC');
        this.appendDummyInput()
            .appendField('is_voltage 用电压判断')
            .appendField(new Blockly.FieldCheckbox('FALSE'), 'IS_VOLTAGE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(330);
        this.setTooltip('IsBatteryLow (低电量)\n' +
            '判断电量是否低于阈值。\n' +
            '• min_battery: 阈值（百分比）\n' +
            '• battery_topic: 电池状态话题\n' +
            '• is_voltage: 是否使用电压字段判断');
    }
};

// ========================================
// IsBatteryCharging - 充电状态检测
// ========================================
Blockly.Blocks['bt_is_battery_charging'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsBatteryCharging 充电中');
        this.appendDummyInput()
            .appendField('battery_topic 电池话题')
            .appendField(new Blockly.FieldTextInput('battery_status'), 'BATTERY_TOPIC');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('IsBatteryCharging (充电中)\n' +
            '判断当前是否处于充电状态。\n' +
            '• battery_topic: 电池状态话题');
    }
};

// ========================================
// IsPathValid - 路径有效性检测
// ========================================
Blockly.Blocks['bt_is_path_valid'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsPathValid 路径有效');
        this.appendDummyInput()
            .appendField('path 路径')
            .appendField(new Blockly.FieldTextInput('{path}'), 'PATH');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('IsPathValid (路径有效)\n' +
            '检查当前路径是否仍然有效（可用/可跟踪）。\n' +
            '• path: 路径黑板变量（如 {path}）');
    }
};

// ========================================
// IsStuck - 卡住检测
// ========================================
Blockly.Blocks['bt_is_stuck'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsStuck 卡住检测');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('IsStuck (卡住检测)\n' +
            '判断机器人是否卡住/无法前进（一般由 Nav2 进度检查器推断）。');
    }
};

// ========================================
// IsStopped - 停止检测
// ========================================
Blockly.Blocks['bt_is_stopped'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsStopped 停止检测');
        this.appendDummyInput()
            .appendField('velocity_threshold 速度阈值')
            .appendField(new Blockly.FieldNumber(0.01, 0, 1, 0.01), 'VELOCITY_THRESHOLD');
        this.appendDummyInput()
            .appendField('duration_stopped 停止时长(ms)')
            .appendField(new Blockly.FieldNumber(500, 0, 10000, 100), 'DURATION_STOPPED');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(225);
        this.setTooltip('IsStopped (停止检测)\n' +
            '判断机器人是否持续低于速度阈值（停止）。\n' +
            '• velocity_threshold: 速度阈值\n' +
            '• duration_stopped: 判定停止所需持续时长');
    }
};

// ========================================
// InitialPoseReceived - 初始位姿接收检测
// ========================================
Blockly.Blocks['bt_initial_pose_received'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('InitialPoseReceived 初始位姿已收到');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('InitialPoseReceived (初始位姿已收到)\n' +
            '判断定位系统是否已经收到初始位姿（例如 AMCL 初始位姿设置）。');
    }
};

// ========================================
// TimeExpired - 时间到期检测
// ========================================
Blockly.Blocks['bt_time_expired'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('TimeExpired 时间到期');
        this.appendDummyInput()
            .appendField('seconds 秒')
            .appendField(new Blockly.FieldNumber(10, 0.1, 3600, 0.1), 'SECONDS');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('TimeExpired (时间到期)\n' +
            '判断计时器是否超时。\n' +
            '• seconds: 超时秒数');
    }
};

// ========================================
// DistanceTraveled - 行驶距离检测
// ========================================
Blockly.Blocks['bt_distance_traveled'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('DistanceTraveled 行驶距离');
        this.appendDummyInput()
            .appendField('distance 距离(m)')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'DISTANCE');
        this.appendDummyInput()
            .appendField('global_frame 全局坐标系')
            .appendField(new Blockly.FieldTextInput('map'), 'GLOBAL_FRAME');
        this.appendDummyInput()
            .appendField('robot_base_frame 机器人坐标系')
            .appendField(new Blockly.FieldTextInput('base_link'), 'ROBOT_BASE_FRAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('DistanceTraveled (行驶距离)\n' +
            '判断是否已行驶超过指定距离。\n' +
            '• distance: 里程阈值\n' +
            '• global_frame/robot_base_frame: 参考坐标系');
    }
};

// ========================================
// TransformAvailable - TF变换可用检测
// ========================================
Blockly.Blocks['bt_transform_available'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('TransformAvailable TF 可用');
        this.appendDummyInput()
            .appendField('parent 父坐标系')
            .appendField(new Blockly.FieldTextInput('map'), 'PARENT');
        this.appendDummyInput()
            .appendField('child 子坐标系')
            .appendField(new Blockly.FieldTextInput('base_link'), 'CHILD');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(195);
        this.setTooltip('TransformAvailable (TF 可用)\n' +
            '判断 TF 变换是否可获取。\n' +
            '• parent/child: 变换的父/子坐标系');
    }
};

// ========================================
// WouldAControllerRecoveryHelp - 控制器恢复是否有帮助
// ========================================
Blockly.Blocks['bt_would_controller_recovery_help'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('WouldAControllerRecoveryHelp 控制器恢复建议');
        this.appendDummyInput()
            .appendField('error_code 错误码')
            .appendField(new Blockly.FieldTextInput('{follow_path_error_code}'), 'ERROR_CODE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(330);
        this.setTooltip('WouldAControllerRecoveryHelp (控制器恢复建议)\n' +
            '根据控制器错误码判断是否需要触发控制器相关恢复。\n' +
            '• error_code: 错误码黑板变量（如 {follow_path_error_code}）');
    }
};

// ========================================
// WouldAPlannerRecoveryHelp - 规划器恢复是否有帮助
// ========================================
Blockly.Blocks['bt_would_planner_recovery_help'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('WouldAPlannerRecoveryHelp 规划器恢复建议');
        this.appendDummyInput()
            .appendField('error_code 错误码')
            .appendField(new Blockly.FieldTextInput('{compute_path_error_code}'), 'ERROR_CODE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(330);
        this.setTooltip('WouldAPlannerRecoveryHelp (规划器恢复建议)\n' +
            '根据规划器错误码判断是否需要触发规划器相关恢复。\n' +
            '• error_code: 错误码黑板变量（如 {compute_path_error_code}）');
    }
};

export default Blockly.Blocks;

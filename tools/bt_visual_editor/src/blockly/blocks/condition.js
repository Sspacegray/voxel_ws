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
            .appendField('GoalReached');
        this.appendDummyInput()
            .appendField('goal')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'GOAL');
        this.appendDummyInput()
            .appendField('robot_base_frame')
            .appendField(new Blockly.FieldTextInput('base_link'), 'ROBOT_BASE_FRAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('Check if goal is reached');
    }
};

// ========================================
// GoalUpdated - 目标已更新检测
// ========================================
Blockly.Blocks['bt_goal_updated'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('GoalUpdated');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('Check if goal was updated');
    }
};

// ========================================
// IsBatteryLow - 低电量检测
// ========================================
Blockly.Blocks['bt_is_battery_low'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsBatteryLow');
        this.appendDummyInput()
            .appendField('min_battery')
            .appendField(new Blockly.FieldNumber(20, 0, 100, 1), 'MIN_BATTERY');
        this.appendDummyInput()
            .appendField('battery_topic')
            .appendField(new Blockly.FieldTextInput('battery_status'), 'BATTERY_TOPIC');
        this.appendDummyInput()
            .appendField('is_voltage')
            .appendField(new Blockly.FieldCheckbox('FALSE'), 'IS_VOLTAGE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(330);
        this.setTooltip('Check if battery is low');
    }
};

// ========================================
// IsBatteryCharging - 充电状态检测
// ========================================
Blockly.Blocks['bt_is_battery_charging'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsBatteryCharging');
        this.appendDummyInput()
            .appendField('battery_topic')
            .appendField(new Blockly.FieldTextInput('battery_status'), 'BATTERY_TOPIC');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('Check if battery is charging');
    }
};

// ========================================
// IsPathValid - 路径有效性检测
// ========================================
Blockly.Blocks['bt_is_path_valid'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsPathValid');
        this.appendDummyInput()
            .appendField('path')
            .appendField(new Blockly.FieldTextInput('{path}'), 'PATH');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('Check if path is valid');
    }
};

// ========================================
// IsStuck - 卡住检测
// ========================================
Blockly.Blocks['bt_is_stuck'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsStuck');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('Check if robot is stuck');
    }
};

// ========================================
// IsStopped - 停止检测
// ========================================
Blockly.Blocks['bt_is_stopped'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('IsStopped');
        this.appendDummyInput()
            .appendField('velocity_threshold')
            .appendField(new Blockly.FieldNumber(0.01, 0, 1, 0.01), 'VELOCITY_THRESHOLD');
        this.appendDummyInput()
            .appendField('duration_stopped (ms)')
            .appendField(new Blockly.FieldNumber(500, 0, 10000, 100), 'DURATION_STOPPED');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(225);
        this.setTooltip('Check if robot is stopped');
    }
};

// ========================================
// InitialPoseReceived - 初始位姿接收检测
// ========================================
Blockly.Blocks['bt_initial_pose_received'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('InitialPoseReceived');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('Check if initial pose was received');
    }
};

// ========================================
// TimeExpired - 时间到期检测
// ========================================
Blockly.Blocks['bt_time_expired'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('TimeExpired');
        this.appendDummyInput()
            .appendField('seconds')
            .appendField(new Blockly.FieldNumber(10, 0.1, 3600, 0.1), 'SECONDS');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('Check if time has expired');
    }
};

// ========================================
// DistanceTraveled - 行驶距离检测
// ========================================
Blockly.Blocks['bt_distance_traveled'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('DistanceTraveled');
        this.appendDummyInput()
            .appendField('distance (m)')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'DISTANCE');
        this.appendDummyInput()
            .appendField('global_frame')
            .appendField(new Blockly.FieldTextInput('map'), 'GLOBAL_FRAME');
        this.appendDummyInput()
            .appendField('robot_base_frame')
            .appendField(new Blockly.FieldTextInput('base_link'), 'ROBOT_BASE_FRAME');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Check if distance was traveled');
    }
};

// ========================================
// TransformAvailable - TF变换可用检测
// ========================================
Blockly.Blocks['bt_transform_available'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('TransformAvailable');
        this.appendDummyInput()
            .appendField('parent')
            .appendField(new Blockly.FieldTextInput('map'), 'PARENT');
        this.appendDummyInput()
            .appendField('child')
            .appendField(new Blockly.FieldTextInput('base_link'), 'CHILD');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(195);
        this.setTooltip('Check if TF transform is available');
    }
};

// ========================================
// WouldAControllerRecoveryHelp - 控制器恢复是否有帮助
// ========================================
Blockly.Blocks['bt_would_controller_recovery_help'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('WouldAControllerRecoveryHelp');
        this.appendDummyInput()
            .appendField('error_code')
            .appendField(new Blockly.FieldTextInput('{follow_path_error_code}'), 'ERROR_CODE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(330);
        this.setTooltip('Check if controller recovery would help');
    }
};

// ========================================
// WouldAPlannerRecoveryHelp - 规划器恢复是否有帮助
// ========================================
Blockly.Blocks['bt_would_planner_recovery_help'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('WouldAPlannerRecoveryHelp');
        this.appendDummyInput()
            .appendField('error_code')
            .appendField(new Blockly.FieldTextInput('{compute_path_error_code}'), 'ERROR_CODE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(330);
        this.setTooltip('Check if planner recovery would help');
    }
};

export default Blockly.Blocks;

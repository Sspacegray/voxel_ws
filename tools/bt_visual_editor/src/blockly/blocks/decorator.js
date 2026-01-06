/**
 * 装饰节点积木块定义
 * Decorator nodes for behavior trees
 */

import * as Blockly from 'blockly';

// ========================================
// RateController - 速率控制器
// ========================================
Blockly.Blocks['bt_rate_controller'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('RateController');
        this.appendDummyInput()
            .appendField('hz')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'HZ');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Execute child at specified frequency');
    }
};

// ========================================
// DistanceController - 距离控制器
// ========================================
Blockly.Blocks['bt_distance_controller'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('DistanceController');
        this.appendDummyInput()
            .appendField('distance (m)')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'DISTANCE');
        this.appendDummyInput()
            .appendField('global_frame')
            .appendField(new Blockly.FieldTextInput('map'), 'GLOBAL_FRAME');
        this.appendDummyInput()
            .appendField('robot_frame')
            .appendField(new Blockly.FieldTextInput('base_link'), 'ROBOT_FRAME');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Execute child after robot travels distance');
    }
};

// ========================================
// SpeedController - 速度控制器
// ========================================
Blockly.Blocks['bt_speed_controller'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('SpeedController');
        this.appendDummyInput()
            .appendField('min_rate')
            .appendField(new Blockly.FieldNumber(0.1, 0.01, 10, 0.01), 'MIN_RATE')
            .appendField('max_rate')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'MAX_RATE');
        this.appendDummyInput()
            .appendField('min_speed')
            .appendField(new Blockly.FieldNumber(0.0, 0, 10, 0.1), 'MIN_SPEED')
            .appendField('max_speed')
            .appendField(new Blockly.FieldNumber(0.5, 0, 10, 0.1), 'MAX_SPEED');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Adjust rate based on robot speed');
    }
};

// ========================================
// GoalUpdater - 目标更新器
// ========================================
Blockly.Blocks['bt_goal_updater'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('GoalUpdater');
        this.appendDummyInput()
            .appendField('input_goal')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'INPUT_GOAL');
        this.appendDummyInput()
            .appendField('output_goal')
            .appendField(new Blockly.FieldTextInput('{updated_goal}'), 'OUTPUT_GOAL');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Listen for goal updates');
    }
};

// ========================================
// SingleTrigger - 单次触发
// ========================================
Blockly.Blocks['bt_single_trigger'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('SingleTrigger');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Execute child only once');
    }
};

// ========================================
// Inverter - 反转器
// ========================================
Blockly.Blocks['bt_inverter'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Inverter');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Invert child result');
    }
};

// ========================================
// ForceSuccess - 强制成功
// ========================================
Blockly.Blocks['bt_force_success'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ForceSuccess');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Always return success');
    }
};

// ========================================
// ForceFailure - 强制失败
// ========================================
Blockly.Blocks['bt_force_failure'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ForceFailure');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Always return failure');
    }
};

// ========================================
// Repeat - 重复执行
// ========================================
Blockly.Blocks['bt_repeat'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Repeat');
        this.appendDummyInput()
            .appendField('num_cycles')
            .appendField(new Blockly.FieldNumber(3, 1, 1000, 1), 'NUM_CYCLES');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Repeat child N times');
    }
};

// ========================================
// Timeout - 超时控制
// ========================================
Blockly.Blocks['bt_timeout'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Timeout');
        this.appendDummyInput()
            .appendField('seconds')
            .appendField(new Blockly.FieldNumber(30, 0.1, 3600, 0.1), 'TIMEOUT_SEC');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Fail if child takes too long');
    }
};

export default Blockly.Blocks;

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
            .appendField('RateController 频率控制');
        this.appendDummyInput()
            .appendField('hz 频率(Hz)')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'HZ');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('RateController (频率控制)\n' +
            '按指定频率 tick 子节点。\n' +
            '常用于：降低规划/检测频率，避免资源占用过高。');
    }
};

// ========================================
// DistanceController - 距离控制器
// ========================================
Blockly.Blocks['bt_distance_controller'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('DistanceController 距离触发');
        this.appendDummyInput()
            .appendField('distance 距离(m)')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'DISTANCE');
        this.appendDummyInput()
            .appendField('global_frame 全局坐标系')
            .appendField(new Blockly.FieldTextInput('map'), 'GLOBAL_FRAME');
        this.appendDummyInput()
            .appendField('robot_frame 机器人坐标系')
            .appendField(new Blockly.FieldTextInput('base_link'), 'ROBOT_FRAME');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('DistanceController (距离触发)\n' +
            '机器人累计行驶超过指定距离后才 tick 子节点。\n' +
            '常用于：里程触发的周期任务（例如每走 1m 做一次检查）。');
    }
};

// ========================================
// SpeedController - 速度控制器
// ========================================
Blockly.Blocks['bt_speed_controller'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('SpeedController 速度调频');
        this.appendDummyInput()
            .appendField('min_rate 最小频率')
            .appendField(new Blockly.FieldNumber(0.1, 0.01, 10, 0.01), 'MIN_RATE')
            .appendField('max_rate 最大频率')
            .appendField(new Blockly.FieldNumber(1.0, 0.1, 100, 0.1), 'MAX_RATE');
        this.appendDummyInput()
            .appendField('min_speed 最小速度')
            .appendField(new Blockly.FieldNumber(0.0, 0, 10, 0.1), 'MIN_SPEED')
            .appendField('max_speed 最大速度')
            .appendField(new Blockly.FieldNumber(0.5, 0, 10, 0.1), 'MAX_SPEED');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('SpeedController (速度调频)\n' +
            '根据机器人速度动态调整 tick 频率。\n' +
            '常用于：走得快时更频繁检查/重规划，走得慢时降低开销。');
    }
};

// ========================================
// GoalUpdater - 目标更新器
// ========================================
Blockly.Blocks['bt_goal_updater'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('GoalUpdater 目标更新');
        this.appendDummyInput()
            .appendField('input_goal 输入目标')
            .appendField(new Blockly.FieldTextInput('{goal}'), 'INPUT_GOAL');
        this.appendDummyInput()
            .appendField('output_goal 输出目标')
            .appendField(new Blockly.FieldTextInput('{updated_goal}'), 'OUTPUT_GOAL');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('GoalUpdater (目标更新)\n' +
            '监听目标更新并写入黑板变量。\n' +
            '常用于：允许外部更新目标点（如 RViz 重新点目标）。');
    }
};

// ========================================
// SingleTrigger - 单次触发
// ========================================
Blockly.Blocks['bt_single_trigger'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('SingleTrigger 单次触发');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('SingleTrigger (单次触发)\n' +
            '仅第一次 tick 时执行子节点，之后直接返回成功。\n' +
            '常用于：只需初始化一次的动作。');
    }
};

// ========================================
// Inverter - 反转器
// ========================================
Blockly.Blocks['bt_inverter'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Inverter 取反');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Inverter (取反)\n' +
            '将子节点返回值 Success/Failure 取反。\n' +
            '常用于：条件取反。');
    }
};

// ========================================
// ForceSuccess - 强制成功
// ========================================
Blockly.Blocks['bt_force_success'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ForceSuccess 强制成功');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('ForceSuccess (强制成功)\n' +
            '无论子节点结果如何，都返回 Success。\n' +
            '常用于：忽略某些失败分支，继续执行后续流程。');
    }
};

// ========================================
// ForceFailure - 强制失败
// ========================================
Blockly.Blocks['bt_force_failure'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ForceFailure 强制失败');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('ForceFailure (强制失败)\n' +
            '无论子节点结果如何，都返回 Failure。\n' +
            '常用于：触发上层 Fallback/Recovery 的后续策略。');
    }
};

// ========================================
// Repeat - 重复执行
// ========================================
Blockly.Blocks['bt_repeat'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Repeat 重复');
        this.appendDummyInput()
            .appendField('num_cycles 次数')
            .appendField(new Blockly.FieldNumber(3, 1, 1000, 1), 'NUM_CYCLES');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Repeat (重复)\n' +
            '重复执行子节点 N 次。\n' +
            '常用于：周期性尝试、重复动作。');
    }
};

// ========================================
// Timeout - 超时控制
// ========================================
Blockly.Blocks['bt_timeout'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Timeout 超时');
        this.appendDummyInput()
            .appendField('seconds 秒')
            .appendField(new Blockly.FieldNumber(30, 0.1, 3600, 0.1), 'TIMEOUT_SEC');
        this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(160);
        this.setTooltip('Timeout (超时)\n' +
            '如果子节点执行超过指定时间则返回 Failure。\n' +
            '常用于：对可能卡住的行为加时间上限。');
    }
};

export default Blockly.Blocks;

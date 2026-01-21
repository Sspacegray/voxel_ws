/**
 * 控制节点积木块定义
 * Control nodes for behavior trees
 * 包含中英文双语tooltip
 */

import * as Blockly from 'blockly';

// ========================================
// Sequence - 顺序执行节点
// ========================================
Blockly.Blocks['bt_sequence'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Sequence 顺序')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('Sequence (顺序节点)\n' +
            '按顺序执行所有子节点，直到某个子节点返回失败。\n' +
            '如果所有子节点都成功，则返回成功。\n' +
            '常用于：执行一系列必须全部完成的任务。');
        this.setHelpUrl('');
    }
};

// ========================================
// PipelineSequence - 管道顺序节点
// ========================================
Blockly.Blocks['bt_pipeline_sequence'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('PipelineSequence 管道顺序')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('PipelineSequence (管道顺序节点)\n' +
            '类似Sequence，但会并行启动第一个和第二个子节点。\n' +
            '当第一个完成后，第二个和第三个并行，以此类推。\n' +
            '常用于：路径规划和跟踪的流水线处理。');
        this.setHelpUrl('');
    }
};

// ========================================
// Fallback - 备选执行节点
// ========================================
Blockly.Blocks['bt_fallback'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Fallback 选择')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('Fallback (备选节点/选择器)\n' +
            '按顺序尝试子节点，直到某个成功为止。\n' +
            '如果所有子节点都失败，则返回失败。\n' +
            '常用于：尝试多种方案直到一种成功。');
        this.setHelpUrl('');
    }
};

// ========================================
// ReactiveFallback - 响应式备选节点
// ========================================
Blockly.Blocks['bt_reactive_fallback'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ReactiveFallback 响应选择')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('ReactiveFallback (响应式备选节点)\n' +
            '每次tick都从第一个子节点开始重新检查。\n' +
            '如果前面的条件变为成功，会中断正在运行的子节点。\n' +
            '常用于：优先级条件检查 + 备选执行。');
        this.setHelpUrl('');
    }
};

// ========================================
// ReactiveSequence - 响应式顺序节点
// ========================================
Blockly.Blocks['bt_reactive_sequence'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('ReactiveSequence 响应顺序')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('ReactiveSequence (响应式顺序节点)\n' +
            '每次tick从第一个子节点开始检查。\n' +
            '如果前面的条件变为失败，会中断正在运行的子节点。\n' +
            '常用于：条件检查 + 动作执行（条件持续有效才继续）。');
        this.setHelpUrl('');
    }
};

// ========================================
// RecoveryNode - 恢复节点
// ========================================
Blockly.Blocks['bt_recovery'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('RecoveryNode 恢复')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendDummyInput()
            .appendField('retries 重试次数')
            .appendField(new Blockly.FieldNumber(3, 0, 20, 1), 'RETRIES');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('RecoveryNode (恢复节点)\n' +
            '第一个子节点执行失败时，执行第二个子节点（恢复动作）。\n' +
            '恢复成功后重试第一个子节点，最多重试指定次数。\n' +
            '常用于：导航失败 -> 清除代价图 -> 重新尝试。');
        this.setHelpUrl('');
    }
};

// ========================================
// RoundRobin - 轮询执行节点
// ========================================
Blockly.Blocks['bt_round_robin'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('RoundRobin 轮询')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('RoundRobin (轮询节点)\n' +
            '每次tick执行下一个子节点（循环）。\n' +
            '记住上次执行位置，下次从下一个开始。\n' +
            '常用于：多个任务轮流执行。');
        this.setHelpUrl('');
    }
};

// ========================================
// Parallel - 并行执行节点
// ========================================
Blockly.Blocks['bt_parallel'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('Parallel 并行')
            .appendField(new Blockly.FieldTextInput(''), 'NAME');
        this.appendDummyInput()
            .appendField('success 成功阈值')
            .appendField(new Blockly.FieldNumber(1, 1, 10, 1), 'SUCCESS_THRESHOLD')
            .appendField('failure 失败阈值')
            .appendField(new Blockly.FieldNumber(1, 1, 10, 1), 'FAILURE_THRESHOLD');
        this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children 子节点');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('Parallel (并行节点)\n' +
            '同时执行所有子节点。\n' +
            '达到成功阈值则成功，达到失败阈值则失败。\n' +
            '常用于：同时监控多个条件。');
        this.setHelpUrl('');
    }
};

export default Blockly.Blocks;

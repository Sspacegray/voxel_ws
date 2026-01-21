/**
 * 通用/未知节点积木块
 * 用于导入 XML 时对未建模节点进行保真编辑（tag + attrs + children）
 */

import * as Blockly from 'blockly';

Blockly.Blocks['bt_unknown_node'] = {
  init: function () {
    this.appendDummyInput()
      .appendField('XMLNode 通用节点')
      .appendField(new Blockly.FieldTextInput('CustomNode'), 'TAG');

    this.appendDummyInput()
      .appendField('attrs 属性')
      .appendField(new Blockly.FieldTextInput(''), 'ATTRS');

    this.appendStatementInput('CHILDREN')
      .setCheck(null)
      .appendField('children 子节点');

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(75);
    this.setTooltip('XMLNode (通用/未知节点)\n' +
      '用于导入 XML 时对未建模节点进行保真编辑：tag + attrs + children。\n' +
      '常用于：自定义 BT 节点尚未加入节点库（TreeNodesModel）时的临时编辑。');
  }
};

export default Blockly.Blocks;

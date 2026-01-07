/**
 * 通用/未知节点积木块
 * 用于导入 XML 时对未建模节点进行保真编辑（tag + attrs + children）
 */

import * as Blockly from 'blockly';

Blockly.Blocks['bt_unknown_node'] = {
  init: function () {
    this.appendDummyInput()
      .appendField('XMLNode')
      .appendField(new Blockly.FieldTextInput('CustomNode'), 'TAG');

    this.appendDummyInput()
      .appendField('attrs')
      .appendField(new Blockly.FieldTextInput(''), 'ATTRS');

    this.appendStatementInput('CHILDREN')
      .setCheck(null)
      .appendField('children');

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(75);
    this.setTooltip('Generic XML node (fallback for custom/unknown BT nodes)');
  }
};

export default Blockly.Blocks;


/**
 * BT 节点描述（Groot/BT.CPP TreeNodesModel）加载器
 *
 * 目标：实现“加节点描述就自动出现积木”
 * - 支持从 nav2_tree_nodes.xml 或自定义 TreeNodesModel XML 导入
 * - 自动注册 Blockly blocks + 对应 XML generator
 */

import * as Blockly from 'blockly';

const state = {
  // nodeId -> blockType
  idToBlockType: new Map(),
  // blockType -> nodeDef
  blockTypeToDef: new Map(),
  // kind -> [{id, blockType}]
  kinds: new Map()
};

function sanitizeId(id) {
  return String(id).replace(/[^a-zA-Z0-9_]/g, '_');
}

function kindToColour(kind) {
  switch (kind) {
    case 'Control':
      return 210;
    case 'Decorator':
      return 160;
    case 'Action':
      return 30;
    case 'Condition':
      return 270;
    default:
      return 75;
  }
}

function parsePorts(nodeEl) {
  const ports = [];
  for (const portEl of Array.from(nodeEl.children)) {
    if (portEl.tagName !== 'input_port' && portEl.tagName !== 'output_port') continue;
    const direction = portEl.tagName === 'input_port' ? 'in' : 'out';
    const name = portEl.getAttribute('name');
    if (!name) continue;
    ports.push({
      name,
      direction,
      type: portEl.getAttribute('type') || null,
      default: portEl.getAttribute('default') || null
    });
  }
  return ports;
}

function defaultPortValue(port) {
  if (port.default != null) return port.default;
  if (port.type === 'bool') return 'false';
  return `{${port.name}}`;
}

function buildPortField(port) {
  const fieldName = `PORT_${port.name}`;
  if (port.type === 'bool') {
    // Blockly checkbox 使用 'TRUE' / 'FALSE'
    const boolDefault = String(defaultPortValue(port)).toLowerCase() === 'true' ? 'TRUE' : 'FALSE';
    return { fieldName, field: new Blockly.FieldCheckbox(boolDefault), label: port.name };
  }
  return { fieldName, field: new Blockly.FieldTextInput(defaultPortValue(port)), label: port.name };
}

export function registerTreeNodesModelXml(xmlString, { generator, namespace = 'pal' } = {}) {
  if (!generator) throw new Error('registerTreeNodesModelXml: generator is required');
  if (!xmlString || !xmlString.trim()) return { added: 0, skipped: 0 };

  const parser = new DOMParser();
  const doc = parser.parseFromString(xmlString, 'text/xml');
  const parseError = doc.querySelector('parsererror');
  if (parseError) throw new Error('TreeNodesModel XML parse error');

  const model = doc.querySelector('TreeNodesModel');
  if (!model) throw new Error('Missing <TreeNodesModel> in palette XML');

  let added = 0;
  let skipped = 0;

  for (const nodeEl of Array.from(model.children)) {
    const kind = nodeEl.tagName;
    const id = nodeEl.getAttribute('ID');
    if (!id) continue;

    const blockType = `${namespace}_${sanitizeId(id)}`;
    if (Blockly.Blocks[blockType] || state.blockTypeToDef.has(blockType)) {
      skipped++;
      continue;
    }

    const ports = parsePorts(nodeEl);
    const def = { id, kind, ports, blockType };

    Blockly.Blocks[blockType] = {
      init: function () {
        this.appendDummyInput()
          .appendField(id)
          .appendField(new Blockly.FieldTextInput(''), 'NAME');

        // ports
        for (const port of ports) {
          const { fieldName, field, label } = buildPortField(port);
          this.appendDummyInput()
            .appendField(label)
            .appendField(field, fieldName);
        }

        // children slot for control/decorator
        if (kind === 'Control') {
          this.appendStatementInput('CHILDREN')
            .setCheck(null)
            .appendField('children');
        } else if (kind === 'Decorator') {
          this.appendStatementInput('CHILD')
            .setCheck(null)
            .appendField('child');
        }

        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(kindToColour(kind));
        this.setTooltip(`${kind} node: ${id}`);
      }
    };

    generator.forBlock[blockType] = function (block, gen) {
      const name = (block.getFieldValue('NAME') || '').trim();
      const attrs = [];
      if (name) attrs.push(`name="${name}"`);

      for (const port of ports) {
        const fieldName = `PORT_${port.name}`;
        const field = block.getField(fieldName);
        if (!field) continue;
        let value = block.getFieldValue(fieldName);
        if (value == null) continue;
        value = String(value).trim();
        if (!value) continue;

        // checkbox -> true/false
        if (port.type === 'bool') {
          value = value === 'TRUE' ? 'true' : 'false';
        }
        attrs.push(`${port.name}="${value}"`);
      }

      const attrStr = attrs.length ? ` ${attrs.join(' ')}` : '';

      const isContainer = kind === 'Control' || kind === 'Decorator';
      const childInput = kind === 'Control' ? 'CHILDREN' : (kind === 'Decorator' ? 'CHILD' : null);
      let children = '';
      if (isContainer && childInput) {
        let childBlock = block.getInputTargetBlock(childInput);
        while (childBlock) {
          children += gen.blockToCode(childBlock);
          childBlock = childBlock.getNextBlock();
        }
      }

      if (!isContainer || !children.trim()) {
        return `    <${id}${attrStr}/>\n`;
      }

      return `    <${id}${attrStr}>\n${children}    </${id}>\n`;
    };

    state.idToBlockType.set(id, blockType);
    state.blockTypeToDef.set(blockType, def);
    if (!state.kinds.has(kind)) state.kinds.set(kind, []);
    state.kinds.get(kind).push({ id, blockType });

    added++;
  }

  // 分类内按字母排序，稳定 toolbox 顺序
  for (const [kind, items] of state.kinds.entries()) {
    items.sort((a, b) => a.id.localeCompare(b.id));
    state.kinds.set(kind, items);
  }

  return { added, skipped };
}

export function getPaletteBlockTypeById(nodeId) {
  return state.idToBlockType.get(nodeId) || null;
}

export function isPaletteBlockType(blockType) {
  return state.blockTypeToDef.has(blockType);
}

export function buildPaletteToolboxCategory({ name = 'Node Palette', colour = '#64748b' } = {}) {
  const kindOrder = ['Control', 'Decorator', 'Action', 'Condition'];
  const contents = [];

  for (const kind of kindOrder) {
    const items = state.kinds.get(kind);
    if (!items?.length) continue;
    contents.push({
      kind: 'category',
      name: kind,
      colour,
      contents: items.map(it => ({ kind: 'block', type: it.blockType }))
    });
  }

  if (!contents.length) return null;

  return {
    kind: 'category',
    name,
    colour,
    contents
  };
}

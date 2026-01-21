/**
 * BT Visual Editor - 主入口文件
 */

import * as Blockly from 'blockly';

// 导入积木块定义
import './blockly/blocks/control.js';
import './blockly/blocks/decorator.js';
import './blockly/blocks/action.js';
import './blockly/blocks/condition.js';
import './blockly/blocks/unknown.js';

// 导入工具箱配置
import { toolboxConfig } from './blockly/toolbox.js';

// 导入代码生成器
import { btXmlGenerator } from './blockly/generators/bt_xml.js';

// 导入节点描述（TreeNodesModel）加载器：实现“加节点描述就自动出现积木”
import {
    buildPaletteToolboxCategory,
    getPaletteBlockTypeById,
    isPaletteBlockType,
    registerTreeNodesModelXml
} from './blockly/palette_registry.js';

// 内置节点库（最小子集），开箱即用；如需完整节点请用“导入节点库”
import nav2TreeNodesXml from './assets/nav2_tree_nodes_min.xml?raw';

// 内置 BT 模板
import { BT_TEMPLATES } from './templates/bt_templates.js';

// 导入可视化组件
import { TreeVisualizer } from './visualization/tree_graph.js';

// 导入验证工具
import { validateBehaviorTree, generateValidationReportHtml } from './utils/validator.js';

// ========================================
// 应用状态
// ========================================
const appState = {
    workspace: null,
    visualizer: null,
    currentFileName: '未命名.xml',
    isModified: false,
    lastSavedTime: null,
    savesDirectory: '/saves/',
    bt: {
        format: '4',
        mainTreeToExecute: 'MainTree',
        treeId: 'MainTree'
    },
    palette: {
        loadedCount: 0
    }
};

const PANEL_SIZE_STORAGE_KEY = 'bt_visual_editor.panel_sizes.v1';
const AI_BACKEND_URL_STORAGE_KEY = 'bt_visual_editor.ai_backend_url.v1';
const DEFAULT_AI_BACKEND_URL = 'http://127.0.0.1:8787/generate';
const CUSTOM_PALETTES_STORAGE_KEY = 'bt_visual_editor.custom_palettes.v1';

// ========================================
// 初始化 Blockly
// ========================================
function initBlockly() {
    const container = document.getElementById('blockly-container');
    if (!container) {
        console.error('Blockly container not found');
        return;
    }

    // Blockly 配置
    const options = {
        toolbox: toolboxConfig,
        grid: {
            spacing: 20,
            length: 3,
            colour: 'rgba(255, 255, 255, 0.1)',
            snap: true
        },
        zoom: {
            controls: true,
            wheel: true,
            startScale: 0.9,
            maxScale: 2,
            minScale: 0.3,
            scaleSpeed: 1.2
        },
        trashcan: true,
        move: {
            scrollbars: {
                horizontal: true,
                vertical: true
            },
            drag: true,
            wheel: true
        },
        theme: createCustomTheme(),
        renderer: 'zelos'
    };

    // 注入 Blockly
    appState.workspace = Blockly.inject(container, options);

    // 监听变化
    appState.workspace.addChangeListener((event) => {
        if (event.type === Blockly.Events.BLOCK_CHANGE ||
            event.type === Blockly.Events.BLOCK_CREATE ||
            event.type === Blockly.Events.BLOCK_DELETE ||
            event.type === Blockly.Events.BLOCK_MOVE) {
            updateXmlOutput();
            appState.isModified = true;
            updateStatusBar();
        }
    });

    // 窗口大小调整
    window.addEventListener('resize', () => {
        Blockly.svgResize(appState.workspace);
        if (appState.visualizer) {
            appState.visualizer.resize();
        }
    });
}

function setPanelFlexBasis(panel, widthPx) {
    if (!panel || !Number.isFinite(widthPx)) return;
    // 允许在窗口变大时自适应拉伸，避免出现“右侧空一块”的未占用区域
    panel.style.flex = `1 1 ${Math.round(widthPx)}px`;
}

function readPanelMinWidth(panel) {
    if (!panel) return 200;
    const cssMin = getComputedStyle(panel).minWidth;
    const parsed = Number.parseFloat(cssMin);
    return Number.isFinite(parsed) ? parsed : 200;
}

function savePanelSizes() {
    const blocklyPanel = document.getElementById('blockly-panel');
    const xmlPanel = document.getElementById('xml-panel');
    const graphPanel = document.getElementById('graph-panel');
    if (!blocklyPanel || !xmlPanel || !graphPanel) return;

    const sizes = {
        blockly: blocklyPanel.getBoundingClientRect().width,
        xml: xmlPanel.getBoundingClientRect().width,
        graph: graphPanel.getBoundingClientRect().width
    };
    try {
        localStorage.setItem(PANEL_SIZE_STORAGE_KEY, JSON.stringify(sizes));
    } catch {
        // ignore
    }
}

function loadPanelSizes() {
    try {
        const raw = localStorage.getItem(PANEL_SIZE_STORAGE_KEY);
        if (!raw) return null;
        const sizes = JSON.parse(raw);
        if (!sizes) return null;
        const blockly = Number(sizes.blockly);
        const xml = Number(sizes.xml);
        const graph = Number(sizes.graph);
        if (![blockly, xml, graph].every(Number.isFinite)) return null;
        return { blockly, xml, graph };
    } catch {
        return null;
    }
}

function applyPanelSizes(sizes) {
    const blocklyPanel = document.getElementById('blockly-panel');
    const xmlPanel = document.getElementById('xml-panel');
    const graphPanel = document.getElementById('graph-panel');
    if (!blocklyPanel || !xmlPanel || !graphPanel) return;

    setPanelFlexBasis(blocklyPanel, sizes.blockly);
    setPanelFlexBasis(xmlPanel, sizes.xml);
    setPanelFlexBasis(graphPanel, sizes.graph);

    Blockly.svgResize(appState.workspace);
    appState.visualizer?.resize?.();
}

function lockPanelSizesFromCurrentLayout() {
    const blocklyPanel = document.getElementById('blockly-panel');
    const xmlPanel = document.getElementById('xml-panel');
    const graphPanel = document.getElementById('graph-panel');
    if (!blocklyPanel || !xmlPanel || !graphPanel) return;

    const sizes = {
        blockly: blocklyPanel.getBoundingClientRect().width,
        xml: xmlPanel.getBoundingClientRect().width,
        graph: graphPanel.getBoundingClientRect().width
    };
    applyPanelSizes(sizes);
}

function initPanelResizers() {
    const resizers = Array.from(document.querySelectorAll('.panel-resizer'));
    if (!resizers.length) return;

    const saved = loadPanelSizes();
    // 先应用保存的尺寸；否则将初始布局锁定为 px，保证拖拽行为可预期
    requestAnimationFrame(() => {
        if (saved) {
            applyPanelSizes(saved);
        } else {
            lockPanelSizesFromCurrentLayout();
        }
    });

    const startDrag = (event, resizer, leftPanel, rightPanel) => {
        if (!leftPanel || !rightPanel) return;
        event.preventDefault();

        resizer.classList.add('dragging');
        resizer.setPointerCapture?.(event.pointerId);

        const startX = event.clientX;
        const startLeft = leftPanel.getBoundingClientRect().width;
        const startRight = rightPanel.getBoundingClientRect().width;
        const minLeft = readPanelMinWidth(leftPanel);
        const minRight = readPanelMinWidth(rightPanel);

        const onMove = (e) => {
            const dx = e.clientX - startX;
            let newLeft = startLeft + dx;
            let newRight = startRight - dx;

            // clamp
            if (newLeft < minLeft) {
                const diff = minLeft - newLeft;
                newLeft = minLeft;
                newRight = Math.max(minRight, newRight - diff);
            } else if (newRight < minRight) {
                const diff = minRight - newRight;
                newRight = minRight;
                newLeft = Math.max(minLeft, newLeft - diff);
            }

            setPanelFlexBasis(leftPanel, newLeft);
            setPanelFlexBasis(rightPanel, newRight);

            Blockly.svgResize(appState.workspace);
            appState.visualizer?.resize?.();
        };

        const onUp = () => {
            resizer.classList.remove('dragging');
            window.removeEventListener('pointermove', onMove);
            window.removeEventListener('pointerup', onUp);
            savePanelSizes();
        };

        window.addEventListener('pointermove', onMove);
        window.addEventListener('pointerup', onUp, { once: true });
    };

    for (const resizer of resizers) {
        const leftSelector = resizer.getAttribute('data-left');
        const rightSelector = resizer.getAttribute('data-right');
        const leftPanel = leftSelector ? document.querySelector(leftSelector) : null;
        const rightPanel = rightSelector ? document.querySelector(rightSelector) : null;

        resizer.addEventListener('pointerdown', (e) => startDrag(e, resizer, leftPanel, rightPanel));
    }
}

function updateToolbox() {
    if (!appState.workspace) return;
    const cfg = JSON.parse(JSON.stringify(toolboxConfig));
    const paletteCategory = buildPaletteToolboxCategory({ name: 'Node Palette 节点库', colour: '#64748b' });
    if (paletteCategory) {
        cfg.contents.push({ kind: 'sep' }, paletteCategory);
    }
    appState.workspace.updateToolbox(cfg);
}

// ========================================
// 创建自定义主题
// ========================================
function createCustomTheme() {
    return Blockly.Theme.defineTheme('btEditor', {
        base: Blockly.Themes.Classic,
        blockStyles: {
            control_blocks: { colourPrimary: '#3498db' },
            decorator_blocks: { colourPrimary: '#1abc9c' },
            action_blocks: { colourPrimary: '#e67e22' },
            condition_blocks: { colourPrimary: '#9b59b6' }
        },
        categoryStyles: {
            control_category: { colour: '#3498db' },
            decorator_category: { colour: '#1abc9c' },
            action_category: { colour: '#e67e22' },
            condition_category: { colour: '#9b59b6' }
        },
        componentStyles: {
            workspaceBackgroundColour: '#1e2746',
            toolboxBackgroundColour: '#16213e',
            toolboxForegroundColour: '#e8e8e8',
            flyoutBackgroundColour: '#0f3460',
            flyoutForegroundColour: '#e8e8e8',
            flyoutOpacity: 0.98,
            scrollbarColour: '#4a5568',
            scrollbarOpacity: 0.6
        },
        fontStyle: {
            family: 'Inter, system-ui, sans-serif',
            weight: 'normal',
            size: 11
        }
    });
}

// ========================================
// 初始化可视化
// ========================================
function initVisualizer() {
    const container = document.getElementById('graph-container');
    if (!container) {
        console.error('Graph container not found');
        return;
    }

    // 清空容器
    container.innerHTML = '';

    appState.visualizer = new TreeVisualizer('graph-container');
}

// ========================================
// 更新 XML 输出
// ========================================
function updateXmlOutput() {
    if (!appState.workspace) return;

    // 同步 XML 根配置（用于导入后保持 main_tree_to_execute / BehaviorTree ID）
    btXmlGenerator.setConfig?.(appState.bt);

    const xmlCode = btXmlGenerator.workspaceToCode(appState.workspace);
    const xmlOutput = document.querySelector('#xml-output code');

    if (xmlOutput) {
        // 直接显示纯文本XML，不使用innerHTML避免渲染问题
        xmlOutput.textContent = xmlCode;
    }

    // 更新可视化
    if (appState.visualizer) {
        appState.visualizer.renderFromXml(xmlCode);
    }

    // 更新节点计数
    updateNodeCount();
}

function collectExportedBlockIds(workspace) {
    const exported = new Set();

    const visitChildren = (block) => {
        if (!block) return;

        const childrenInput = block.getInput('CHILDREN');
        if (childrenInput) {
            const child = childrenInput.connection?.targetBlock();
            if (child) visitChain(child);
            return;
        }

        const childInput = block.getInput('CHILD');
        if (childInput) {
            // Decorator 语义：只允许一个 child，多余的 next sibling 不属于子树
            const child = childInput.connection?.targetBlock();
            if (child) {
                exported.add(child.id);
                visitChildren(child);
            }
        }
    };

    const visitChain = (block) => {
        let cur = block;
        while (cur) {
            if (exported.has(cur.id)) break;
            exported.add(cur.id);
            visitChildren(cur);
            cur = cur.getNextBlock();
        }
    };

    const topBlocks = workspace.getTopBlocks(false) || [];
    for (const top of topBlocks) {
        visitChain(top);
    }

    return exported;
}

function countDecoratorMultiChildIssues(workspace) {
    const decorators = workspace.getAllBlocks(false).filter(b => b.getInput('CHILD'));
    let count = 0;
    for (const deco of decorators) {
        const child = deco.getInput('CHILD')?.connection?.targetBlock();
        if (child && child.getNextBlock()) count++;
    }
    return count;
}

// ========================================
// 更新节点计数
// ========================================
function updateNodeCount() {
    const nodeCountEl = document.getElementById('node-count');
    if (!nodeCountEl || !appState.workspace) return;

    const blocks = appState.workspace.getAllBlocks(false);
    const exportedIds = collectExportedBlockIds(appState.workspace);
    const unexported = Math.max(0, blocks.length - exportedIds.size);
    const decoMultiChild = countDecoratorMultiChildIssues(appState.workspace);

    let text = `节点: ${blocks.length} | 导出: ${exportedIds.size}`;
    if (unexported > 0) text += ` | 未导出: ${unexported}`;
    if (decoMultiChild > 0) text += ` | ⚠ 装饰节点多 child: ${decoMultiChild}`;
    nodeCountEl.textContent = text;
}

// ========================================
// 更新状态栏
// ========================================
function updateStatusBar() {
    const lastSavedEl = document.getElementById('last-saved');
    const fileNameEl = document.getElementById('file-name');
    const validationEl = document.getElementById('validation-status');

    if (fileNameEl) {
        fileNameEl.textContent = appState.currentFileName + (appState.isModified ? ' *' : '');
    }

    if (lastSavedEl) {
        if (appState.lastSavedTime) {
            const time = appState.lastSavedTime.toLocaleTimeString('zh-CN');
            lastSavedEl.textContent = `上次保存: ${time}`;
        } else {
            lastSavedEl.textContent = '未保存';
        }
    }

    if (validationEl) {
        const blocks = appState.workspace ? appState.workspace.getAllBlocks(false) : [];
        if (blocks.length === 0) {
            validationEl.textContent = '○ 空';
            validationEl.className = '';
            return;
        }

        const result = validateBehaviorTree(appState.workspace);
        const errors = result.issues.filter(i => i.level === 'error').length;
        const warnings = result.issues.filter(i => i.level === 'warning').length;

        if (errors > 0) {
            validationEl.textContent = `✗ 错误: ${errors}`;
            validationEl.className = 'invalid';
        } else if (warnings > 0) {
            validationEl.textContent = `⚠ 警告: ${warnings}`;
            validationEl.className = 'warning';
        } else {
            validationEl.textContent = '✓ 通过';
            validationEl.className = 'valid';
        }
    }
}

// ========================================
// 工具栏功能
// ========================================

// 新建
function handleNew() {
    if (appState.isModified) {
        if (!confirm('当前项目未保存，确定要新建吗？')) {
            return;
        }
    }

    appState.workspace.clear();
    appState.currentFileName = '未命名.xml';
    appState.isModified = false;
    appState.lastSavedTime = null;
    updateStatusBar();
    updateXmlOutput();
}

// 保存项目（JSON格式，包含Blockly工作区状态）
function handleSave() {
    if (!appState.workspace) return;

    const state = Blockly.serialization.workspaces.save(appState.workspace);
    const json = JSON.stringify(state, null, 2);

    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);

    const fileName = appState.currentFileName.replace('.xml', '.json');

    const a = document.createElement('a');
    a.href = url;
    a.download = fileName;
    a.click();

    URL.revokeObjectURL(url);

    appState.isModified = false;
    appState.lastSavedTime = new Date();
    updateStatusBar();

    showNotification('项目已保存');
}

// 打开项目
function handleOpen() {
    const input = document.getElementById('file-input-project');
    if (!input) return;

    input.onchange = (event) => {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const state = JSON.parse(e.target.result);
                appState.workspace.clear();
                Blockly.serialization.workspaces.load(state, appState.workspace);

                appState.currentFileName = file.name.replace('.json', '.xml');
                appState.isModified = false;
                updateStatusBar();
                updateXmlOutput();

                showNotification('项目已加载');
            } catch (error) {
                console.error('Error loading project:', error);
                showNotification('加载失败：无效的项目文件', 'error');
            }
        };
        reader.readAsText(file);

        // 重置input以便重复选择同一文件
        input.value = '';
    };

    input.click();
}

// 导出 XML
function handleExportXml() {
    if (!appState.workspace) return;

    const xmlCode = btXmlGenerator.workspaceToCode(appState.workspace);

    const blob = new Blob([xmlCode], { type: 'application/xml' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = appState.currentFileName;
    a.click();

    URL.revokeObjectURL(url);

    showNotification('XML已导出');
}

// 导入 XML
function handleImportXml() {
    const input = document.getElementById('file-input-xml');
    if (!input) return;

    input.onchange = (event) => {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const xmlContent = e.target.result;

                // 尝试导回 Blockly 工作区（可编辑）
                const imported = importBehaviorTreeXmlToBlockly(xmlContent);

                // 更新文件名
                appState.currentFileName = file.name;
                updateStatusBar();

                if (imported) {
                    appState.isModified = false;
                    appState.lastSavedTime = null;
                    updateStatusBar();
                    updateXmlOutput();
                    showNotification('XML已导入并转换为可编辑积木块');
                } else {
                    // 降级：仅可视化显示
                    if (appState.visualizer) {
                        appState.visualizer.renderFromXml(xmlContent);
                    }
                    const xmlOutput = document.querySelector('#xml-output code');
                    if (xmlOutput) {
                        xmlOutput.textContent = xmlContent;
                    }
                    showNotification('XML已导入（仅可视化，当前内容无法完整转换为积木块）', 'error');
                }
            } catch (error) {
                console.error('Error importing XML:', error);
                showNotification('导入失败：无效的XML文件', 'error');
            }
        };
        reader.readAsText(file);

        input.value = '';
    };

    input.click();
}

// 复制 XML
function handleCopyXml() {
    if (!appState.workspace) return;

    const xmlCode = btXmlGenerator.workspaceToCode(appState.workspace);

    navigator.clipboard.writeText(xmlCode).then(() => {
        showNotification('XML已复制到剪贴板');
    }).catch(err => {
        console.error('Failed to copy:', err);
        showNotification('复制失败', 'error');
    });
}

// 显示帮助
function handleHelp() {
    showModal('使用帮助', `
    <h4>快速开始</h4>
    <ol>
      <li>从左侧工具箱拖拽积木块到编辑区</li>
      <li>连接积木块构建行为树</li>
      <li>中间面板实时显示生成的XML</li>
      <li>右侧面板显示行为树图形</li>
    </ol>

    <h4>节点库（TreeNodesModel）</h4>
    <ul>
      <li>点击“导入节点库”导入 Groot/BT.CPP 的 <code>TreeNodesModel</code> XML 后，会自动生成可拖拽的积木块</li>
      <li>导入完成后可选择“保存到本地”，下次打开页面会自动加载并出现在 <b>Node Palette</b> 中</li>
      <li>需要管理/导出/删除已保存节点库：点击“节点库管理”（本地浏览器存储）</li>
    </ul>
    
    <h4>快捷键</h4>
    <ul>
      <li><kbd>Ctrl+S</kbd> - 保存项目</li>
      <li><kbd>Ctrl+Z</kbd> - 撤销</li>
      <li><kbd>Ctrl+Y</kbd> - 重做</li>
      <li><kbd>Delete</kbd> - 删除选中积木块</li>
    </ul>
    
    <h4>节点类型</h4>
    <ul>
      <li><b>🔵 控制节点</b> - Sequence, Fallback, Recovery等</li>
      <li><b>🟢 装饰节点</b> - RateController, Repeat等</li>
      <li><b>🟠 动作节点</b> - NavigateToPose, FollowPath等</li>
      <li><b>🟣 条件节点</b> - GoalReached, IsBatteryLow等</li>
    </ul>
    
    <h4>XML导入方式</h4>
    <ul>
      <li>点击"导入XML"按钮选择文件</li>
      <li>直接将XML文件拖拽到浏览器窗口</li>
    </ul>

    <h4>诊断与警告</h4>
    <ul>
      <li>左下角“节点/导出/未导出/警告”可点击，打开详情并可定位到问题积木块</li>
      <li>装饰节点（Decorator）只允许一个 child，多余连接会被标为“未导出”并提示修正</li>
    </ul>
  `);
}

function escapeHtml(s) {
    return String(s ?? '')
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#39;');
}

function loadCustomPalettes() {
    try {
        const raw = localStorage.getItem(CUSTOM_PALETTES_STORAGE_KEY);
        if (!raw) return [];
        const parsed = JSON.parse(raw);
        if (!Array.isArray(parsed)) return [];
        return parsed
            .filter(x => x && typeof x.xml === 'string' && typeof x.id === 'string')
            .map(x => ({
                id: String(x.id),
                name: String(x.name || x.id),
                xml: String(x.xml),
                enabled: x.enabled !== false,
                createdAt: Number(x.createdAt || Date.now())
            }));
    } catch {
        return [];
    }
}

function saveCustomPalettes(palettes) {
    localStorage.setItem(CUSTOM_PALETTES_STORAGE_KEY, JSON.stringify(palettes));
}

function registerSavedPalettes() {
    const palettes = loadCustomPalettes().filter(p => p.enabled);
    if (!palettes.length) return { added: 0, skipped: 0, count: 0 };

    let added = 0;
    let skipped = 0;
    for (const p of palettes) {
        try {
            const ns = `saved_${p.id}`;
            const result = registerTreeNodesModelXml(p.xml, { generator: btXmlGenerator, namespace: ns });
            added += result.added;
            skipped += result.skipped;
        } catch (err) {
            console.warn('Saved palette load failed:', p.name, err);
        }
    }
    return { added, skipped, count: palettes.length };
}

function downloadTextFile(fileName, text, mime = 'text/plain') {
    const blob = new Blob([text], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = fileName;
    a.click();
    URL.revokeObjectURL(url);
}

function loadTemplate(template) {
    if (!template?.xml) return;
    if (appState.isModified) {
        if (!confirm('当前项目未保存，确定要加载模板并覆盖当前内容吗？')) return;
    }

    const imported = importBehaviorTreeXmlToBlockly(template.xml);
    if (!imported) {
        showNotification('模板加载失败：模板 XML 无法转换为积木块', 'error');
        return;
    }

    appState.currentFileName = template.fileName || `${template.id}.xml`;
    appState.isModified = false;
    appState.lastSavedTime = null;
    updateStatusBar();
    updateXmlOutput();
    showNotification(`已加载模板：${template.name}`);
}

function handleTemplates() {
    const itemsHtml = BT_TEMPLATES.map(t => {
        return `
          <div class="template-item" style="padding:10px;border:1px solid rgba(148,163,184,0.15);border-radius:8px;margin:8px 0;">
            <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;">
              <div style="min-width:0;">
                <div style="font-weight:600;color:#e2e8f0;">${escapeHtml(t.name)}</div>
                <div style="color:#94a3b8;font-size:12px;margin-top:4px;line-height:1.5;">${escapeHtml(t.description)}</div>
              </div>
              <button class="toolbar-btn" data-template-id="${escapeHtml(t.id)}" style="white-space:nowrap;">加载</button>
            </div>
          </div>
        `;
    }).join('');

    showModal('内置模板', `
      <div style="color:#94a3b8;font-size:12px;line-height:1.6;">
        这些模板与当前编辑器内置节点/生成器兼容，可直接加载后修改。
        <br/>提示：巡检模板里的 <code>{goals}</code> 需要运行时由任务节点或上游 BT 写入黑板。
      </div>
      <div style="margin-top:10px;">${itemsHtml}</div>
    `);

    // 绑定“加载”按钮
    const container = document.getElementById('modal-content');
    container?.querySelectorAll('[data-template-id]')?.forEach(btn => {
        btn.addEventListener('click', () => {
            const id = btn.getAttribute('data-template-id');
            const tpl = BT_TEMPLATES.find(x => x.id === id);
            if (!tpl) return;
            hideModal();
            loadTemplate(tpl);
        });
    });
}

function handleAiMission() {
    const storedUrl = (() => {
        try { return localStorage.getItem(AI_BACKEND_URL_STORAGE_KEY) || ''; } catch { return ''; }
    })();
    const defaultUrl = storedUrl || DEFAULT_AI_BACKEND_URL;

    const optionsHtml = BT_TEMPLATES.map(t =>
        `<option value="${escapeHtml(t.id)}">${escapeHtml(t.name)}</option>`
    ).join('');

    showModal('AI 任务生成（本地服务）', `
      <div style="color:#94a3b8;font-size:12px;line-height:1.6;">
        说明：此功能默认调用本地 HTTP 服务生成“任务计划 + BT XML + 巡检点”。不会在浏览器内直接访问外网。
        <br/>后端接口约定：<code>POST</code> <code>/generate</code>，JSON 输入 <code>{ prompt, template_id }</code>，输出 <code>{ mission, bt_xml, waypoints_yaml }</code>。
      </div>

      <div style="margin-top:10px;display:flex;gap:10px;align-items:center;flex-wrap:wrap;">
        <label style="font-size:12px;color:#cbd5e1;">后端URL</label>
        <input id="ai-backend-url" style="width:420px;max-width:100%;padding:6px 8px;border-radius:6px;border:1px solid rgba(148,163,184,0.2);background:#0b1220;color:#e2e8f0;"
               value="${escapeHtml(defaultUrl)}" />
        <label style="font-size:12px;color:#cbd5e1;">模板</label>
        <select id="ai-template-id" style="padding:6px 8px;border-radius:6px;border:1px solid rgba(148,163,184,0.2);background:#0b1220;color:#e2e8f0;">
          ${optionsHtml}
        </select>
        <button id="btn-ai-generate" class="toolbar-btn primary">生成</button>
      </div>

      <div style="margin-top:10px;">
        <textarea id="ai-prompt" rows="6" style="width:100%;padding:10px;border-radius:8px;border:1px solid rgba(148,163,184,0.2);background:#0b1220;color:#e2e8f0;resize:vertical;"
          placeholder="例：在 map 坐标系做巡检。点位：A(1.2,0.3,90deg)、B(2.1,1.0,0deg)、C(0.5,2.0,-90deg)。循环 3 次；到点停留 10s 并触发拍照 action；失败重试 2 次，仍失败则跳过并记录。"></textarea>
      </div>

      <div id="ai-result" style="margin-top:12px;display:none;">
        <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:8px;">
          <button id="btn-ai-apply-bt" class="toolbar-btn">应用 BT</button>
          <button id="btn-ai-download-mission" class="toolbar-btn">下载 mission.json</button>
          <button id="btn-ai-download-waypoints" class="toolbar-btn">下载 waypoints.yaml</button>
        </div>
        <div style="display:grid;grid-template-columns:1fr;gap:10px;">
          <div>
            <div style="color:#cbd5e1;font-size:12px;margin-bottom:6px;">Mission（任务计划）</div>
            <pre id="ai-mission-json" style="max-height:220px;overflow:auto;background:#0d1117;border:1px solid rgba(148,163,184,0.15);border-radius:8px;padding:10px;color:#7ee787;"></pre>
          </div>
          <div>
            <div style="color:#cbd5e1;font-size:12px;margin-bottom:6px;">BT XML</div>
            <pre id="ai-bt-xml" style="max-height:220px;overflow:auto;background:#0d1117;border:1px solid rgba(148,163,184,0.15);border-radius:8px;padding:10px;color:#7ee787;white-space:pre;"></pre>
          </div>
          <div>
            <div style="color:#cbd5e1;font-size:12px;margin-bottom:6px;">Waypoints YAML</div>
            <pre id="ai-waypoints-yaml" style="max-height:220px;overflow:auto;background:#0d1117;border:1px solid rgba(148,163,184,0.15);border-radius:8px;padding:10px;color:#7ee787;white-space:pre;"></pre>
          </div>
        </div>
      </div>
    `);

    const resultState = { mission: null, btXml: '', waypointsYaml: '' };

    const btnGen = document.getElementById('btn-ai-generate');
    btnGen?.addEventListener('click', async () => {
        const url = document.getElementById('ai-backend-url')?.value?.trim();
        const prompt = document.getElementById('ai-prompt')?.value?.trim();
        const templateId = document.getElementById('ai-template-id')?.value?.trim();

        if (!url) {
            showNotification('请输入后端URL', 'error');
            return;
        }
        if (!prompt) {
            showNotification('请输入任务描述', 'error');
            return;
        }

        try { localStorage.setItem(AI_BACKEND_URL_STORAGE_KEY, url); } catch { /* ignore */ }

        btnGen.disabled = true;
        btnGen.textContent = '生成中...';

        try {
            const resp = await fetch(url, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ prompt, template_id: templateId })
            });
            if (!resp.ok) {
                throw new Error(`HTTP ${resp.status}`);
            }
            const data = await resp.json();
            resultState.mission = data.mission ?? null;
            resultState.btXml = String(data.bt_xml ?? '');
            resultState.waypointsYaml = String(data.waypoints_yaml ?? '');

            const resultBox = document.getElementById('ai-result');
            const missionEl = document.getElementById('ai-mission-json');
            const btEl = document.getElementById('ai-bt-xml');
            const wpEl = document.getElementById('ai-waypoints-yaml');
            if (resultBox) resultBox.style.display = 'block';
            if (missionEl) missionEl.textContent = resultState.mission ? JSON.stringify(resultState.mission, null, 2) : '(empty)';
            if (btEl) btEl.textContent = resultState.btXml || '(empty)';
            if (wpEl) wpEl.textContent = resultState.waypointsYaml || '(empty)';

            showNotification('AI 任务生成完成');
        } catch (err) {
            console.error('AI generate failed:', err);
            showNotification('生成失败：请检查本地后端服务是否启动 / CORS 是否允许', 'error');
        } finally {
            btnGen.disabled = false;
            btnGen.textContent = '生成';
        }
    });

    document.getElementById('btn-ai-apply-bt')?.addEventListener('click', () => {
        if (!resultState.btXml?.trim()) {
            showNotification('没有可用的 BT XML', 'error');
            return;
        }
        hideModal();
        const imported = importBehaviorTreeXmlToBlockly(resultState.btXml);
        if (!imported) {
            showNotification('BT 应用失败：XML 无法完整转换为积木块', 'error');
            // 降级显示
            const xmlOutput = document.querySelector('#xml-output code');
            if (xmlOutput) xmlOutput.textContent = resultState.btXml;
            appState.visualizer?.renderFromXml?.(resultState.btXml);
            return;
        }
        appState.currentFileName = 'ai_generated.xml';
        appState.isModified = true;
        updateStatusBar();
        updateXmlOutput();
    });

    document.getElementById('btn-ai-download-mission')?.addEventListener('click', () => {
        downloadTextFile('mission.json', JSON.stringify(resultState.mission ?? {}, null, 2), 'application/json');
    });

    document.getElementById('btn-ai-download-waypoints')?.addEventListener('click', () => {
        downloadTextFile('waypoints.yaml', resultState.waypointsYaml || '', 'text/yaml');
    });
}

// ========================================
// 验证行为树
// ========================================
function handleValidate() {
    if (!appState.workspace) {
        showNotification('工作区未准备好', 'error');
        return;
    }

    const result = validateBehaviorTree(appState.workspace);
    const reportHtml = generateValidationReportHtml(result);

    showModal('行为树验证报告', reportHtml);
}

function buildUnexportedReasonMap(workspace) {
    const reasons = new Map(); // blockId -> reason string
    const decorators = workspace.getAllBlocks(false).filter(b => b.getInput('CHILD'));
    for (const deco of decorators) {
        const child = deco.getInput('CHILD')?.connection?.targetBlock();
        if (!child) continue;
        let hidden = child.getNextBlock();
        while (hidden) {
            reasons.set(hidden.id, `该节点被串在装饰节点 "${getBlockLabel(deco)}" 的 child 下面。装饰节点只允许一个 child，请用 Sequence/Fallback 等控制节点包裹多个节点。`);
            hidden = hidden.getNextBlock();
        }
    }
    return reasons;
}

function getBlockLabel(block) {
    if (!block) return '(unknown)';
    const name = (block.getFieldValue?.('NAME') || '').trim();
    return name ? `${name} (${block.type})` : block.type;
}

function handleStatusDiagnostics() {
    if (!appState.workspace) {
        showNotification('工作区未准备好', 'error');
        return;
    }

    const result = validateBehaviorTree(appState.workspace);
    const reportHtml = generateValidationReportHtml(result);

    const blocks = appState.workspace.getAllBlocks(false);
    const exportedIds = collectExportedBlockIds(appState.workspace);
    const unexportedBlocks = blocks.filter(b => !exportedIds.has(b.id));
    const decoMultiChild = countDecoratorMultiChildIssues(appState.workspace);
    const reasons = buildUnexportedReasonMap(appState.workspace);

    const statsHtml = `
      <div style="margin-top:14px;padding:10px 12px;border:1px solid rgba(148,163,184,0.15);border-radius:8px;">
        <div style="font-weight:600;margin-bottom:6px;">导出诊断</div>
        <div style="color:#94a3b8;font-size:12px;line-height:1.6;">
          节点总数: <b>${blocks.length}</b>，预计导出: <b>${exportedIds.size}</b>
          ${unexportedBlocks.length ? `，未导出: <b style="color:#fbbf24;">${unexportedBlocks.length}</b>` : ''}
          ${decoMultiChild ? `，装饰节点多 child: <b style="color:#fbbf24;">${decoMultiChild}</b>` : ''}
        </div>
        <div style="color:#94a3b8;font-size:12px;line-height:1.6;margin-top:6px;">
          提示：未导出的节点通常是“没有连接到根”或“被装饰节点 child 串了多个 sibling（只会导出第一个）”。
        </div>
      </div>
    `;

    let unexportedHtml = '';
    if (unexportedBlocks.length) {
        const items = unexportedBlocks.slice(0, 20).map(b => {
            const reason = reasons.get(b.id) || '该节点不在可导出子树中（可能未连接到根，或处于不被序列化的位置）。';
            return `
              <div style="display:flex;gap:10px;align-items:flex-start;justify-content:space-between;padding:8px 10px;border:1px solid rgba(148,163,184,0.12);border-radius:8px;">
                <div style="min-width:0;">
                  <div style="font-weight:500;">${escapeHtml(getBlockLabel(b))}</div>
                  <div style="color:#94a3b8;font-size:12px;line-height:1.5;margin-top:4px;">${escapeHtml(reason)}</div>
                </div>
                <button class="toolbar-btn" data-focus-block-id="${escapeHtml(b.id)}" style="white-space:nowrap;">定位</button>
              </div>
            `;
        }).join('');

        const more = unexportedBlocks.length > 20 ? `<div style="color:#94a3b8;font-size:12px;margin-top:6px;">仅显示前 20 个，剩余 ${unexportedBlocks.length - 20} 个未展开。</div>` : '';

        unexportedHtml = `
          <div style="margin-top:14px;">
            <div style="font-weight:600;margin-bottom:8px;">未导出节点列表</div>
            <div style="display:flex;flex-direction:column;gap:8px;">${items}</div>
            ${more}
          </div>
        `;
    }

    showModal('警告/错误详情', `${reportHtml}${statsHtml}${unexportedHtml}`);

    // 绑定“定位”按钮：选中并居中到对应 block
    const container = document.getElementById('modal-content');
    container?.querySelectorAll('[data-focus-block-id]')?.forEach(btn => {
        btn.addEventListener('click', () => {
            const id = btn.getAttribute('data-focus-block-id');
            if (!id) return;
            const block = appState.workspace.getBlockById(id);
            if (!block) return;
            block.select?.();
            appState.workspace.centerOnBlock?.(id);
        });
    });
}

// 导入节点描述（TreeNodesModel XML）
function handleImportPalette() {
    const input = document.getElementById('file-input-palette');
    if (!input) return;

    input.onchange = (event) => {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const xml = e.target.result;
                const id = String(Date.now());
                const namespace = `saved_${id}`;
                const result = registerTreeNodesModelXml(xml, { generator: btXmlGenerator, namespace });
                appState.palette.loadedCount += result.added;
                updateToolbox();

                const shouldSave = confirm('节点库已导入。是否保存到本地（下次启动自动加载）？');
                if (shouldSave) {
                    const name = prompt('给这个节点库起个名字（用于下次识别）', file.name || `palette_${id}`) || (file.name || `palette_${id}`);
                    const palettes = loadCustomPalettes();
                    const normalizedXml = String(xml || '').trim();
                    const exists = palettes.some(p => String(p.xml || '').trim() === normalizedXml);
                    if (!exists) {
                        palettes.push({ id, name, xml: normalizedXml, enabled: true, createdAt: Date.now() });
                        try {
                            saveCustomPalettes(palettes);
                            showNotification(`节点库已保存：${name}`);
                        } catch (err) {
                            console.error('Failed to save palette:', err);
                            showNotification('保存失败：浏览器存储空间不足（localStorage）', 'error');
                        }
                    } else {
                        const openMgr = confirm('该节点库已存在（内容相同），未重复保存。\n是否打开“节点库管理”查看/启用？');
                        if (openMgr) {
                            hideModal();
                            handleManagePalettes();
                        } else {
                            showNotification('该节点库已存在（内容相同），未重复保存', 'warning');
                        }
                    }
                } else {
                    showNotification(`节点库已导入（未保存）：新增 ${result.added}，跳过 ${result.skipped}`);
                }
            } catch (err) {
                console.error('Error importing palette:', err);
                showNotification('导入节点库失败：无效的 TreeNodesModel XML', 'error');
            }
        };
        reader.readAsText(file);
        input.value = '';
    };

    input.click();
}

function handleManagePalettes() {
    const palettes = loadCustomPalettes();

    const rows = palettes.length ? palettes.map(p => {
        const sizeKb = Math.max(1, Math.round((p.xml.length || 0) / 1024));
        const checked = p.enabled ? 'checked' : '';
        return `
          <div style="display:flex;gap:10px;align-items:flex-start;justify-content:space-between;padding:10px;border:1px solid rgba(148,163,184,0.12);border-radius:8px;margin:8px 0;">
            <div style="min-width:0;">
              <div style="font-weight:600;color:#e2e8f0;word-break:break-word;">${escapeHtml(p.name)}</div>
              <div style="color:#94a3b8;font-size:12px;margin-top:4px;">id=${escapeHtml(p.id)} · ${sizeKb}KB</div>
            </div>
            <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:flex-end;">
              <label style="display:flex;gap:6px;align-items:center;color:#cbd5e1;font-size:12px;">
                <input type="checkbox" data-palette-toggle="${escapeHtml(p.id)}" ${checked}/>
                启用
              </label>
              <button class="toolbar-btn" data-palette-export="${escapeHtml(p.id)}">导出</button>
              <button class="toolbar-btn" data-palette-delete="${escapeHtml(p.id)}" style="border-color: rgba(239,68,68,0.4); color:#fecaca;">删除</button>
            </div>
          </div>
        `;
    }).join('') : `<div style="color:#94a3b8;font-size:12px;line-height:1.6;">暂无已保存的节点库。你可以先点击“导入节点库”，导入后选择保存。</div>`;

    showModal('节点库管理（本地）', `
      <div style="color:#94a3b8;font-size:12px;line-height:1.6;">
        这些节点库保存在浏览器 localStorage 中。启用/删除后建议刷新页面让工具箱完全更新。
      </div>
      <div style="margin-top:10px;">${rows}</div>
      <div style="display:flex;gap:8px;justify-content:flex-end;margin-top:12px;">
        <button id="btn-palettes-refresh" class="toolbar-btn">刷新页面生效</button>
      </div>
    `);

    const container = document.getElementById('modal-content');

    container?.querySelectorAll('[data-palette-toggle]')?.forEach(el => {
        el.addEventListener('change', () => {
            const id = el.getAttribute('data-palette-toggle');
            if (!id) return;
            const updated = loadCustomPalettes().map(p => p.id === id ? { ...p, enabled: el.checked } : p);
            try {
                saveCustomPalettes(updated);
                showNotification('已保存设置：建议刷新页面使工具箱完全生效', 'info');
            } catch {
                showNotification('保存失败：浏览器存储空间不足（localStorage）', 'error');
            }
        });
    });

    container?.querySelectorAll('[data-palette-delete]')?.forEach(btn => {
        btn.addEventListener('click', () => {
            const id = btn.getAttribute('data-palette-delete');
            if (!id) return;
            if (!confirm('确定删除该节点库吗？（删除后不可恢复）')) return;
            const updated = loadCustomPalettes().filter(p => p.id !== id);
            try { saveCustomPalettes(updated); } catch { /* ignore */ }
            hideModal();
            handleManagePalettes();
        });
    });

    container?.querySelectorAll('[data-palette-export]')?.forEach(btn => {
        btn.addEventListener('click', () => {
            const id = btn.getAttribute('data-palette-export');
            if (!id) return;
            const p = loadCustomPalettes().find(x => x.id === id);
            if (!p) return;
            downloadTextFile(`${p.name || 'palette'}.xml`, p.xml, 'application/xml');
        });
    });

    document.getElementById('btn-palettes-refresh')?.addEventListener('click', () => {
        if (appState.isModified) {
            if (!confirm('当前项目未保存，刷新会丢失未保存修改，确定继续？')) return;
        }
        location.reload();
    });
}

// ========================================
// XML -> Blockly 导入
// ========================================

function importBehaviorTreeXmlToBlockly(xmlString) {
    if (!appState.workspace) {
        alert("错误: Blockly workspace 未初始化");
        return false;
    }
    if (!xmlString || !xmlString.trim()) {
        alert("错误: XML 内容为空");
        return false;
    }

    const parsed = parseBehaviorTreeXml(xmlString);
    if (!parsed?.rootNode) {
        alert("错误: 无法解析 XML 或找不到 root/BehaviorTree 节点");
        return false;
    }

    // 记录 root 配置，导出时保持一致
    appState.bt.format = parsed.format ?? appState.bt.format;
    appState.bt.mainTreeToExecute = parsed.mainTreeToExecute ?? appState.bt.mainTreeToExecute;
    appState.bt.treeId = parsed.treeId ?? appState.bt.treeId;
    btXmlGenerator.setConfig?.(appState.bt);

    appState.workspace.clear();

    const rootBlock = buildBlocksFromXmlElement(parsed.rootNode);
    if (!rootBlock) {
        alert("错误: 无法从 XML 构建积木块 (rootBlock 为空)");
        return false;
    }

    // 顶层放置
    rootBlock.moveBy(40, 40);

    // 触发布局/渲染
    appState.workspace.render?.();
    return true;
}

function parseBehaviorTreeXml(xmlString) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(xmlString, 'text/xml');

    const parseError = xmlDoc.querySelector('parsererror');
    if (parseError) {
        throw new Error('XML parse error');
    }

    const root = xmlDoc.querySelector('root');
    const format = root?.getAttribute('BTCPP_format') ?? root?.getAttribute('BTCPP_format');
    const mainTreeToExecute = root?.getAttribute('main_tree_to_execute') ?? null;

    const behaviorTrees = Array.from(xmlDoc.querySelectorAll('BehaviorTree'));
    if (behaviorTrees.length === 0) return null;

    let selected = behaviorTrees[0];
    if (mainTreeToExecute) {
        const match = behaviorTrees.find(bt => bt.getAttribute('ID') === mainTreeToExecute);
        if (match) selected = match;
    }

    const treeId = selected.getAttribute('ID') ?? (mainTreeToExecute || 'MainTree');
    const rootNode = selected.firstElementChild;
    return { format, mainTreeToExecute: mainTreeToExecute || treeId, treeId, rootNode };
}

function buildBlocksFromXmlElement(element) {
    if (!element) return null;

    const tag = element.tagName;
    const blockType = mapXmlTagToBlockType(tag);
    const block = appState.workspace.newBlock(blockType);

    // 填充字段（根据不同块类型映射 XML attributes）
    applyXmlAttributesToBlock(element, block, blockType);

    block.initSvg();
    block.render();

    // 递归导入子节点（control/decorator/unknown 才会有 children input）
    const childInputName = block.getInput('CHILDREN') ? 'CHILDREN' : (block.getInput('CHILD') ? 'CHILD' : null);
    if (childInputName) {
        const children = Array.from(element.children);
        connectChildrenStatement(block, childInputName, children);
    }

    return block;
}

function connectChildrenStatement(parentBlock, inputName, childElements) {
    if (!childElements?.length) return;
    const input = parentBlock.getInput(inputName);
    const parentConn = input?.connection;
    if (!parentConn) return;

    let previousChildBlock = null;
    for (const childEl of childElements) {
        const childBlock = buildBlocksFromXmlElement(childEl);
        if (!childBlock) continue;

        if (!previousChildBlock) {
            parentConn.connect(childBlock.previousConnection);
        } else if (previousChildBlock.nextConnection) {
            previousChildBlock.nextConnection.connect(childBlock.previousConnection);
        }
        previousChildBlock = childBlock;
    }
}

function mapXmlTagToBlockType(tagName) {
    const paletteType = getPaletteBlockTypeById(tagName);
    if (paletteType) return paletteType;

    const controlMap = {
        'Sequence': 'bt_sequence',
        'PipelineSequence': 'bt_pipeline_sequence',
        'Fallback': 'bt_fallback',
        'ReactiveFallback': 'bt_reactive_fallback',
        'ReactiveSequence': 'bt_reactive_sequence',
        'RecoveryNode': 'bt_recovery',
        'RoundRobin': 'bt_round_robin',
        'Parallel': 'bt_parallel'
    };

    const decoratorMap = {
        'RateController': 'bt_rate_controller',
        'DistanceController': 'bt_distance_controller',
        'SpeedController': 'bt_speed_controller',
        'GoalUpdater': 'bt_goal_updater',
        'SingleTrigger': 'bt_single_trigger',
        'Inverter': 'bt_inverter',
        'ForceSuccess': 'bt_force_success',
        'ForceFailure': 'bt_force_failure',
        'Repeat': 'bt_repeat',
        'Timeout': 'bt_timeout'
    };

    const leafMap = {
        'NavigateToPose': 'bt_navigate_to_pose',
        'NavigateThroughPoses': 'bt_navigate_through_poses',
        'ComputePathToPose': 'bt_compute_path_to_pose',
        'FollowPath': 'bt_follow_path',
        'ComputeRoute': 'bt_compute_route',
        'ComputeAndTrackRoute': 'bt_compute_and_track_route',
        'BackUp': 'bt_backup',
        'Spin': 'bt_spin',
        'Wait': 'bt_wait',
        'ClearEntireCostmap': 'bt_clear_costmap',
        'DockRobot': 'bt_dock_robot',
        'UndockRobot': 'bt_undock_robot',
        'SmoothPath': 'bt_smooth_path',
        'TruncatePath': 'bt_truncate_path',
        'ReinitializeGlobalLocalization': 'bt_reinitialize_localization',
        'ControllerSelector': 'bt_controller_selector',
        'PlannerSelector': 'bt_planner_selector',
        'GoalReached': 'bt_goal_reached',
        'GoalUpdated': 'bt_goal_updated',
        'IsBatteryLow': 'bt_is_battery_low',
        'IsBatteryCharging': 'bt_is_battery_charging',
        'IsPathValid': 'bt_is_path_valid',
        'IsStuck': 'bt_is_stuck',
        'IsStopped': 'bt_is_stopped',
        'InitialPoseReceived': 'bt_initial_pose_received',
        'TimeExpired': 'bt_time_expired',
        'DistanceTraveled': 'bt_distance_traveled',
        'TransformAvailable': 'bt_transform_available',
        'WouldAControllerRecoveryHelp': 'bt_would_controller_recovery_help',
        'WouldAPlannerRecoveryHelp': 'bt_would_planner_recovery_help',
        'AlwaysSuccess': 'bt_always_success'
    };

    return controlMap[tagName] || decoratorMap[tagName] || leafMap[tagName] || 'bt_unknown_node';
}

function applyXmlAttributesToBlock(element, block, blockType) {
    const getAttr = (name) => element.getAttribute(name);

    // Palette (TreeNodesModel) 自动生成的块：通用 attribute -> PORT_<name> 映射
    if (isPaletteBlockType(blockType)) {
        const name = getAttr('name');
        if (name != null && block.getField('NAME')) block.setFieldValue(String(name), 'NAME');

        for (const attr of element.attributes) {
            if (attr.name === 'name') continue;
            const fieldName = `PORT_${attr.name}`;
            if (!block.getField(fieldName)) continue;

            // checkbox 字段需要 TRUE/FALSE
            const raw = String(attr.value);
            const lowered = raw.toLowerCase();
            if (lowered === 'true' || lowered === 'false') {
                block.setFieldValue(lowered === 'true' ? 'TRUE' : 'FALSE', fieldName);
            } else {
                block.setFieldValue(raw, fieldName);
            }
        }
        return;
    }

    // Control nodes
    if (['bt_sequence', 'bt_pipeline_sequence', 'bt_fallback', 'bt_reactive_fallback', 'bt_reactive_sequence', 'bt_round_robin', 'bt_parallel'].includes(blockType)) {
        const name = getAttr('name') ?? '';
        if (block.getField('NAME')) block.setFieldValue(name, 'NAME');
    }
    if (blockType === 'bt_recovery') {
        const name = getAttr('name') ?? '';
        const retries = getAttr('number_of_retries');
        if (block.getField('NAME')) block.setFieldValue(name, 'NAME');
        if (retries != null && block.getField('RETRIES')) block.setFieldValue(String(retries), 'RETRIES');
    }
    if (blockType === 'bt_parallel') {
        const success = getAttr('success_threshold');
        const failure = getAttr('failure_threshold');
        if (success != null && block.getField('SUCCESS_THRESHOLD')) block.setFieldValue(String(success), 'SUCCESS_THRESHOLD');
        if (failure != null && block.getField('FAILURE_THRESHOLD')) block.setFieldValue(String(failure), 'FAILURE_THRESHOLD');
    }

    // Decorators
    if (blockType === 'bt_rate_controller') {
        const hz = getAttr('hz');
        if (hz != null) block.setFieldValue(String(hz), 'HZ');
    }
    if (blockType === 'bt_distance_controller') {
        const distance = getAttr('distance');
        const globalFrame = getAttr('global_frame');
        const robotBaseFrame = getAttr('robot_base_frame');
        if (distance != null) block.setFieldValue(String(distance), 'DISTANCE');
        if (globalFrame != null) block.setFieldValue(String(globalFrame), 'GLOBAL_FRAME');
        if (robotBaseFrame != null) block.setFieldValue(String(robotBaseFrame), 'ROBOT_FRAME');
    }
    if (blockType === 'bt_speed_controller') {
        const minRate = getAttr('min_rate');
        const maxRate = getAttr('max_rate');
        const minSpeed = getAttr('min_speed');
        const maxSpeed = getAttr('max_speed');
        if (minRate != null) block.setFieldValue(String(minRate), 'MIN_RATE');
        if (maxRate != null) block.setFieldValue(String(maxRate), 'MAX_RATE');
        if (minSpeed != null) block.setFieldValue(String(minSpeed), 'MIN_SPEED');
        if (maxSpeed != null) block.setFieldValue(String(maxSpeed), 'MAX_SPEED');
    }
    if (blockType === 'bt_goal_updater') {
        const inputGoal = getAttr('input_goal');
        const outputGoal = getAttr('output_goal');
        if (inputGoal != null) block.setFieldValue(String(inputGoal), 'INPUT_GOAL');
        if (outputGoal != null) block.setFieldValue(String(outputGoal), 'OUTPUT_GOAL');
    }
    if (blockType === 'bt_repeat') {
        const numCycles = getAttr('num_cycles');
        if (numCycles != null) block.setFieldValue(String(numCycles), 'NUM_CYCLES');
    }
    if (blockType === 'bt_timeout') {
        const msec = getAttr('msec');
        if (msec != null) {
            const sec = Number(msec) / 1000;
            if (!Number.isNaN(sec)) block.setFieldValue(String(sec), 'TIMEOUT_SEC');
        }
    }

    // Actions
    if (blockType === 'bt_navigate_to_pose') {
        const goal = getAttr('goal');
        const behaviorTree = getAttr('behavior_tree');
        const serverName = getAttr('server_name');
        if (goal != null) block.setFieldValue(String(goal), 'GOAL');
        if (behaviorTree != null) block.setFieldValue(String(behaviorTree), 'BEHAVIOR_TREE');
        if (serverName != null) block.setFieldValue(String(serverName), 'SERVER_NAME');
    }
    if (blockType === 'bt_navigate_through_poses') {
        const goals = getAttr('goals');
        const serverName = getAttr('server_name');
        if (goals != null) block.setFieldValue(String(goals), 'GOALS');
        if (serverName != null) block.setFieldValue(String(serverName), 'SERVER_NAME');
    }
    if (blockType === 'bt_compute_path_to_pose') {
        const goal = getAttr('goal');
        const path = getAttr('path');
        const plannerId = getAttr('planner_id');
        if (goal != null) block.setFieldValue(String(goal), 'GOAL');
        if (path != null) block.setFieldValue(String(path), 'PATH');
        if (plannerId != null) block.setFieldValue(String(plannerId), 'PLANNER_ID');
    }
    if (blockType === 'bt_follow_path') {
        const path = getAttr('path');
        const controllerId = getAttr('controller_id');
        const errorCodeId = getAttr('error_code_id');
        if (path != null) block.setFieldValue(String(path), 'PATH');
        if (controllerId != null) block.setFieldValue(String(controllerId), 'CONTROLLER_ID');
        if (errorCodeId != null && block.getField('ERROR_CODE_ID')) block.setFieldValue(String(errorCodeId), 'ERROR_CODE_ID');
    }
    if (blockType === 'bt_compute_route') {
        const startId = getAttr('start_id');
        const goalId = getAttr('goal_id');
        const route = getAttr('route');
        const path = getAttr('path');
        if (startId != null) block.setFieldValue(String(startId), 'START_ID');
        if (goalId != null) block.setFieldValue(String(goalId), 'GOAL_ID');
        if (route != null) block.setFieldValue(String(route), 'ROUTE');
        if (path != null) block.setFieldValue(String(path), 'PATH');
    }
    if (blockType === 'bt_compute_and_track_route') {
        const goal = getAttr('goal');
        const path = getAttr('path');
        const route = getAttr('route');
        const usePoses = getAttr('use_poses');
        const errorCodeId = getAttr('error_code_id');
        if (goal != null) block.setFieldValue(String(goal), 'GOAL');
        if (path != null) block.setFieldValue(String(path), 'PATH');
        if (route != null) block.setFieldValue(String(route), 'ROUTE');
        if (usePoses != null) block.setFieldValue(String(usePoses).toLowerCase() === 'true' ? 'TRUE' : 'FALSE', 'USE_POSES');
        if (errorCodeId != null) block.setFieldValue(String(errorCodeId), 'ERROR_CODE_ID');
    }
    if (blockType === 'bt_backup') {
        const dist = getAttr('backup_dist');
        const speed = getAttr('backup_speed');
        if (dist != null) block.setFieldValue(String(dist), 'BACKUP_DIST');
        if (speed != null) block.setFieldValue(String(speed), 'BACKUP_SPEED');
    }
    if (blockType === 'bt_spin') {
        const dist = getAttr('spin_dist');
        if (dist != null) block.setFieldValue(String(dist), 'SPIN_DIST');
    }
    if (blockType === 'bt_wait') {
        const dur = getAttr('wait_duration');
        if (dur != null) block.setFieldValue(String(dur), 'WAIT_DURATION');
    }
    if (blockType === 'bt_clear_costmap') {
        const serviceName = getAttr('service_name');
        if (serviceName != null) block.setFieldValue(String(serviceName), 'SERVICE_NAME');
    }
    if (blockType === 'bt_dock_robot') {
        const dockId = getAttr('dock_id');
        const nav = getAttr('navigate_to_staging_pose');
        if (dockId != null) block.setFieldValue(String(dockId), 'DOCK_ID');
        if (nav != null && block.getField('NAVIGATE_TO_STAGING')) {
            block.setFieldValue(String(nav).toLowerCase() === 'true' ? 'TRUE' : 'FALSE', 'NAVIGATE_TO_STAGING');
        }
    }
    if (blockType === 'bt_undock_robot') {
        const dockType = getAttr('dock_type');
        if (dockType != null) block.setFieldValue(String(dockType), 'DOCK_TYPE');
    }
    if (blockType === 'bt_smooth_path') {
        const unsmoothed = getAttr('unsmoothed_path');
        const smoothed = getAttr('smoothed_path');
        const smootherId = getAttr('smoother_id');
        if (unsmoothed != null) block.setFieldValue(String(unsmoothed), 'UNSMOOTHED_PATH');
        if (smoothed != null) block.setFieldValue(String(smoothed), 'SMOOTHED_PATH');
        if (smootherId != null) block.setFieldValue(String(smootherId), 'SMOOTHER_ID');
    }
    if (blockType === 'bt_truncate_path') {
        const inputPath = getAttr('input_path');
        const outputPath = getAttr('output_path');
        const distance = getAttr('distance');
        if (inputPath != null) block.setFieldValue(String(inputPath), 'INPUT_PATH');
        if (outputPath != null) block.setFieldValue(String(outputPath), 'OUTPUT_PATH');
        if (distance != null) block.setFieldValue(String(distance), 'DISTANCE');
    }
    if (blockType === 'bt_reinitialize_localization') {
        const serviceName = getAttr('service_name');
        if (serviceName != null) block.setFieldValue(String(serviceName), 'SERVICE_NAME');
    }
    if (blockType === 'bt_controller_selector') {
        const def = getAttr('default_controller');
        const sel = getAttr('selected_controller');
        const topic = getAttr('topic_name');
        if (def != null) block.setFieldValue(String(def), 'DEFAULT_CONTROLLER');
        if (sel != null) block.setFieldValue(String(sel), 'SELECTED_CONTROLLER');
        if (topic != null && block.getField('TOPIC_NAME')) block.setFieldValue(String(topic), 'TOPIC_NAME');
    }
    if (blockType === 'bt_planner_selector') {
        const def = getAttr('default_planner');
        const sel = getAttr('selected_planner');
        const topic = getAttr('topic_name');
        if (def != null) block.setFieldValue(String(def), 'DEFAULT_PLANNER');
        if (sel != null) block.setFieldValue(String(sel), 'SELECTED_PLANNER');
        if (topic != null && block.getField('TOPIC_NAME')) block.setFieldValue(String(topic), 'TOPIC_NAME');
    }

    // Conditions
    if (blockType === 'bt_goal_reached') {
        const goal = getAttr('goal');
        const frame = getAttr('robot_base_frame');
        if (goal != null) block.setFieldValue(String(goal), 'GOAL');
        if (frame != null) block.setFieldValue(String(frame), 'ROBOT_BASE_FRAME');
    }
    if (blockType === 'bt_is_battery_low') {
        const minBattery = getAttr('min_battery');
        const topic = getAttr('battery_topic');
        const isVoltage = getAttr('is_voltage');
        if (minBattery != null) block.setFieldValue(String(minBattery), 'MIN_BATTERY');
        if (topic != null) block.setFieldValue(String(topic), 'BATTERY_TOPIC');
        if (isVoltage != null) block.setFieldValue(String(isVoltage).toLowerCase() === 'true' ? 'TRUE' : 'FALSE', 'IS_VOLTAGE');
    }
    if (blockType === 'bt_is_battery_charging') {
        const topic = getAttr('battery_topic');
        if (topic != null) block.setFieldValue(String(topic), 'BATTERY_TOPIC');
    }
    if (blockType === 'bt_is_path_valid') {
        const path = getAttr('path');
        if (path != null) block.setFieldValue(String(path), 'PATH');
    }
    if (blockType === 'bt_is_stopped') {
        const vel = getAttr('velocity_threshold');
        const dur = getAttr('duration_stopped');
        if (vel != null) block.setFieldValue(String(vel), 'VELOCITY_THRESHOLD');
        if (dur != null) block.setFieldValue(String(dur), 'DURATION_STOPPED');
    }
    if (blockType === 'bt_time_expired') {
        const seconds = getAttr('seconds');
        if (seconds != null) block.setFieldValue(String(seconds), 'SECONDS');
    }
    if (blockType === 'bt_distance_traveled') {
        const distance = getAttr('distance');
        const globalFrame = getAttr('global_frame');
        const robotBaseFrame = getAttr('robot_base_frame');
        if (distance != null) block.setFieldValue(String(distance), 'DISTANCE');
        if (globalFrame != null) block.setFieldValue(String(globalFrame), 'GLOBAL_FRAME');
        if (robotBaseFrame != null) block.setFieldValue(String(robotBaseFrame), 'ROBOT_BASE_FRAME');
    }
    if (blockType === 'bt_transform_available') {
        const parent = getAttr('parent');
        const child = getAttr('child');
        if (parent != null) block.setFieldValue(String(parent), 'PARENT');
        if (child != null) block.setFieldValue(String(child), 'CHILD');
    }
    if (blockType === 'bt_would_controller_recovery_help' || blockType === 'bt_would_planner_recovery_help') {
        const code = getAttr('error_code');
        if (code != null) block.setFieldValue(String(code), 'ERROR_CODE');
    }

    // Unknown node fallback
    if (blockType === 'bt_unknown_node') {
        block.setFieldValue(element.tagName, 'TAG');
        const attrs = [];
        for (const attr of element.attributes) {
            attrs.push(`${attr.name}="${attr.value}"`);
        }
        block.setFieldValue(attrs.join(' '), 'ATTRS');
    }
}

// ========================================
// 通知和模态框
// ========================================

function showNotification(message, type = 'success') {
    // 简单的通知实现
    const notification = document.createElement('div');
    notification.className = `notification ${type}`;
    notification.textContent = message;
    notification.style.cssText = `
    position: fixed;
    top: 70px;
    right: 20px;
    padding: 12px 20px;
    background: ${type === 'error' ? '#e74c3c' : '#00d4ff'};
    color: ${type === 'error' ? 'white' : 'black'};
    border-radius: 8px;
    font-size: 14px;
    font-weight: 500;
    z-index: 1000;
    animation: fadeIn 0.3s ease;
    box-shadow: 0 4px 12px rgba(0,0,0,0.3);
  `;

    document.body.appendChild(notification);

    setTimeout(() => {
        notification.style.opacity = '0';
        notification.style.transition = 'opacity 0.3s ease';
        setTimeout(() => notification.remove(), 300);
    }, 3000);
}

function showModal(title, content) {
    const overlay = document.getElementById('modal-overlay');
    const modalTitle = document.getElementById('modal-title');
    const modalContent = document.getElementById('modal-content');

    if (overlay && modalTitle && modalContent) {
        modalTitle.textContent = title;
        modalContent.innerHTML = content;
        overlay.classList.remove('hidden');
    }
}

function hideModal() {
    const overlay = document.getElementById('modal-overlay');
    if (overlay) {
        overlay.classList.add('hidden');
    }
}

// ========================================
// 拖拽导入XML文件
// ========================================
function setupDragAndDrop() {
    const dropZone = document.body;

    // 阻止默认行为
    ['dragenter', 'dragover', 'dragleave', 'drop'].forEach(eventName => {
        dropZone.addEventListener(eventName, (e) => {
            e.preventDefault();
            e.stopPropagation();
        }, false);
    });

    // 拖拽进入/离开时的视觉反馈
    let dragCounter = 0;

    dropZone.addEventListener('dragenter', () => {
        dragCounter++;
        dropZone.classList.add('drag-over');
    });

    dropZone.addEventListener('dragleave', () => {
        dragCounter--;
        if (dragCounter === 0) {
            dropZone.classList.remove('drag-over');
        }
    });

    // 处理拖拽放置
    dropZone.addEventListener('drop', (e) => {
        console.log('Drop event detected');
        dragCounter = 0;
        dropZone.classList.remove('drag-over');

        const files = e.dataTransfer?.files;
        if (!files || files.length === 0) {
            alert('Drop detected but no files found!');
            return;
        }
        alert(`Drop detected: ${files[0].name}`);

        const file = files[0];
        if (!file.name.endsWith('.xml') && !file.name.endsWith('.json')) {
            showNotification('请拖拽XML或JSON文件', 'error');
            return;
        }

        const reader = new FileReader();
        reader.onload = (event) => {
            alert('File read successfully');
            try {
                const content = event.target.result;
                let success = false;

                if (file.name.endsWith('.json')) {
                    const state = JSON.parse(content);
                    appState.workspace.clear();
                    Blockly.serialization.workspaces.load(state, appState.workspace);
                    appState.currentFileName = file.name.replace('.json', '.xml');
                    showNotification('项目已加载 (JSON)');
                    success = true;
                } else {
                    // 默认 XML
                    const imported = importBehaviorTreeXmlToBlockly(content);
                    appState.currentFileName = file.name;
                    if (imported) {
                        showNotification('XML已导入');
                        success = imported;
                    } else {
                        // 降级：仅可视化
                        if (appState.visualizer) {
                            appState.visualizer.renderFromXml(content);
                        }
                        const xmlOutput = document.querySelector('#xml-output code');
                        if (xmlOutput) xmlOutput.textContent = content;
                        showNotification('XML已导入（仅可视化）', 'warning');
                        success = true; // 视为成功导入（虽然降级）
                    }
                }

                if (success) {
                    appState.isModified = false;
                    appState.lastSavedTime = null;
                    updateStatusBar();
                    updateXmlOutput();
                }
            } catch (error) {
                console.error('Error importing dropped file:', error);
                showNotification('导入失败：无效的文件', 'error');
            }
        };
        reader.readAsText(file);
    });
}

// ========================================
// 绑定事件
// ========================================
function bindEvents() {
    // 工具栏按钮
    document.getElementById('btn-new')?.addEventListener('click', handleNew);
    document.getElementById('btn-open')?.addEventListener('click', handleOpen);
    document.getElementById('btn-save')?.addEventListener('click', handleSave);
    document.getElementById('btn-export')?.addEventListener('click', handleExportXml);
    document.getElementById('btn-import')?.addEventListener('click', handleImportXml);
    document.getElementById('btn-import-palette')?.addEventListener('click', handleImportPalette);
    document.getElementById('btn-manage-palettes')?.addEventListener('click', handleManagePalettes);
    document.getElementById('btn-copy-xml')?.addEventListener('click', handleCopyXml);
    document.getElementById('btn-validate')?.addEventListener('click', handleValidate);
    document.getElementById('btn-help')?.addEventListener('click', handleHelp);
    document.getElementById('btn-templates')?.addEventListener('click', handleTemplates);
    document.getElementById('btn-ai-mission')?.addEventListener('click', handleAiMission);

    // 状态栏：点击查看警告/错误详情
    document.getElementById('validation-status')?.addEventListener('click', handleStatusDiagnostics);
    document.getElementById('node-count')?.addEventListener('click', handleStatusDiagnostics);

    // 拖拽导入XML文件
    setupDragAndDrop();

    // 缩放按钮
    document.getElementById('btn-zoom-in')?.addEventListener('click', () => {
        appState.visualizer?.zoomIn();
    });
    document.getElementById('btn-zoom-out')?.addEventListener('click', () => {
        appState.visualizer?.zoomOut();
    });
    document.getElementById('btn-zoom-fit')?.addEventListener('click', () => {
        appState.visualizer?.resetZoom();
    });

    // 模态框
    document.getElementById('modal-close')?.addEventListener('click', hideModal);
    document.getElementById('modal-cancel')?.addEventListener('click', hideModal);
    document.getElementById('modal-confirm')?.addEventListener('click', hideModal);
    document.getElementById('modal-overlay')?.addEventListener('click', (e) => {
        if (e.target.id === 'modal-overlay') {
            hideModal();
        }
    });

    // 键盘快捷键
    document.addEventListener('keydown', (e) => {
        if (e.ctrlKey || e.metaKey) {
            if (e.key === 's') {
                e.preventDefault();
                handleSave();
            }
        }
    });

    // 离开页面提醒
    window.addEventListener('beforeunload', (e) => {
        if (appState.isModified) {
            e.preventDefault();
            e.returnValue = '';
        }
    });

    setupDragAndDrop();
}




// ========================================
// 应用初始化
// ========================================
function init() {
    console.log('🌲 BT Visual Editor initializing...');

    initBlockly();
    // 内置加载 Nav2 TreeNodesModel（可选：后续也可导入自定义 XML 扩展节点库）
    try {
        const result = registerTreeNodesModelXml(nav2TreeNodesXml, { generator: btXmlGenerator, namespace: 'nav2' });
        appState.palette.loadedCount += result.added;
        updateToolbox();
    } catch (err) {
        console.warn('Nav2 palette load failed:', err);
    }

    // 自动加载用户已保存的节点库（localStorage）
    try {
        const saved = registerSavedPalettes();
        if (saved.count > 0) {
            appState.palette.loadedCount += saved.added;
            updateToolbox();
        }
    } catch (err) {
        console.warn('Saved palettes load failed:', err);
    }
    initVisualizer();
    initPanelResizers();
    bindEvents();
    updateStatusBar();

    console.log('✅ BT Visual Editor ready');
}

// DOM 加载完成后初始化
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}

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

// 内置 Nav2 节点描述（也可导入自定义 TreeNodesModel XML）
import nav2TreeNodesXml from '../../../src/nav2_main/nav2_behavior_tree/nav2_tree_nodes.xml?raw';

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

function updateToolbox() {
    if (!appState.workspace) return;
    const cfg = JSON.parse(JSON.stringify(toolboxConfig));
    const paletteCategory = buildPaletteToolboxCategory({ name: 'Node Palette', colour: '#64748b' });
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

// ========================================
// 更新节点计数
// ========================================
function updateNodeCount() {
    const nodeCountEl = document.getElementById('node-count');
    if (!nodeCountEl || !appState.workspace) return;

    const blocks = appState.workspace.getAllBlocks(false);
    nodeCountEl.textContent = `节点: ${blocks.length}`;
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
        if (blocks.length > 0) {
            validationEl.textContent = '✓ 有效';
            validationEl.className = 'valid';
        } else {
            validationEl.textContent = '○ 空';
            validationEl.className = '';
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
  `);
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
                const namespace = `custom${Date.now()}`;
                const result = registerTreeNodesModelXml(xml, { generator: btXmlGenerator, namespace });
                appState.palette.loadedCount += result.added;
                updateToolbox();
                showNotification(`节点库已导入：新增 ${result.added}，跳过 ${result.skipped}`);
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

// ========================================
// XML -> Blockly 导入
// ========================================

function importBehaviorTreeXmlToBlockly(xmlString) {
    if (!appState.workspace) return false;
    if (!xmlString || !xmlString.trim()) return false;

    const parsed = parseBehaviorTreeXml(xmlString);
    if (!parsed?.rootNode) return false;

    // 记录 root 配置，导出时保持一致
    appState.bt.format = parsed.format ?? appState.bt.format;
    appState.bt.mainTreeToExecute = parsed.mainTreeToExecute ?? appState.bt.mainTreeToExecute;
    appState.bt.treeId = parsed.treeId ?? appState.bt.treeId;
    btXmlGenerator.setConfig?.(appState.bt);

    appState.workspace.clear();

    const rootBlock = buildBlocksFromXmlElement(parsed.rootNode);
    if (!rootBlock) return false;

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
        'WouldAPlannerRecoveryHelp': 'bt_would_planner_recovery_help'
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
        dragCounter = 0;
        dropZone.classList.remove('drag-over');

        const files = e.dataTransfer?.files;
        if (!files || files.length === 0) return;

        const file = files[0];
        if (!file.name.endsWith('.xml')) {
            showNotification('请拖拽XML文件', 'error');
            return;
        }

        const reader = new FileReader();
        reader.onload = (event) => {
            try {
                const xmlContent = event.target.result;

                // 尝试导入到Blockly
                const imported = importBehaviorTreeXmlToBlockly(xmlContent);

                // 更新文件名
                appState.currentFileName = file.name;
                updateStatusBar();

                if (imported) {
                    appState.isModified = false;
                    appState.lastSavedTime = null;
                    updateXmlOutput();
                    showNotification(`✓ 已导入: ${file.name}`);
                } else {
                    // 降级：仅可视化
                    if (appState.visualizer) {
                        appState.visualizer.renderFromXml(xmlContent);
                    }
                    const xmlOutput = document.querySelector('#xml-output code');
                    if (xmlOutput) {
                        xmlOutput.textContent = xmlContent;
                    }
                    showNotification('XML已导入（仅可视化）', 'warning');
                }
            } catch (error) {
                console.error('Error importing dropped file:', error);
                showNotification('导入失败：无效的XML文件', 'error');
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
    document.getElementById('btn-copy-xml')?.addEventListener('click', handleCopyXml);
    document.getElementById('btn-validate')?.addEventListener('click', handleValidate);
    document.getElementById('btn-help')?.addEventListener('click', handleHelp);

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
    initVisualizer();
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

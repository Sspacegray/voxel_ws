/**
 * BehaviorTree XML 代码生成器
 * 将 Blockly 积木块转换为 BehaviorTree.CPP 格式的 XML
 */

import * as Blockly from 'blockly';

// 创建自定义代码生成器
export const btXmlGenerator = new Blockly.Generator('BehaviorTreeXML');

// 根配置（导入 XML 后可保持 main_tree_to_execute / tree ID）
btXmlGenerator._config = {
    format: '4',
    mainTreeToExecute: 'MainTree',
    treeId: 'MainTree'
};

btXmlGenerator.setConfig = function (partial) {
    if (!partial) return;
    btXmlGenerator._config = {
        ...btXmlGenerator._config,
        ...partial
    };
};

// 缩进设置
btXmlGenerator.INDENT = '  ';

// 初始化
btXmlGenerator.init = function (workspace) {
    this.definitions_ = Object.create(null);
    this.nameDB_ = new Blockly.Names(Blockly.Names.DEVELOPER_VARIABLE_TYPE);
};

// 完成时添加根元素
btXmlGenerator.finish = function (code) {
    const format = btXmlGenerator._config.format ?? '4';
    const mainTreeToExecute = btXmlGenerator._config.mainTreeToExecute ?? 'MainTree';
    const treeId = btXmlGenerator._config.treeId ?? mainTreeToExecute ?? 'MainTree';

    if (!code.trim()) {
        return `<?xml version="1.0"?>
<root BTCPP_format="${format}" main_tree_to_execute="${mainTreeToExecute}">
  <BehaviorTree ID="${treeId}">
    <!-- 请添加节点 -->
  </BehaviorTree>
</root>`;
    }

    return `<?xml version="1.0"?>
<root BTCPP_format="${format}" main_tree_to_execute="${mainTreeToExecute}">
  <BehaviorTree ID="${treeId}">
${code}  </BehaviorTree>
</root>`;
};

// 清理函数
btXmlGenerator.scrubNakedValue = function (code) {
    return code;
};

// 工具函数：添加缩进
function addIndent(code, level = 2) {
    const indent = '  '.repeat(level);
    return code.split('\n').map(line => line ? indent + line : '').join('\n');
}

// 工具函数：生成子节点代码
function generateChildren(block, inputName, generator) {
    let code = '';
    let childBlock = block.getInputTargetBlock(inputName);
    while (childBlock) {
        code += generator.blockToCode(childBlock);
        childBlock = childBlock.getNextBlock();
    }
    return code;
}

// Decorator 节点只允许一个 child：仅序列化第一个子块，避免生成非法 BT（多个 child）
function generateSingleChild(block, inputName, generator) {
    const childBlock = block.getInputTargetBlock(inputName);
    if (!childBlock) return '';
    return generator.blockToCode(childBlock);
}

function formatNameAttr(name) {
    const trimmed = String(name ?? '').trim();
    if (!trimmed) return '';
    return ` name="${escapeXmlAttr(trimmed)}"`;
}

function escapeXmlAttr(value) {
    return String(value ?? '')
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&apos;');
}

// ========================================
// 控制节点生成器
// ========================================

btXmlGenerator.forBlock['bt_sequence'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<Sequence${formatNameAttr(name)}/>\n`);
    }
    return addIndent(`<Sequence${formatNameAttr(name)}>\n`) +
        children +
        addIndent(`</Sequence>\n`);
};

btXmlGenerator.forBlock['bt_pipeline_sequence'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<PipelineSequence${formatNameAttr(name)}/>\n`);
    }
    return addIndent(`<PipelineSequence${formatNameAttr(name)}>\n`) +
        children +
        addIndent(`</PipelineSequence>\n`);
};

btXmlGenerator.forBlock['bt_fallback'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<Fallback${formatNameAttr(name)}/>\n`);
    }
    return addIndent(`<Fallback${formatNameAttr(name)}>\n`) +
        children +
        addIndent(`</Fallback>\n`);
};

btXmlGenerator.forBlock['bt_reactive_fallback'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<ReactiveFallback${formatNameAttr(name)}/>\n`);
    }
    return addIndent(`<ReactiveFallback${formatNameAttr(name)}>\n`) +
        children +
        addIndent(`</ReactiveFallback>\n`);
};

btXmlGenerator.forBlock['bt_reactive_sequence'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<ReactiveSequence${formatNameAttr(name)}/>\n`);
    }
    return addIndent(`<ReactiveSequence${formatNameAttr(name)}>\n`) +
        children +
        addIndent(`</ReactiveSequence>\n`);
};

btXmlGenerator.forBlock['bt_recovery'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const retries = block.getFieldValue('RETRIES');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<RecoveryNode${formatNameAttr(name)} number_of_retries="${retries}"/>\n`);
    }
    return addIndent(`<RecoveryNode${formatNameAttr(name)} number_of_retries="${retries}">\n`) +
        children +
        addIndent(`</RecoveryNode>\n`);
};

btXmlGenerator.forBlock['bt_round_robin'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<RoundRobin${formatNameAttr(name)}/>\n`);
    }
    return addIndent(`<RoundRobin${formatNameAttr(name)}>\n`) +
        children +
        addIndent(`</RoundRobin>\n`);
};

btXmlGenerator.forBlock['bt_parallel'] = function (block, generator) {
    const name = block.getFieldValue('NAME');
    const successThreshold = block.getFieldValue('SUCCESS_THRESHOLD');
    const failureThreshold = block.getFieldValue('FAILURE_THRESHOLD');
    const children = generateChildren(block, 'CHILDREN', generator);
    if (!children.trim()) {
        return addIndent(`<Parallel${formatNameAttr(name)} success_threshold="${successThreshold}" failure_threshold="${failureThreshold}"/>\n`);
    }
    return addIndent(`<Parallel${formatNameAttr(name)} success_threshold="${successThreshold}" failure_threshold="${failureThreshold}">\n`) +
        children +
        addIndent(`</Parallel>\n`);
};

// ========================================
// 装饰节点生成器
// ========================================

btXmlGenerator.forBlock['bt_rate_controller'] = function (block, generator) {
    const hz = block.getFieldValue('HZ');
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<RateController hz="${hz}"/>\n`);
    }
    return addIndent(`<RateController hz="${hz}">\n`) +
        child +
        addIndent(`</RateController>\n`);
};

btXmlGenerator.forBlock['bt_distance_controller'] = function (block, generator) {
    const distance = block.getFieldValue('DISTANCE');
    const globalFrame = block.getFieldValue('GLOBAL_FRAME');
    const robotFrame = block.getFieldValue('ROBOT_FRAME');
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<DistanceController distance="${distance}" global_frame="${escapeXmlAttr(globalFrame)}" robot_base_frame="${escapeXmlAttr(robotFrame)}"/>\n`);
    }
    return addIndent(`<DistanceController distance="${distance}" global_frame="${escapeXmlAttr(globalFrame)}" robot_base_frame="${escapeXmlAttr(robotFrame)}">\n`) +
        child +
        addIndent(`</DistanceController>\n`);
};

btXmlGenerator.forBlock['bt_speed_controller'] = function (block, generator) {
    const minRate = block.getFieldValue('MIN_RATE');
    const maxRate = block.getFieldValue('MAX_RATE');
    const minSpeed = block.getFieldValue('MIN_SPEED');
    const maxSpeed = block.getFieldValue('MAX_SPEED');
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<SpeedController min_rate="${minRate}" max_rate="${maxRate}" min_speed="${minSpeed}" max_speed="${maxSpeed}"/>\n`);
    }
    return addIndent(`<SpeedController min_rate="${minRate}" max_rate="${maxRate}" min_speed="${minSpeed}" max_speed="${maxSpeed}">\n`) +
        child +
        addIndent(`</SpeedController>\n`);
};

btXmlGenerator.forBlock['bt_goal_updater'] = function (block, generator) {
    const inputGoal = block.getFieldValue('INPUT_GOAL');
    const outputGoal = block.getFieldValue('OUTPUT_GOAL');
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<GoalUpdater input_goal="${escapeXmlAttr(inputGoal)}" output_goal="${escapeXmlAttr(outputGoal)}"/>\n`);
    }
    return addIndent(`<GoalUpdater input_goal="${escapeXmlAttr(inputGoal)}" output_goal="${escapeXmlAttr(outputGoal)}">\n`) +
        child +
        addIndent(`</GoalUpdater>\n`);
};

btXmlGenerator.forBlock['bt_single_trigger'] = function (block, generator) {
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<SingleTrigger/>\n`);
    }
    return addIndent(`<SingleTrigger>\n`) +
        child +
        addIndent(`</SingleTrigger>\n`);
};

btXmlGenerator.forBlock['bt_inverter'] = function (block, generator) {
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<Inverter/>\n`);
    }
    return addIndent(`<Inverter>\n`) +
        child +
        addIndent(`</Inverter>\n`);
};

btXmlGenerator.forBlock['bt_force_success'] = function (block, generator) {
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<ForceSuccess/>\n`);
    }
    return addIndent(`<ForceSuccess>\n`) +
        child +
        addIndent(`</ForceSuccess>\n`);
};

btXmlGenerator.forBlock['bt_force_failure'] = function (block, generator) {
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<ForceFailure/>\n`);
    }
    return addIndent(`<ForceFailure>\n`) +
        child +
        addIndent(`</ForceFailure>\n`);
};

btXmlGenerator.forBlock['bt_repeat'] = function (block, generator) {
    const numCycles = block.getFieldValue('NUM_CYCLES');
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<Repeat num_cycles="${numCycles}"/>\n`);
    }
    return addIndent(`<Repeat num_cycles="${numCycles}">\n`) +
        child +
        addIndent(`</Repeat>\n`);
};

btXmlGenerator.forBlock['bt_timeout'] = function (block, generator) {
    const timeout = block.getFieldValue('TIMEOUT_SEC');
    const child = generateSingleChild(block, 'CHILD', generator);
    if (!child.trim()) {
        return addIndent(`<Timeout msec="${timeout * 1000}"/>\n`);
    }
    return addIndent(`<Timeout msec="${timeout * 1000}">\n`) +
        child +
        addIndent(`</Timeout>\n`);
};

// ========================================
// 动作节点生成器
// ========================================

btXmlGenerator.forBlock['bt_navigate_to_pose'] = function (block, generator) {
    const goal = block.getFieldValue('GOAL');
    const behaviorTree = block.getFieldValue('BEHAVIOR_TREE');
    const serverName = block.getFieldValue('SERVER_NAME');
    let attrs = `goal="${escapeXmlAttr(goal)}" server_name="${escapeXmlAttr(serverName)}"`;
    if (behaviorTree) {
        attrs += ` behavior_tree="${escapeXmlAttr(behaviorTree)}"`;
    }
    return addIndent(`<NavigateToPose ${attrs}/>\n`);
};

btXmlGenerator.forBlock['bt_navigate_through_poses'] = function (block, generator) {
    const goals = block.getFieldValue('GOALS');
    const serverName = block.getFieldValue('SERVER_NAME');
    return addIndent(`<NavigateThroughPoses goals="${escapeXmlAttr(goals)}" server_name="${escapeXmlAttr(serverName)}"/>\n`);
};

btXmlGenerator.forBlock['bt_compute_path_to_pose'] = function (block, generator) {
    const goal = block.getFieldValue('GOAL');
    const path = block.getFieldValue('PATH');
    const plannerId = block.getFieldValue('PLANNER_ID');
    return addIndent(`<ComputePathToPose goal="${escapeXmlAttr(goal)}" path="${escapeXmlAttr(path)}" planner_id="${escapeXmlAttr(plannerId)}"/>\n`);
};

btXmlGenerator.forBlock['bt_follow_path'] = function (block, generator) {
    const path = block.getFieldValue('PATH');
    const controllerId = block.getFieldValue('CONTROLLER_ID');
    const errorCodeId = block.getFieldValue('ERROR_CODE_ID');
    const attrs = [
        `path="${escapeXmlAttr(path)}"`,
        `controller_id="${escapeXmlAttr(controllerId)}"`
    ];
    if (errorCodeId) {
        attrs.push(`error_code_id="${escapeXmlAttr(errorCodeId)}"`);
    }
    return addIndent(`<FollowPath ${attrs.join(' ')}/>\n`);
};

btXmlGenerator.forBlock['bt_compute_route'] = function (block, generator) {
    const startId = block.getFieldValue('START_ID');
    const goalId = block.getFieldValue('GOAL_ID');
    const route = block.getFieldValue('ROUTE');
    const path = block.getFieldValue('PATH');
    return addIndent(`<ComputeRoute start_id="${escapeXmlAttr(startId)}" goal_id="${escapeXmlAttr(goalId)}" route="${escapeXmlAttr(route)}" path="${escapeXmlAttr(path)}"/>\n`);
};

btXmlGenerator.forBlock['bt_compute_and_track_route'] = function (block, generator) {
    const goal = block.getFieldValue('GOAL');
    const path = block.getFieldValue('PATH');
    const route = block.getFieldValue('ROUTE');
    const usePoses = block.getFieldValue('USE_POSES') === 'TRUE';
    const errorCodeId = block.getFieldValue('ERROR_CODE_ID');

    const attrs = [
        `goal="${escapeXmlAttr(goal)}"`,
        `path="${escapeXmlAttr(path)}"`,
        `route="${escapeXmlAttr(route)}"`,
        `use_poses="${usePoses}"`
    ];
    if (errorCodeId) {
        attrs.push(`error_code_id="${escapeXmlAttr(errorCodeId)}"`);
    }
    return addIndent(`<ComputeAndTrackRoute ${attrs.join(' ')}/>\n`);
};

btXmlGenerator.forBlock['bt_backup'] = function (block, generator) {
    const backupDist = block.getFieldValue('BACKUP_DIST');
    const backupSpeed = block.getFieldValue('BACKUP_SPEED');
    return addIndent(`<BackUp backup_dist="${backupDist}" backup_speed="${backupSpeed}"/>\n`);
};

btXmlGenerator.forBlock['bt_spin'] = function (block, generator) {
    const spinDist = block.getFieldValue('SPIN_DIST');
    return addIndent(`<Spin spin_dist="${spinDist}"/>\n`);
};

btXmlGenerator.forBlock['bt_wait'] = function (block, generator) {
    const waitDuration = block.getFieldValue('WAIT_DURATION');
    return addIndent(`<Wait wait_duration="${waitDuration}"/>\n`);
};

btXmlGenerator.forBlock['bt_clear_costmap'] = function (block, generator) {
    const serviceName = block.getFieldValue('SERVICE_NAME');
    return addIndent(`<ClearEntireCostmap service_name="${escapeXmlAttr(serviceName)}"/>\n`);
};

btXmlGenerator.forBlock['bt_dock_robot'] = function (block, generator) {
    const dockId = block.getFieldValue('DOCK_ID');
    const navigateToStaging = block.getFieldValue('NAVIGATE_TO_STAGING') === 'TRUE';
    return addIndent(`<DockRobot dock_id="${escapeXmlAttr(dockId)}" navigate_to_staging_pose="${navigateToStaging}"/>\n`);
};

btXmlGenerator.forBlock['bt_undock_robot'] = function (block, generator) {
    const dockType = block.getFieldValue('DOCK_TYPE');
    let attrs = '';
    if (dockType) {
        attrs = ` dock_type="${escapeXmlAttr(dockType)}"`;
    }
    return addIndent(`<UndockRobot${attrs}/>\n`);
};

btXmlGenerator.forBlock['bt_smooth_path'] = function (block, generator) {
    const unsmoothedPath = block.getFieldValue('UNSMOOTHED_PATH');
    const smoothedPath = block.getFieldValue('SMOOTHED_PATH');
    const smootherId = block.getFieldValue('SMOOTHER_ID');
    return addIndent(`<SmoothPath unsmoothed_path="${escapeXmlAttr(unsmoothedPath)}" smoothed_path="${escapeXmlAttr(smoothedPath)}" smoother_id="${escapeXmlAttr(smootherId)}"/>\n`);
};

btXmlGenerator.forBlock['bt_truncate_path'] = function (block, generator) {
    const inputPath = block.getFieldValue('INPUT_PATH');
    const outputPath = block.getFieldValue('OUTPUT_PATH');
    const distance = block.getFieldValue('DISTANCE');
    return addIndent(`<TruncatePath input_path="${escapeXmlAttr(inputPath)}" output_path="${escapeXmlAttr(outputPath)}" distance="${distance}"/>\n`);
};

btXmlGenerator.forBlock['bt_reinitialize_localization'] = function (block, generator) {
    const serviceName = block.getFieldValue('SERVICE_NAME');
    return addIndent(`<ReinitializeGlobalLocalization service_name="${escapeXmlAttr(serviceName)}"/>\n`);
};

btXmlGenerator.forBlock['bt_controller_selector'] = function (block, generator) {
    const defaultController = block.getFieldValue('DEFAULT_CONTROLLER');
    const selectedController = block.getFieldValue('SELECTED_CONTROLLER');
    const topicName = block.getFieldValue('TOPIC_NAME') || 'controller_selector';
    return addIndent(`<ControllerSelector default_controller="${escapeXmlAttr(defaultController)}" selected_controller="${escapeXmlAttr(selectedController)}" topic_name="${escapeXmlAttr(topicName)}"/>\n`);
};

btXmlGenerator.forBlock['bt_planner_selector'] = function (block, generator) {
    const defaultPlanner = block.getFieldValue('DEFAULT_PLANNER');
    const selectedPlanner = block.getFieldValue('SELECTED_PLANNER');
    const topicName = block.getFieldValue('TOPIC_NAME') || 'planner_selector';
    return addIndent(`<PlannerSelector default_planner="${escapeXmlAttr(defaultPlanner)}" selected_planner="${escapeXmlAttr(selectedPlanner)}" topic_name="${escapeXmlAttr(topicName)}"/>\n`);
};

// ========================================
// 条件节点生成器
// ========================================

btXmlGenerator.forBlock['bt_goal_reached'] = function (block, generator) {
    const goal = block.getFieldValue('GOAL');
    const robotBaseFrame = block.getFieldValue('ROBOT_BASE_FRAME');
    return addIndent(`<GoalReached goal="${escapeXmlAttr(goal)}" robot_base_frame="${escapeXmlAttr(robotBaseFrame)}"/>\n`);
};

btXmlGenerator.forBlock['bt_goal_updated'] = function (block, generator) {
    return addIndent(`<GoalUpdated/>\n`);
};

btXmlGenerator.forBlock['bt_is_battery_low'] = function (block, generator) {
    const minBattery = block.getFieldValue('MIN_BATTERY');
    const batteryTopic = block.getFieldValue('BATTERY_TOPIC');
    const isVoltage = block.getFieldValue('IS_VOLTAGE') === 'TRUE';
    return addIndent(`<IsBatteryLow min_battery="${minBattery}" battery_topic="${escapeXmlAttr(batteryTopic)}" is_voltage="${isVoltage}"/>\n`);
};

btXmlGenerator.forBlock['bt_is_battery_charging'] = function (block, generator) {
    const batteryTopic = block.getFieldValue('BATTERY_TOPIC');
    return addIndent(`<IsBatteryCharging battery_topic="${escapeXmlAttr(batteryTopic)}"/>\n`);
};

btXmlGenerator.forBlock['bt_is_path_valid'] = function (block, generator) {
    const path = block.getFieldValue('PATH');
    return addIndent(`<IsPathValid path="${escapeXmlAttr(path)}"/>\n`);
};

btXmlGenerator.forBlock['bt_is_stuck'] = function (block, generator) {
    return addIndent(`<IsStuck/>\n`);
};

btXmlGenerator.forBlock['bt_is_stopped'] = function (block, generator) {
    const velocityThreshold = block.getFieldValue('VELOCITY_THRESHOLD');
    const durationStopped = block.getFieldValue('DURATION_STOPPED');
    return addIndent(`<IsStopped velocity_threshold="${velocityThreshold}" duration_stopped="${durationStopped}"/>\n`);
};

btXmlGenerator.forBlock['bt_initial_pose_received'] = function (block, generator) {
    return addIndent(`<InitialPoseReceived/>\n`);
};

btXmlGenerator.forBlock['bt_time_expired'] = function (block, generator) {
    const seconds = block.getFieldValue('SECONDS');
    return addIndent(`<TimeExpired seconds="${seconds}"/>\n`);
};

btXmlGenerator.forBlock['bt_distance_traveled'] = function (block, generator) {
    const distance = block.getFieldValue('DISTANCE');
    const globalFrame = block.getFieldValue('GLOBAL_FRAME');
    const robotBaseFrame = block.getFieldValue('ROBOT_BASE_FRAME');
    return addIndent(`<DistanceTraveled distance="${distance}" global_frame="${escapeXmlAttr(globalFrame)}" robot_base_frame="${escapeXmlAttr(robotBaseFrame)}"/>\n`);
};

btXmlGenerator.forBlock['bt_transform_available'] = function (block, generator) {
    const parent = block.getFieldValue('PARENT');
    const child = block.getFieldValue('CHILD');
    return addIndent(`<TransformAvailable parent="${escapeXmlAttr(parent)}" child="${escapeXmlAttr(child)}"/>\n`);
};

btXmlGenerator.forBlock['bt_would_controller_recovery_help'] = function (block, generator) {
    const errorCode = block.getFieldValue('ERROR_CODE');
    return addIndent(`<WouldAControllerRecoveryHelp error_code="${escapeXmlAttr(errorCode)}"/>\n`);
};

btXmlGenerator.forBlock['bt_would_planner_recovery_help'] = function (block, generator) {
    const errorCode = block.getFieldValue('ERROR_CODE');
    return addIndent(`<WouldAPlannerRecoveryHelp error_code="${escapeXmlAttr(errorCode)}"/>\n`);
};

// ========================================
// 通用/未知节点生成器
// ========================================

btXmlGenerator.forBlock['bt_unknown_node'] = function (block, generator) {
    const tag = (block.getFieldValue('TAG') || 'CustomNode').trim();
    const attrs = (block.getFieldValue('ATTRS') || '').trim();
    const children = generateChildren(block, 'CHILDREN', generator);

    const attrStr = attrs ? ` ${attrs}` : '';
    if (!children.trim()) {
        return addIndent(`<${tag}${attrStr}/>\n`);
    }
    return addIndent(`<${tag}${attrStr}>\n`) +
        children +
        addIndent(`</${tag}>\n`);
};

btXmlGenerator.forBlock['bt_always_success'] = function (block, generator) {
    return addIndent(`<AlwaysSuccess/>\n`);
};

export default btXmlGenerator;

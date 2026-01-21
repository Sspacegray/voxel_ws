/**
 * 行为树验证工具
 * 检查行为树结构是否合理并给出修改建议
 */

// 验证规则
const validationRules = {
    // 控制节点类型
    controlNodes: [
        'bt_sequence', 'bt_pipeline_sequence', 'bt_fallback',
        'bt_reactive_fallback', 'bt_reactive_sequence',
        'bt_recovery', 'bt_round_robin', 'bt_parallel'
    ],

    // 装饰节点类型
    decoratorNodes: [
        'bt_rate_controller', 'bt_distance_controller', 'bt_speed_controller',
        'bt_goal_updater', 'bt_single_trigger', 'bt_inverter',
        'bt_force_success', 'bt_force_failure', 'bt_repeat', 'bt_timeout'
    ],

    // 动作节点类型
    actionNodes: [
        'bt_navigate_to_pose', 'bt_navigate_through_poses',
        'bt_compute_path_to_pose', 'bt_follow_path',
        'bt_compute_route', 'bt_compute_and_track_route',
        'bt_backup', 'bt_spin', 'bt_wait', 'bt_clear_costmap',
        'bt_dock_robot', 'bt_undock_robot', 'bt_smooth_path',
        'bt_truncate_path', 'bt_reinitialize_localization',
        'bt_controller_selector', 'bt_planner_selector'
    ],

    // 条件节点类型
    conditionNodes: [
        'bt_goal_reached', 'bt_goal_updated',
        'bt_is_battery_low', 'bt_is_battery_charging',
        'bt_is_path_valid', 'bt_is_stuck', 'bt_is_stopped',
        'bt_initial_pose_received', 'bt_time_expired',
        'bt_distance_traveled', 'bt_transform_available',
        'bt_would_controller_recovery_help', 'bt_would_planner_recovery_help'
    ]
};

// 验证结果类型
export const ValidationLevel = {
    ERROR: 'error',
    WARNING: 'warning',
    INFO: 'info',
    SUCCESS: 'success'
};

/**
 * 验证行为树
 * @param {Blockly.Workspace} workspace - Blockly工作区
 * @returns {Object} 验证结果
 */
export function validateBehaviorTree(workspace) {
    if (!workspace) {
        return {
            isValid: false,
            score: 0,
            issues: [{ level: ValidationLevel.ERROR, message: '工作区不存在', suggestion: '请刷新页面' }]
        };
    }

    const blocks = workspace.getAllBlocks(false);
    const issues = [];

    // 检查是否有积木块
    if (blocks.length === 0) {
        return {
            isValid: false,
            score: 0,
            issues: [{
                level: ValidationLevel.INFO,
                message: '行为树为空',
                suggestion: '从左侧工具箱拖拽节点开始构建行为树'
            }]
        };
    }

    // 找到顶层块
    const topBlocks = workspace.getTopBlocks(false);

    // 1. 检查是否只有一个根节点
    if (topBlocks.length > 1) {
        issues.push({
            level: ValidationLevel.WARNING,
            message: `存在 ${topBlocks.length} 个独立的顶层节点`,
            suggestion: '行为树通常只有一个根节点，请将多个分支整合到一个控制节点下'
        });
    }

    // 2. 检查根节点类型
    for (const topBlock of topBlocks) {
        const blockType = topBlock.type;

        // 根节点应该是控制节点
        if (!isControlNode(blockType) && !isDecoratorNode(blockType)) {
            if (isActionNode(blockType) || isConditionNode(blockType)) {
                issues.push({
                    level: ValidationLevel.WARNING,
                    message: `顶层节点 "${getBlockDisplayName(topBlock)}" 是叶子节点`,
                    suggestion: '通常根节点应该是控制节点（如Sequence、Fallback），让它们包含具体的动作/条件节点'
                });
            }
        }

        // 递归验证子节点
        validateBlock(topBlock, issues, 0);
    }

    // 3. 检查常见模式
    checkCommonPatterns(blocks, issues);

    // 4. 检查黑板变量使用
    checkBlackboardVariables(blocks, issues);

    // 计算分数
    const errorCount = issues.filter(i => i.level === ValidationLevel.ERROR).length;
    const warningCount = issues.filter(i => i.level === ValidationLevel.WARNING).length;

    let score = 100;
    score -= errorCount * 20;
    score -= warningCount * 5;
    score = Math.max(0, Math.min(100, score));

    const isValid = errorCount === 0;

    // 添加总结
    if (issues.length === 0) {
        issues.push({
            level: ValidationLevel.SUCCESS,
            message: '行为树结构验证通过！',
            suggestion: '当前行为树结构看起来合理，可以导出使用。'
        });
    }

    return { isValid, score, issues, blockCount: blocks.length };
}

/**
 * 递归验证单个块
 */
function validateBlock(block, issues, depth) {
    const blockType = block.type;
    const displayName = getBlockDisplayName(block);

    // 检查过深的嵌套
    if (depth > 10) {
        issues.push({
            level: ValidationLevel.WARNING,
            message: `节点嵌套过深（${depth}层）: "${displayName}"`,
            suggestion: '过深的嵌套会增加理解和调试难度，考虑使用SubTree简化结构'
        });
    }

    // 检查控制节点是否有子节点
    if (isControlNode(blockType)) {
        const childrenInput = block.getInput('CHILDREN');
        const childBlock = childrenInput?.connection?.targetBlock();

        if (!childBlock) {
            issues.push({
                level: ValidationLevel.ERROR,
                message: `控制节点 "${displayName}" 没有子节点`,
                suggestion: '控制节点必须包含子节点才能正常工作'
            });
        } else {
            // 检查子节点数量
            let childCount = 0;
            let current = childBlock;
            while (current) {
                childCount++;
                validateBlock(current, issues, depth + 1);
                current = current.getNextBlock();
            }

            // RecoveryNode应该有2个子节点
            if (blockType === 'bt_recovery' && childCount < 2) {
                issues.push({
                    level: ValidationLevel.WARNING,
                    message: `RecoveryNode "${displayName}" 只有 ${childCount} 个子节点`,
                    suggestion: 'RecoveryNode通常需要2个子节点：第一个是主任务，第二个是恢复动作'
                });
            }

            // Sequence只有一个子节点没有意义
            if ((blockType === 'bt_sequence' || blockType === 'bt_fallback') && childCount === 1) {
                issues.push({
                    level: ValidationLevel.INFO,
                    message: `${blockType === 'bt_sequence' ? 'Sequence' : 'Fallback'} "${displayName}" 只有1个子节点`,
                    suggestion: '单子节点的控制节点可以考虑简化或添加更多分支'
                });
            }
        }
    }

    // 检查装饰节点是否有子节点
    if (isDecoratorNode(blockType)) {
        const childInput = block.getInput('CHILD');
        const childBlock = childInput?.connection?.targetBlock();

        if (!childBlock) {
            issues.push({
                level: ValidationLevel.ERROR,
                message: `装饰节点 "${displayName}" 没有子节点`,
                suggestion: '装饰节点必须包含一个子节点'
            });
        } else {
            // 装饰节点只能有一个 child，Blockly 的 statement input 可能被串多个块
            if (childBlock.getNextBlock()) {
                issues.push({
                    level: ValidationLevel.WARNING,
                    message: `装饰节点 "${displayName}" 的 child 中存在多个节点`,
                    suggestion: '装饰节点只能包裹一个子节点；如需顺序执行多个节点，请在 child 中放入 Sequence 等控制节点'
                });
            }
            validateBlock(childBlock, issues, depth + 1);
        }
    }

    // 检查下一个兄弟节点
    const nextBlock = block.getNextBlock();
    if (nextBlock) {
        validateBlock(nextBlock, issues, depth);
    }
}

/**
 * 检查常见的行为树模式
 */
function checkCommonPatterns(blocks, issues) {
    // 检查是否有导航动作
    const hasNavigation = blocks.some(b =>
        ['bt_navigate_to_pose', 'bt_navigate_through_poses',
            'bt_compute_path_to_pose', 'bt_follow_path',
            'bt_compute_and_track_route'].includes(b.type)
    );

    // 检查是否有恢复机制
    const hasRecovery = blocks.some(b =>
        ['bt_recovery', 'bt_backup', 'bt_spin',
            'bt_clear_costmap', 'bt_wait'].includes(b.type)
    );

    if (hasNavigation && !hasRecovery) {
        issues.push({
            level: ValidationLevel.INFO,
            message: '行为树包含导航动作但没有恢复机制',
            suggestion: '建议添加RecoveryNode包裹导航，配合BackUp/Spin/ClearCostmap等恢复动作，提高导航鲁棒性'
        });
    }

    // 检查ComputePathToPose和FollowPath是否配对
    const hasComputePath = blocks.some(b => b.type === 'bt_compute_path_to_pose');
    const hasFollowPath = blocks.some(b => b.type === 'bt_follow_path');

    if (hasComputePath && !hasFollowPath) {
        issues.push({
            level: ValidationLevel.WARNING,
            message: '有ComputePathToPose但没有FollowPath',
            suggestion: 'ComputePathToPose只计算路径，需要配合FollowPath执行跟踪'
        });
    }

    if (hasFollowPath && !hasComputePath &&
        !blocks.some(b => b.type === 'bt_compute_and_track_route')) {
        issues.push({
            level: ValidationLevel.WARNING,
            message: '有FollowPath但没有路径来源',
            suggestion: 'FollowPath需要读取路径，确保之前有ComputePathToPose或其他节点生成路径'
        });
    }
}

/**
 * 检查黑板变量使用
 */
function checkBlackboardVariables(blocks, issues) {
    const producers = new Map(); // 变量生产者
    const consumers = new Map(); // 变量消费者

    for (const block of blocks) {
        // 分析每个块的输入输出
        const blockInfo = getBlockIOInfo(block);

        for (const output of blockInfo.outputs) {
            if (!producers.has(output)) {
                producers.set(output, []);
            }
            producers.get(output).push(block);
        }

        for (const input of blockInfo.inputs) {
            if (!consumers.has(input)) {
                consumers.set(input, []);
            }
            consumers.get(input).push(block);
        }
    }

    // 检查是否有未被生产的变量被消费
    for (const [varName, consumerBlocks] of consumers) {
        if (!producers.has(varName) && !['goal', 'goals'].includes(varName)) {
            // goal通常由外部提供，不需要警告
            if (consumerBlocks.length > 0) {
                issues.push({
                    level: ValidationLevel.INFO,
                    message: `黑板变量 "{${varName}}" 被使用但未见产生`,
                    suggestion: `确保变量 {${varName}} 在使用前被正确设置（可能由外部或前序节点提供）`
                });
            }
        }
    }
}

/**
 * 获取块的输入输出变量
 */
function getBlockIOInfo(block) {
    const outputs = [];
    const inputs = [];

    // 解析黑板变量格式: {variable_name}
    const extractVar = (value) => {
        if (!value) return null;
        const match = String(value).match(/^\{(.+)\}$/);
        return match ? match[1] : null;
    };

    const blockType = block.type;

    if (blockType === 'bt_compute_path_to_pose') {
        const goal = extractVar(block.getFieldValue('GOAL'));
        const path = extractVar(block.getFieldValue('PATH'));
        if (goal) inputs.push(goal);
        if (path) outputs.push(path);
    } else if (blockType === 'bt_follow_path') {
        const path = extractVar(block.getFieldValue('PATH'));
        if (path) inputs.push(path);
    } else if (blockType === 'bt_compute_and_track_route') {
        const goal = extractVar(block.getFieldValue('GOAL'));
        const path = extractVar(block.getFieldValue('PATH'));
        const route = extractVar(block.getFieldValue('ROUTE'));
        if (goal) inputs.push(goal);
        if (path) outputs.push(path);
        if (route) outputs.push(route);
    } else if (blockType === 'bt_navigate_to_pose') {
        const goal = extractVar(block.getFieldValue('GOAL'));
        if (goal) inputs.push(goal);
    }

    return { outputs, inputs };
}

// 辅助函数
function isControlNode(type) {
    return validationRules.controlNodes.includes(type);
}

function isDecoratorNode(type) {
    return validationRules.decoratorNodes.includes(type);
}

function isActionNode(type) {
    return validationRules.actionNodes.includes(type);
}

function isConditionNode(type) {
    return validationRules.conditionNodes.includes(type);
}

function getBlockDisplayName(block) {
    const name = block.getFieldValue('NAME');
    if (name && name.trim()) {
        return name;
    }
    // 返回类型名称的简化版
    return block.type.replace('bt_', '').replace(/_/g, ' ');
}

/**
 * 生成验证报告HTML
 */
export function generateValidationReportHtml(result) {
    const { isValid, score, issues, blockCount } = result;

    const scoreColor = score >= 80 ? '#22c55e' : score >= 50 ? '#eab308' : '#ef4444';
    const statusIcon = isValid ? '✅' : '⚠️';

    let html = `
        <div style="margin-bottom: 16px;">
            <div style="display: flex; align-items: center; gap: 12px; margin-bottom: 8px;">
                <span style="font-size: 24px;">${statusIcon}</span>
                <span style="font-size: 18px; font-weight: 600;">
                    评分: <span style="color: ${scoreColor};">${score}</span>/100
                </span>
                <span style="color: #64748b; font-size: 13px;">
                    (${blockCount || 0} 个节点)
                </span>
            </div>
        </div>
        <div style="display: flex; flex-direction: column; gap: 10px;">
    `;

    for (const issue of issues) {
        const iconMap = {
            [ValidationLevel.ERROR]: '❌',
            [ValidationLevel.WARNING]: '⚠️',
            [ValidationLevel.INFO]: '💡',
            [ValidationLevel.SUCCESS]: '✅'
        };
        const bgMap = {
            [ValidationLevel.ERROR]: 'rgba(239, 68, 68, 0.15)',
            [ValidationLevel.WARNING]: 'rgba(234, 179, 8, 0.15)',
            [ValidationLevel.INFO]: 'rgba(59, 130, 246, 0.15)',
            [ValidationLevel.SUCCESS]: 'rgba(34, 197, 94, 0.15)'
        };

        html += `
            <div style="
                background: ${bgMap[issue.level]};
                border-radius: 8px;
                padding: 10px 12px;
            ">
                <div style="display: flex; align-items: flex-start; gap: 8px;">
                    <span>${iconMap[issue.level]}</span>
                    <div>
                        <div style="font-weight: 500; margin-bottom: 4px;">${issue.message}</div>
                        <div style="font-size: 12px; color: #94a3b8;">${issue.suggestion}</div>
                    </div>
                </div>
            </div>
        `;
    }

    html += '</div>';

    return html;
}

export default { validateBehaviorTree, generateValidationReportHtml, ValidationLevel };

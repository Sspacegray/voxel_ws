/**
 * Blockly 工具箱配置
 * Toolbox configuration with clean English labels
 */

export const toolboxConfig = {
    kind: 'categoryToolbox',
    contents: [
        // ==============================
        // 控制节点
        // ==============================
        {
            kind: 'category',
            name: 'Control 控制',
            colour: '#3b82f6',
            contents: [
                { kind: 'block', type: 'bt_sequence' },
                { kind: 'block', type: 'bt_pipeline_sequence' },
                { kind: 'block', type: 'bt_fallback' },
                { kind: 'block', type: 'bt_reactive_fallback' },
                { kind: 'block', type: 'bt_reactive_sequence' },
                { kind: 'block', type: 'bt_recovery' },
                { kind: 'block', type: 'bt_round_robin' },
                { kind: 'block', type: 'bt_parallel' }
            ]
        },

        // ==============================
        // 装饰节点
        // ==============================
        {
            kind: 'category',
            name: 'Decorators 装饰',
            colour: '#14b8a6',
            contents: [
                { kind: 'block', type: 'bt_rate_controller' },
                { kind: 'block', type: 'bt_distance_controller' },
                { kind: 'block', type: 'bt_speed_controller' },
                { kind: 'block', type: 'bt_goal_updater' },
                { kind: 'block', type: 'bt_single_trigger' },
                { kind: 'block', type: 'bt_inverter' },
                { kind: 'block', type: 'bt_force_success' },
                { kind: 'block', type: 'bt_force_failure' },
                { kind: 'block', type: 'bt_repeat' },
                { kind: 'block', type: 'bt_timeout' }
            ]
        },

        // ==============================
        // 导航动作
        // ==============================
        {
            kind: 'category',
            name: 'Navigation 导航',
            colour: '#f97316',
            contents: [
                { kind: 'block', type: 'bt_navigate_to_pose' },
                { kind: 'block', type: 'bt_navigate_through_poses' },
                { kind: 'block', type: 'bt_compute_path_to_pose' },
                { kind: 'block', type: 'bt_follow_path' },
                { kind: 'block', type: 'bt_compute_route' },
                { kind: 'block', type: 'bt_compute_and_track_route' }
            ]
        },

        // ==============================
        // 恢复动作
        // ==============================
        {
            kind: 'category',
            name: 'Recovery 恢复',
            colour: '#ef4444',
            contents: [
                { kind: 'block', type: 'bt_backup' },
                { kind: 'block', type: 'bt_spin' },
                { kind: 'block', type: 'bt_wait' },
                { kind: 'block', type: 'bt_clear_costmap' },
                { kind: 'block', type: 'bt_reinitialize_localization' }
            ]
        },

        // ==============================
        // 充电动作
        // ==============================
        {
            kind: 'category',
            name: 'Charging 充电',
            colour: '#22c55e',
            contents: [
                { kind: 'block', type: 'bt_dock_robot' },
                { kind: 'block', type: 'bt_undock_robot' }
            ]
        },

        // ==============================
        // 路径处理
        // ==============================
        {
            kind: 'category',
            name: 'Path Utils 路径工具',
            colour: '#a855f7',
            contents: [
                { kind: 'block', type: 'bt_smooth_path' },
                { kind: 'block', type: 'bt_truncate_path' }
            ]
        },

        // ==============================
        // 选择器
        // ==============================
        {
            kind: 'category',
            name: 'Selectors 选择器',
            colour: '#64748b',
            contents: [
                { kind: 'block', type: 'bt_controller_selector' },
                { kind: 'block', type: 'bt_planner_selector' }
            ]
        },

        { kind: 'sep' },

        // ==============================
        // 条件节点
        // ==============================
        {
            kind: 'category',
            name: 'Conditions 条件',
            colour: '#a855f7',
            contents: [
                {
                    kind: 'category',
                    name: 'Goal 目标',
                    contents: [
                        { kind: 'block', type: 'bt_goal_reached' },
                        { kind: 'block', type: 'bt_goal_updated' }
                    ]
                },
                {
                    kind: 'category',
                    name: 'Battery 电池',
                    contents: [
                        { kind: 'block', type: 'bt_is_battery_low' },
                        { kind: 'block', type: 'bt_is_battery_charging' }
                    ]
                },
                {
                    kind: 'category',
                    name: 'Path 路径',
                    contents: [
                        { kind: 'block', type: 'bt_is_path_valid' }
                    ]
                },
                {
                    kind: 'category',
                    name: 'Motion 运动状态',
                    contents: [
                        { kind: 'block', type: 'bt_is_stuck' },
                        { kind: 'block', type: 'bt_is_stopped' }
                    ]
                },
                {
                    kind: 'category',
                    name: 'Localization 定位',
                    contents: [
                        { kind: 'block', type: 'bt_initial_pose_received' },
                        { kind: 'block', type: 'bt_transform_available' }
                    ]
                },
                {
                    kind: 'category',
                    name: 'Time/Distance 时间/距离',
                    contents: [
                        { kind: 'block', type: 'bt_time_expired' },
                        { kind: 'block', type: 'bt_distance_traveled' }
                    ]
                },
                {
                    kind: 'category',
                    name: 'Recovery Helper 恢复建议',
                    contents: [
                        { kind: 'block', type: 'bt_would_controller_recovery_help' },
                        { kind: 'block', type: 'bt_would_planner_recovery_help' }
                    ]
                }
            ]
        }
    ]
};

export default toolboxConfig;

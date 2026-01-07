/**
 * 行为树图形可视化 - 详细版本
 * 显示完整节点名称和所有属性
 */

import * as d3 from 'd3';

// 节点类型颜色映射
const nodeColors = {
    // 控制节点 - 蓝色系
    'Sequence': '#3b82f6',
    'PipelineSequence': '#2563eb',
    'Fallback': '#22c55e',
    'ReactiveFallback': '#16a34a',
    'ReactiveSequence': '#3b82f6',
    'RecoveryNode': '#6366f1',
    'RoundRobin': '#8b5cf6',
    'Parallel': '#a855f7',

    // 装饰节点 - 青色系
    'RateController': '#14b8a6',
    'DistanceController': '#0d9488',
    'SpeedController': '#0f766e',
    'GoalUpdater': '#06b6d4',
    'GoalUpdatedController': '#0891b2',
    'SingleTrigger': '#0ea5e9',
    'Inverter': '#14b8a6',
    'ForceSuccess': '#22c55e',
    'ForceFailure': '#ef4444',
    'Repeat': '#f59e0b',
    'Timeout': '#f97316',

    // 动作节点 - 橙色系
    'NavigateToPose': '#f97316',
    'NavigateThroughPoses': '#ea580c',
    'ComputePathToPose': '#fb923c',
    'ComputePathThroughPoses': '#fdba74',
    'FollowPath': '#f97316',
    'ComputeRoute': '#c2410c',
    'ComputeAndTrackRoute': '#9a3412',
    'BackUp': '#dc2626',
    'Spin': '#b91c1c',
    'Wait': '#eab308',
    'ClearEntireCostmap': '#ef4444',
    'ClearCostmapExceptRegion': '#f87171',
    'ClearCostmapAroundRobot': '#fca5a5',
    'DockRobot': '#22c55e',
    'UndockRobot': '#15803d',
    'SmoothPath': '#a855f7',
    'TruncatePath': '#9333ea',
    'TruncatePathLocal': '#7c3aed',
    'ReinitializeGlobalLocalization': '#ef4444',
    'ControllerSelector': '#64748b',
    'PlannerSelector': '#475569',
    'SmootherSelector': '#334155',
    'GoalCheckerSelector': '#1e293b',

    // 条件节点 - 紫色系
    'GoalReached': '#a855f7',
    'GoalUpdated': '#9333ea',
    'GlobalUpdatedGoal': '#7c3aed',
    'IsBatteryLow': '#ec4899',
    'IsBatteryCharging': '#22c55e',
    'IsPathValid': '#a855f7',
    'IsStuck': '#ef4444',
    'IsStopped': '#64748b',
    'InitialPoseReceived': '#8b5cf6',
    'TimeExpired': '#f59e0b',
    'PathExpiringTimer': '#d97706',
    'DistanceTraveled': '#14b8a6',
    'TransformAvailable': '#06b6d4',
    'WouldAControllerRecoveryHelp': '#ec4899',
    'WouldAPlannerRecoveryHelp': '#db2777',
    'WouldASmootherRecoveryHelp': '#be185d',
    'WouldARouteRecoveryHelp': '#9d174d',
    'AreErrorCodesPresent': '#f43f5e',

    // 默认
    'default': '#64748b'
};

export class TreeVisualizer {
    constructor(containerId) {
        this.containerId = containerId;
        this.svg = null;
        this.g = null;
        this.zoom = null;
        this.width = 0;
        this.height = 0;

        // 节点尺寸 - 更大以显示更多信息
        this.nodeWidth = 200;
        this.nodeHeight = 60;
        this.horizontalSpacing = 30;
        this.verticalSpacing = 40;

        this.init();
    }

    init() {
        const container = document.getElementById(this.containerId);
        if (!container) return;

        const rect = container.getBoundingClientRect();
        this.width = rect.width || 800;
        this.height = rect.height || 600;

        container.innerHTML = '';

        this.svg = d3.select(`#${this.containerId}`)
            .append('svg')
            .attr('width', '100%')
            .attr('height', '100%')
            .style('display', 'block')
            .style('background', '#1e293b');

        this.addDefs();

        this.g = this.svg.append('g')
            .attr('class', 'tree-container');

        this.zoom = d3.zoom()
            .scaleExtent([0.1, 3])
            .on('zoom', (event) => {
                this.g.attr('transform', event.transform);
            });

        this.svg.call(this.zoom);

        this.showEmptyState();
    }

    addDefs() {
        const defs = this.svg.append('defs');

        // 阴影滤镜
        const filter = defs.append('filter')
            .attr('id', 'node-shadow')
            .attr('x', '-50%')
            .attr('y', '-50%')
            .attr('width', '200%')
            .attr('height', '200%');

        filter.append('feDropShadow')
            .attr('dx', 0)
            .attr('dy', 3)
            .attr('stdDeviation', 4)
            .attr('flood-color', 'rgba(0,0,0,0.5)')
            .attr('flood-opacity', 0.5);

        // 发光效果
        const glow = defs.append('filter')
            .attr('id', 'node-glow')
            .attr('x', '-50%')
            .attr('y', '-50%')
            .attr('width', '200%')
            .attr('height', '200%');

        glow.append('feGaussianBlur')
            .attr('stdDeviation', '3')
            .attr('result', 'coloredBlur');

        const feMerge = glow.append('feMerge');
        feMerge.append('feMergeNode').attr('in', 'coloredBlur');
        feMerge.append('feMergeNode').attr('in', 'SourceGraphic');
    }

    showEmptyState() {
        this.g.selectAll('*').remove();

        this.g.append('text')
            .attr('x', this.width / 2)
            .attr('y', this.height / 2)
            .attr('text-anchor', 'middle')
            .attr('dominant-baseline', 'middle')
            .attr('fill', '#64748b')
            .attr('font-size', '14px')
            .attr('font-family', 'Inter, system-ui, sans-serif')
            .text('Drag blocks to create behavior tree');
    }

    renderFromXml(xmlString) {
        if (!xmlString || !xmlString.trim()) {
            this.showEmptyState();
            return;
        }

        try {
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(xmlString, 'text/xml');

            const parseError = xmlDoc.querySelector('parsererror');
            if (parseError) {
                console.warn('XML parse error');
                this.showEmptyState();
                return;
            }

            const btRoot = xmlDoc.querySelector('BehaviorTree');
            if (!btRoot || !btRoot.firstElementChild) {
                this.showEmptyState();
                return;
            }

            const treeData = this.buildTreeData(btRoot.firstElementChild);
            this.renderTree(treeData);

        } catch (error) {
            console.error('Error parsing XML:', error);
            this.showEmptyState();
        }
    }

    buildTreeData(element, depth = 0) {
        const nodeName = element.tagName;
        const displayName = element.getAttribute('name') || nodeName;

        // 收集所有属性
        const attributes = {};
        for (const attr of element.attributes) {
            attributes[attr.name] = attr.value;
        }

        // 生成属性显示文本
        const attrList = [];
        for (const [key, value] of Object.entries(attributes)) {
            if (key !== 'name') {
                // 简化显示：只显示变量名或短数值
                let displayValue = value;
                if (value.startsWith('{') && value.endsWith('}')) {
                    displayValue = value; // 保留黑板变量格式
                } else if (value.length > 15) {
                    displayValue = value.substring(0, 12) + '...';
                }
                attrList.push(`${key}=${displayValue}`);
            }
        }

        const node = {
            id: `${nodeName}_${depth}_${Math.random().toString(36).substr(2, 9)}`,
            type: nodeName,
            name: displayName,
            attributes: attributes,
            attrText: attrList.slice(0, 3).join(', '), // 最多显示3个属性
            hasMoreAttrs: attrList.length > 3,
            children: []
        };

        for (const child of element.children) {
            node.children.push(this.buildTreeData(child, depth + 1));
        }

        return node;
    }

    renderTree(treeData) {
        this.g.selectAll('*').remove();

        const container = document.getElementById(this.containerId);
        if (container) {
            const rect = container.getBoundingClientRect();
            this.width = rect.width || 800;
            this.height = rect.height || 600;
        }

        const hierarchy = d3.hierarchy(treeData);

        // 树布局
        const treeLayout = d3.tree()
            .nodeSize([this.nodeWidth + this.horizontalSpacing, this.nodeHeight + this.verticalSpacing]);

        const root = treeLayout(hierarchy);

        // 计算边界
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        root.each(d => {
            minX = Math.min(minX, d.x);
            maxX = Math.max(maxX, d.x);
            minY = Math.min(minY, d.y);
            maxY = Math.max(maxY, d.y);
        });

        const margin = 80;
        const offsetX = -minX + margin + this.nodeWidth / 2;
        const offsetY = margin;

        // 绘制连接线 - 使用贝塞尔曲线
        const links = root.links();

        this.g.selectAll('.link')
            .data(links)
            .enter()
            .append('path')
            .attr('class', 'link')
            .attr('fill', 'none')
            .attr('stroke', '#475569')
            .attr('stroke-width', 2)
            .attr('stroke-linecap', 'round')
            .attr('d', d => {
                const sx = d.source.x + offsetX;
                const sy = d.source.y + offsetY + this.nodeHeight;
                const tx = d.target.x + offsetX;
                const ty = d.target.y + offsetY;

                // 平滑贝塞尔曲线
                const midY = (sy + ty) / 2;
                return `M${sx},${sy} C${sx},${midY} ${tx},${midY} ${tx},${ty}`;
            });

        // 绘制节点
        const nodes = this.g.selectAll('.node')
            .data(root.descendants())
            .enter()
            .append('g')
            .attr('class', 'node')
            .attr('transform', d => `translate(${d.x + offsetX - this.nodeWidth / 2}, ${d.y + offsetY})`);

        // 节点背景 - 圆角矩形
        nodes.append('rect')
            .attr('width', this.nodeWidth)
            .attr('height', this.nodeHeight)
            .attr('rx', 8)
            .attr('ry', 8)
            .attr('fill', d => nodeColors[d.data.type] || nodeColors['default'])
            .attr('filter', 'url(#node-shadow)')
            .style('cursor', 'pointer');

        // 节点类型标题 (完整名称)
        nodes.append('text')
            .attr('x', 10)
            .attr('y', 20)
            .attr('fill', 'white')
            .attr('font-size', '12px')
            .attr('font-weight', '600')
            .attr('font-family', 'Inter, system-ui, sans-serif')
            .text(d => d.data.type);

        // 节点名称 (如果不同于类型)
        nodes.append('text')
            .attr('x', 10)
            .attr('y', 36)
            .attr('fill', 'rgba(255,255,255,0.85)')
            .attr('font-size', '10px')
            .attr('font-family', 'Inter, system-ui, sans-serif')
            .text(d => {
                if (d.data.name === d.data.type) return '';
                const name = d.data.name;
                return name.length > 25 ? name.substring(0, 22) + '...' : name;
            });

        // 属性显示 (关键参数)
        nodes.append('text')
            .attr('x', 10)
            .attr('y', 52)
            .attr('fill', 'rgba(255,255,255,0.6)')
            .attr('font-size', '9px')
            .attr('font-family', 'JetBrains Mono, monospace')
            .text(d => {
                const text = d.data.attrText;
                if (!text) return '';
                return text.length > 30 ? text.substring(0, 27) + '...' : text;
            });

        // 右侧儿童数量指示器
        nodes.filter(d => d.data.children && d.data.children.length > 0)
            .append('circle')
            .attr('cx', this.nodeWidth - 15)
            .attr('cy', 15)
            .attr('r', 10)
            .attr('fill', 'rgba(0,0,0,0.3)');

        nodes.filter(d => d.data.children && d.data.children.length > 0)
            .append('text')
            .attr('x', this.nodeWidth - 15)
            .attr('y', 19)
            .attr('text-anchor', 'middle')
            .attr('fill', 'white')
            .attr('font-size', '10px')
            .attr('font-weight', '600')
            .text(d => d.data.children.length);

        // 悬停效果
        nodes.on('mouseover', function () {
            d3.select(this).select('rect')
                .transition()
                .duration(150)
                .attr('stroke', 'white')
                .attr('stroke-width', 2);
        }).on('mouseout', function () {
            d3.select(this).select('rect')
                .transition()
                .duration(150)
                .attr('stroke', 'none');
        });

        // 点击显示详情 (在控制台)
        nodes.on('click', (event, d) => {
            console.log('Node Details:', d.data);
        });

        this.fitToView(minX, maxX, minY, maxY);
    }

    fitToView(minX, maxX, minY, maxY) {
        const treeWidth = maxX - minX + this.nodeWidth + 160;
        const treeHeight = maxY - minY + this.nodeHeight + 160;

        const scale = Math.min(
            this.width / treeWidth,
            this.height / treeHeight,
            1.0
        ) * 0.85;

        const translateX = (this.width - treeWidth * scale) / 2 + 40;
        const translateY = 40;

        this.svg.transition()
            .duration(400)
            .call(
                this.zoom.transform,
                d3.zoomIdentity.translate(translateX, translateY).scale(scale)
            );
    }

    zoomIn() {
        this.svg.transition().duration(200).call(this.zoom.scaleBy, 1.3);
    }

    zoomOut() {
        this.svg.transition().duration(200).call(this.zoom.scaleBy, 0.7);
    }

    resetZoom() {
        this.svg.transition().duration(400).call(
            this.zoom.transform,
            d3.zoomIdentity.translate(40, 40).scale(0.7)
        );
    }

    resize() {
        const container = document.getElementById(this.containerId);
        if (!container) return;

        const rect = container.getBoundingClientRect();
        this.width = rect.width || 800;
        this.height = rect.height || 600;
    }
}

export default TreeVisualizer;

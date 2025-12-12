# Nav2 路网绘制与集成指南 (Route Network Guide)

本文档记录了基于 QGIS 绘制能够被 `nav2_route` 识别的路网的完整流程。

## 1. 准备工作 (QGIS 环境)

1.  **加载地图**: 将 `1126.pgm` 拖入 QGIS。
    *   *注意*: 系统已自动生成 `1126.pgw` (World File)，QGIS 会自动对齐地图坐标。
2.  **创建图层**:
    *   点击 `Layer` -> `Create Layer` -> `New GeoPackage Layer`...
    *   **Geometry type (几何类型)**: 选择 `LineString` (线)。
    *   **Fields (字段列表)**:
        *   `id`: 类型 `Integer` (必填，每条路唯一的编号)。
        *   `bidirectional`: 类型 `Integer` (选填，控制方向)。

## 2. 绘制路网 (Drawing)

1.  **开启吸附 (Snapping)**:
    *   **极度重要**: 点击工具栏上的磁铁图标 (Enable Snapping)。
    *   设置模式为 `Vertex` (顶点) 和 `Segment` (边)。
    *   *作用*: 保证你画的线在交叉口是真正“连在一起”的，否则机器人走不过去。
2.  **开始绘制**:
    *   点击铅笔图标 (Toggle Editing)。
    *   使用 `Add Line Feature` 工具画线。
    *   **关于方向**: 顺着你画线的方向（起点 -> 终点），这就是这条路的“正向”。

## 3. 设置属性 (Attributes)

画完每条线后，在弹出的属性框里填写：

*   **id**: 填一个唯一的数字 (0, 1, 2...)。
*   **bidirectional** (方向控制):
    *   **`1` (或不填)**: **双向通行**。脚本会自动生成一条反向的路。
    *   **`0`**: **单向通行**。机器人只能沿着你画线的方向走。

## 4. 导出数据 (Export)

1.  右键点击图层 -> `Export` -> `Save Features As...`
2.  **Format**: 选择 **`GeoJSON`**。
3.  **File name**: 保存为 `1126.geojson`。
4.  **Layer Options (关键设置)**:
    *   **`fid` (Feature ID)**: **取消勾选** (Do not export)。
        *   *原因*: QGIS 自动生成的 fid 会干扰 Route Server 的 ID 读取。
    *   **Coordinate Precision**: 建议设置 `10`。

## 5. 数据处理 (Processing)

我们需要运行一个脚本，把 QGIS 导出的“只有线”的文件，转换成 Nav2 需要的“有点有线”的拓扑图，并进行坐标转换。

**脚本路径**: `src/robot_route/maps/process_graph.py`

**运行命令**:
```bash
python3 src/robot_route/maps/process_graph.py
```

*脚本会自动完成以下工作：*
1.  **坐标转换**: 将 QGIS 的像素/大坐标转换为 Nav2 的米制地图坐标。
2.  **生成节点 (Nodes)**: 自动识别交叉点生成 Node。
3.  **生成拓扑 (Edges)**: 建立 Node 之间的连接关系。
4.  **处理双向**: 如果 `bidirectional` 为 1，自动生成反向边。

## 6. 启动与可视化 (Visualization)

**启动**:
```bash
ros2 launch robot_navigation navigation.launch.py
```
*(路网服务 `route_server` 会随导航自动启动)*

**在 RViz 中查看**:
1.  **Add**: 点击左下角 Add -> `By topic` -> 选择 `/route_graph` -> `MarkerArray`。
2.  **设置 QoS (重要)**:
    *   展开 `MarkerArray` -> `Topic`。
    *   将 **`Durability Policy`** 改为 **`Transient Local`**。
    *   *(如果不改这个，你可能因为启动晚了而看不到路网)*
3.  **显示内容**: 确保勾选了 `Namespaces` 下的 `edges`, `nodes`, `route_graph_ids`。

---
**常见问题排查**:
*   *看不到路网？* -> 检查 RViz 的 Durability Policy 是否为 Transient Local。
*   *路网飞了？* -> 检查是否运行了 `process_graph.py` 进行坐标转换。
*   *报错 ID 重复？* -> 检查导出时是否取消勾选了 `fid`。

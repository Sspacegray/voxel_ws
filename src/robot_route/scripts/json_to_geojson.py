#!/usr/bin/env python3
"""
JSON 路径文件 → GeoJSON 路网文件 转换工具

将 Waypoint Editor 保存的 JSON 路径转换为 nav2_route 需要的 GeoJSON 格式。

使用方法:
    python3 json_to_geojson.py input.json output.geojson [--bidirectional]

参数:
    input.json       : Waypoint Editor 保存的 JSON 路径文件
    output.geojson   : 输出的 GeoJSON 路网文件
    --bidirectional  : 可选，生成双向边（默认根据 JSON 中的 bidirectional 字段）
    --all-bidirectional : 强制所有边都为双向
"""

import json
import argparse
import sys
from pathlib import Path


def load_waypoints_from_json(json_path: str) -> list:
    """从 JSON 路径文件加载 waypoints，返回 [(x_m, y_m, bidirectional), ...]"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    if 'paths' not in data:
        raise ValueError("JSON 格式错误：缺少 'paths' 字段")
    
    waypoints = []
    paths = data['paths']
    
    for i, segment in enumerate(paths):
        # 获取起点坐标 (mm -> m)
        if 'start_point' in segment:
            x_mm = segment['start_point'].get('x', 0)
            y_mm = segment['start_point'].get('y', 0)
            x_m = x_mm / 1000.0
            y_m = y_mm / 1000.0
            
            # 获取 bidirectional 属性（默认 True）
            bidirectional = segment.get('bidirectional', True)
            
            waypoints.append((x_m, y_m, bidirectional))
        
        # 最后一个 segment 的终点也要加入
        if i == len(paths) - 1 and 'end_point' in segment:
            x_mm = segment['end_point'].get('x', 0)
            y_mm = segment['end_point'].get('y', 0)
            x_m = x_mm / 1000.0
            y_m = y_mm / 1000.0
            waypoints.append((x_m, y_m, True))  # 终点默认双向
    
    return waypoints


def create_geojson(waypoints: list, all_bidirectional: bool = False) -> dict:
    """
    将 waypoints 转换为 GeoJSON 格式
    
    Args:
        waypoints: [(x_m, y_m, bidirectional), ...]
        all_bidirectional: 强制所有边都为双向
    
    Returns:
        GeoJSON dict
    """
    features = []
    
    # 1. 创建 Node Features
    for i, (x, y, _) in enumerate(waypoints):
        node_feature = {
            "type": "Feature",
            "properties": {
                "id": i,
                "frame": "map"
            },
            "geometry": {
                "type": "Point",
                "coordinates": [x, y]
            }
        }
        features.append(node_feature)
    
    # 2. 创建 Edge Features
    edge_id = 0
    for i in range(len(waypoints) - 1):
        x1, y1, bidirectional = waypoints[i]
        x2, y2, _ = waypoints[i + 1]
        
        if all_bidirectional:
            bidirectional = True
        
        # 正向边 (i -> i+1)
        forward_edge = {
            "type": "Feature",
            "properties": {
                "id": edge_id,
                "startid": i,
                "endid": i + 1,
                "weight": 1.0
            },
            "geometry": {
                "type": "MultiLineString",
                "coordinates": [[[x1, y1], [x2, y2]]]
            }
        }
        features.append(forward_edge)
        edge_id += 1
        
        # 反向边 (i+1 -> i)，如果是双向
        if bidirectional:
            reverse_edge = {
                "type": "Feature",
                "properties": {
                    "id": edge_id,
                    "startid": i + 1,
                    "endid": i,
                    "weight": 1.0
                },
                "geometry": {
                    "type": "MultiLineString",
                    "coordinates": [[[x2, y2], [x1, y1]]]
                }
            }
            features.append(reverse_edge)
            edge_id += 1
    
    # 3. 组装 GeoJSON
    geojson = {
        "type": "FeatureCollection",
        "name": "graph",
        "crs": {
            "type": "name",
            "properties": {
                "name": "urn:ogc:def:crs:EPSG::3857"
            }
        },
        "features": features
    }
    
    return geojson


def main():
    parser = argparse.ArgumentParser(
        description='将 Waypoint Editor JSON 转换为 nav2_route GeoJSON'
    )
    parser.add_argument('input', help='输入 JSON 路径文件')
    parser.add_argument('output', help='输出 GeoJSON 文件')
    parser.add_argument('--all-bidirectional', action='store_true',
                        help='强制所有边都为双向')
    
    args = parser.parse_args()
    
    # 检查输入文件
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"错误：输入文件不存在 - {args.input}", file=sys.stderr)
        sys.exit(1)
    
    try:
        # 加载 waypoints
        waypoints = load_waypoints_from_json(args.input)
        print(f"加载了 {len(waypoints)} 个 waypoints")
        
        if len(waypoints) < 2:
            print("错误：至少需要 2 个 waypoints", file=sys.stderr)
            sys.exit(1)
        
        # 转换为 GeoJSON
        geojson = create_geojson(waypoints, args.all_bidirectional)
        
        # 统计边数
        edges = [f for f in geojson['features'] if f['geometry']['type'] == 'MultiLineString']
        nodes = [f for f in geojson['features'] if f['geometry']['type'] == 'Point']
        print(f"生成了 {len(nodes)} 个节点, {len(edges)} 条边")
        
        # 保存
        output_path = Path(args.output)
        with open(output_path, 'w') as f:
            json.dump(geojson, f, indent=4)
        
        print(f"GeoJSON 已保存到: {output_path}")
        
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()

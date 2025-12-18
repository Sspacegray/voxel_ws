import argparse
import json
import math
import os
import yaml

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_INPUT_FILE = os.path.join(_SCRIPT_DIR, '1126.geojson')
_DEFAULT_OUTPUT_FILE = os.path.join(_SCRIPT_DIR, '1126_processed.geojson')
_DEFAULT_MAP_YAML = os.path.join(_SCRIPT_DIR, '1126.yaml')


def _parse_args():
    parser = argparse.ArgumentParser(description='Process route geojson graph coordinates/topology.')
    parser.add_argument('--input', default=_DEFAULT_INPUT_FILE, help='Input geojson path')
    parser.add_argument('--output', default=_DEFAULT_OUTPUT_FILE, help='Output geojson path')
    parser.add_argument('--map_yaml', default=_DEFAULT_MAP_YAML, help='Map YAML file for parameters')
    parser.add_argument('--merge_threshold', type=float, default=0.1, help='Node merge distance threshold (m)')
    return parser.parse_args()

def load_map_params(yaml_path):
    """从地图YAML加载参数"""
    if not os.path.exists(yaml_path):
        print(f"警告: 地图YAML文件不存在: {yaml_path}")
        print("使用默认参数")
        return {
            'origin_x': -12.0,
            'origin_y': -6.14,
            'resolution': 0.05,
            'height_px': 887
        }
    
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        origin = data.get('origin', [-12.0, -6.14, 0.0])
        resolution = data.get('resolution', 0.05)
        
        # 从image字段获取图片文件名并读取高度
        image_file = data.get('image', '1126.pgm')
        image_path = os.path.join(os.path.dirname(yaml_path), image_file)
        
        height_px = 887  # 默认值
        if os.path.exists(image_path):
            # 简单读取PGM高度(P5格式第二行)
            try:
                with open(image_path, 'rb') as img:
                    img.readline()  # P5
                    size_line = img.readline().decode('ascii').strip()
                    if not size_line.startswith('#'):
                        width, height = map(int, size_line.split())
                        height_px = height
            except:
                pass
        
        params = {
            'origin_x': float(origin[0]),
            'origin_y': float(origin[1]),
            'resolution': float(resolution),
            'height_px': height_px
        }
        
        print(f"从YAML加载地图参数: {params}")
        return params
        
    except Exception as e:
        print(f"警告: 无法加载YAML参数: {e}")
        print("使用默认参数")
        return {
            'origin_x': -12.0,
            'origin_y': -6.14,
            'resolution': 0.05,
            'height_px': 887
        }

def get_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def find_node_id(point, nodes, threshold=0.1):
    for node in nodes:
        if get_dist(point, node['coords']) < threshold:
            return node['id']
    return None

def transform_point(pt, map_params):
    """使用地图参数转换点坐标"""
    x_px = pt[0]
    y_px = pt[1]  # negative
    
    origin_x = map_params['origin_x']
    origin_y = map_params['origin_y']
    resolution = map_params['resolution']
    height_px = map_params['height_px']
    
    # 计算top_y
    height_m = height_px * resolution
    top_y = origin_y + height_m
    
    new_x = origin_x + (x_px * resolution)
    new_y = top_y + (y_px * resolution)  # y_px is negative
    
    return [new_x, new_y]

def main():
    args = _parse_args()
    
    # 加载地图参数
    map_params = load_map_params(args.map_yaml)

    # 验证输入文件
    if not os.path.exists(args.input):
        print(f"错误: 输入文件不存在: {args.input}")
        return

    try:
        with open(args.input, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        print(f"错误: GeoJSON格式错误: {e}")
        return
    except Exception as e:
        print(f"错误: 无法读取输入文件: {e}")
        return

    # 验证GeoJSON结构
    if 'features' not in data:
        print("错误: GeoJSON缺少features字段")
        return

    edges = []
    raw_nodes = []  # list of [x, y]

    # 1. Parse all LineStrings (Edges) and collect (and transform) endpoints
    for i, feature in enumerate(data['features']):
        geom = feature.get('geometry', {})
        props = feature.get('properties', {})
        
        if not geom:
            print(f"警告: Feature {i} 缺少geometry,跳过")
            continue
        
        geom_type = geom.get('type')
        if geom_type == 'LineString' or geom_type == 'MultiLineString':
            coords = geom.get('coordinates', [])
            
            if not coords:
                print(f"警告: Feature {i} 缺少coordinates,跳过")
                continue
            
            # Handle MultiLineString (take first line)
            if geom_type == 'MultiLineString':
                if len(coords) == 0:
                    continue
                points = coords[0]
            else:
                points = coords
            
            # Transform all points in the line
            transformed_points = [transform_point(p, map_params) for p in points]
            
            start_pt = transformed_points[0]
            end_pt = transformed_points[-1]
            
            raw_nodes.append(start_pt)
            raw_nodes.append(end_pt)
            
            # Keep track of edge data
            edges.append({
                'feature': feature,
                'points': transformed_points,
                'props': props
            })

    if not edges:
        print("警告: 没有找到任何边,检查GeoJSON是否包含LineString特征")
        return

    # 2. Deduplicate Nodes
    unique_nodes = []
    node_id_counter = 0
    
    for pt in raw_nodes:
        if find_node_id(pt, unique_nodes, args.merge_threshold) is None:
            unique_nodes.append({
                'id': node_id_counter,
                'coords': pt
            })
            node_id_counter += 1

    # 3. Build Final Features List
    final_features = []

    # Add Node Features
    for node in unique_nodes:
        node_feature = {
            "type": "Feature",
            "properties": {
                "id": node['id'],
                "frame": "map"
            },
            "geometry": {
                "type": "Point",
                "coordinates": node['coords']
            }
        }
        final_features.append(node_feature)

    # Add Edge Features with topology
    for i, edge in enumerate(edges):
        start_pt = edge['points'][0]
        end_pt = edge['points'][-1]
        
        start_id = find_node_id(start_pt, unique_nodes, args.merge_threshold)
        end_id = find_node_id(end_pt, unique_nodes, args.merge_threshold)
        
        if start_id is None or end_id is None:
            print(f"警告: Edge {i} 的起点或终点无法匹配节点,跳过")
            continue
        
        # Ensure we have an ID for the edge
        edge_id = edge['props'].get('id')
        if edge_id is None:
            edge_id = i + 1000  # fallback ID
        
        # Construct Nav2 Route Edge Feature (Forward)
        edge_feature_fwd = {
            "type": "Feature",
            "properties": {
                "id": int(edge_id),
                "startid": start_id,
                "endid": end_id,
                "weight": 1.0
            },
            "geometry": {
                "type": "MultiLineString",
                "coordinates": [ edge['points'] ]
            }
        }
        final_features.append(edge_feature_fwd)

        # Check 'bidirectional' property from QGIS
        is_bidirectional = True
        bidir_val = edge['props'].get('bidirectional')
        if bidir_val is not None:
             if str(bidir_val).lower() in ['0', 'false', 'no', 'f']:
                 is_bidirectional = False

        if is_bidirectional:
            # Construct reverse edge with unique string ID
            rev_points = edge['points'][::-1]
            edge_feature_bwd = {
                "type": "Feature",
                "properties": {
                    "id": f"{edge_id}_reverse",  # ✅ 使用字符串避免ID冲突
                    "startid": end_id,
                    "endid": start_id,
                    "weight": 1.0
                },
                "geometry": {
                    "type": "MultiLineString",
                    "coordinates": [ rev_points ]
                }
            }
            final_features.append(edge_feature_bwd)

    # 4. Save
    output_data = {
        "type": "FeatureCollection",
        "name": "graph",
        "crs": { "type": "name", "properties": { "name": "urn:ogc:def:crs:EPSG::3857" } },
        "features": final_features
    }

    try:
        with open(args.output, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=4, ensure_ascii=False)
        
        print(f"✅ 路网处理成功!")
        print(f"生成 {len(unique_nodes)} 个节点和 {len(final_features) - len(unique_nodes)} 条边")
        print(f"保存到: {args.output}")
    except Exception as e:
        print(f"错误: 无法保存输出文件: {e}")

if __name__ == '__main__':
    main()

import json
import math

INPUT_FILE = '/home/suja/voxel_ws/src/robot_route/maps/1126.geojson'
OUTPUT_FILE = '/home/suja/voxel_ws/src/robot_route/maps/1126_processed.geojson'

def get_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def find_node_id(point, nodes, threshold=0.1):
    for node in nodes:
        if get_dist(point, node['coords']) < threshold:
            return node['id']
    return None

def transform_point(pt):
    # Transform from likely Pixel Space (QGIS default) to Map Frame (Meters)
    # Map Params from yaml:
    # Origin X (Left) = -12.0
    # Origin Y (Bottom) = -6.14
    # Resolution = 0.05
    # Height_px = 887 -> Height_m = 44.35
    # Top_Y = -6.14 + 44.35 = 38.21
    
    # Input pt[0] is X (approx 0..1021)
    # Input pt[1] is Y (approx -887..0)
    
    # We detected user coordinates are large (e.g. 273, -761), so they are likely pixels.
    # We apply the World File transform manually.
    
    x_px = pt[0]
    y_px = pt[1] # negative
    
    new_x = -12.0 + (x_px * 0.05)
    new_y = 38.21 + (y_px * 0.05) # y_px is negative, so this subtracts
    
    return [new_x, new_y]

def main():
    with open(INPUT_FILE, 'r') as f:
        data = json.load(f)

    edges = []
    raw_nodes = [] # list of [x, y]

    # 1. Parse all LineStrings (Edges) and collect (and transform) endpoints
    for feature in data['features']:
        geom = feature.get('geometry', {})
        props = feature.get('properties', {})
        
        if geom.get('type') == 'LineString' or geom.get('type') == 'MultiLineString':
            coords = geom['coordinates']
            # Handle MultiLineString (take first line)
            if geom['type'] == 'MultiLineString':
                points = coords[0]
            else:
                points = coords
            
            # Transform all points in the line
            transformed_points = [transform_point(p) for p in points]
            
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

    # 2. Deduplicate Nodes
    unique_nodes = []
    node_id_counter = 0
    
    for pt in raw_nodes:
        if find_node_id(pt, unique_nodes) is None:
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
        
        start_id = find_node_id(start_pt, unique_nodes)
        end_id = find_node_id(end_pt, unique_nodes)
        
        # Ensure we have an ID for the edge
        edge_id = edge['props'].get('id')
        if edge_id is None:
            edge_id = i + 1000 # fallback ID
        
        # Construct Nav2 Route Edge Feature
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
        # Default is True (1) unless specified as 0/"0"/"false"
        is_bidirectional = True
        bidir_val = edge['props'].get('bidirectional')
        if bidir_val is not None:
             if str(bidir_val).lower() in ['0', 'false', 'no', 'f']:
                 is_bidirectional = False

        if is_bidirectional:
            # Construct Nav2 Route Edge Feature (Backward)
            # Reverse points
            rev_points = edge['points'][::-1]
            edge_feature_bwd = {
                "type": "Feature",
                "properties": {
                    "id": int(edge_id) + 10000, # distinct ID for reverse edge
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

    with open(INPUT_FILE, 'w') as f: # Overwrite original
        json.dump(output_data, f, indent=4)
    
    print(f"Graph processed! Generated {len(unique_nodes)} nodes and {len(edges)} edges.")

if __name__ == '__main__':
    main()

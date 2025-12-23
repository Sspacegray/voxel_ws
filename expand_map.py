
import cv2
import numpy as np
import yaml
import os

map_dir = '/home/wu/voxel_ws/src/robot_simulation/wpr_simulation2/maps'
pgm_file = os.path.join(map_dir, 'my_map.pgm')
yaml_file = os.path.join(map_dir, 'my_map.yaml')

# Load image
# Read as grayscale
img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
if img is None:
    print(f"Failed to load {pgm_file}")
    exit(1)

height, width = img.shape
print(f"Original size: {width}x{height}")

# Padding configuration (in meters)
pad_meters = 4.0
resolution = 0.05
pad_pixels = int(pad_meters / resolution)

print(f"Padding: {pad_meters}m -> {pad_pixels} pixels")

# Create new image with padding
# Fill with 205 (unknown space gray)
new_height = height + 2 * pad_pixels
new_width = width + 2 * pad_pixels
new_img = np.full((new_height, new_width), 205, dtype=np.uint8)

# Calculate insert position (center)
start_x = pad_pixels
start_y = pad_pixels

# Copy old image to center
new_img[start_y:start_y+height, start_x:start_x+width] = img

# Save new image
new_pgm_file = os.path.join(map_dir, 'my_map_expanded.pgm')
cv2.imwrite(new_pgm_file, new_img)
print(f"Saved expanded map to {new_pgm_file}")

# Update YAML
with open(yaml_file, 'r') as f:
    map_config = yaml.safe_load(f)

old_origin = map_config['origin']
print(f"Old origin: {old_origin}")

# Adjust origin
# origin is [x, y, yaw]
# Moving the image right/down by padding means the origin (bottom-left) moves left/down in world coords
new_origin_x = old_origin[0] - pad_meters
new_origin_y = old_origin[1] - pad_meters
new_origin = [float(new_origin_x), float(new_origin_y), old_origin[2]]

print(f"New origin: {new_origin}")

# Update config
map_config['image'] = 'my_map_expanded.pgm'
map_config['origin'] = new_origin

new_yaml_file = os.path.join(map_dir, 'my_map_expanded.yaml')
with open(new_yaml_file, 'w') as f:
    yaml.dump(map_config, f, default_flow_style=None)

print(f"Saved new yaml to {new_yaml_file}")

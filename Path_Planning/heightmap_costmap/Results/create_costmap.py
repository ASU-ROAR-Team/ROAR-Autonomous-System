import numpy as np
import pandas as pd
import yaml
import os
import cv2

def load_parameters():
    """Load parameters from YAML file"""
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up one directory to reach the config folder
    config_dir = os.path.join(os.path.dirname(script_dir), 'config')
    param_file = os.path.join(config_dir, 'costmap_params.yaml')
    
    with open(param_file, 'r') as file:
        params = yaml.safe_load(file)
    return params

def main():
    # Load parameters
    params = load_parameters()
    costmap_params = params['costmap_parameters']
    io_params = params['input_output']

    # Read the path
    df = pd.read_csv(io_params['input_path'])
    path_points = df.values

    # Define costmap parameters from YAML
    resolution = costmap_params['resolution']
    margin = costmap_params['margin']
    path_width = costmap_params['path_width']
    gradient_width = costmap_params['gradient_width']
    max_cost = costmap_params['max_cost']
    min_cost = costmap_params['min_cost']
    gradient_cost = costmap_params['gradient_cost']

    # Calculate bounds
    min_x = min(path_points[:, 0]) - margin
    max_x = max(path_points[:, 0]) + margin
    min_y = min(path_points[:, 1]) - margin
    max_y = max(path_points[:, 1]) + margin

    # Calculate dimensions
    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)

    # Create blank costmap (all max cost)
    costmap = np.ones((height, width), dtype=np.uint8) * max_cost

    # Convert path points to pixel coordinates
    pixel_path = []
    for x, y in path_points:
        px = int((x - min_x) / resolution)
        py = int((y - min_y) / resolution)
        # Flip y to match image coordinates (optional, depending on your convention)
        py = height - 1 - py
        pixel_path.append([px, py])
    pixel_path = np.array(pixel_path, dtype=np.int32)

    # Draw the path as a polyline
    cv2.polylines(costmap, [pixel_path], isClosed=False, color=min_cost, thickness=int(path_width / resolution))

    # Create a binary mask for the path
    path_mask = (costmap == min_cost).astype(np.uint8)

    # Compute distance transform for the gradient
    dist = cv2.distanceTransform(255 - path_mask * 255, cv2.DIST_L2, 5)
    dist = dist * resolution  # Convert to meters

    # Apply gradient cost
    for i in range(height):
        for j in range(width):
            if path_mask[i, j] == 0:
                if dist[i, j] <= gradient_width:
                    costmap[i, j] = int(gradient_cost * (dist[i, j] / gradient_width))
                else:
                    costmap[i, j] = max_cost

    # Convert to DataFrame and save as CSV
    costmap_df = pd.DataFrame(costmap)
    costmap_df.to_csv(io_params['output_costmap'], index=False, header=False)

    # Save costmap parameters
    with open(io_params['output_params'], 'w') as f:
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin_x: {min_x}\n")
        f.write(f"origin_y: {min_y}\n")
        f.write(f"width: {width}\n")
        f.write(f"height: {height}\n")

    print(f"Costmap created with dimensions: {width}x{height} pixels")
    print(f"Physical dimensions: {width*resolution:.1f}m x {height*resolution:.1f}m")
    print(f"Origin at: ({min_x:.1f}, {min_y:.1f})")

if __name__ == "__main__":
    main() 
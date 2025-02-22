#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
import os  # Added to get the current working directory

class HeightmapConverter:
    def __init__(self):
        rospy.init_node('heightmap_to_costmap')

        # **Load Parameters**
        image_path = rospy.get_param('~image_path', 'heightmap.png')
        resolution = rospy.get_param('~resolution', 0.0586901)  # meters/pixel
        origin_x = rospy.get_param('~origin_x', 0.0)    # Map origin (meters)
        origin_y = rospy.get_param('~origin_y', 0.0)
        gradient_scale = rospy.get_param('~gradient_scale', 150.0)  # Slope penalty
        stability_scale = rospy.get_param('~stability_scale', 90.0) # Terrain change penalty

        # **Load PNG Heightmap (Grayscale)**
        heightmap = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if heightmap is None:
            rospy.logerr(f"Failed to load image: {image_path}")
            return

        # Mask out white background (pixel > 250)
        mask = (heightmap > 250)
        heightmap = heightmap.astype(np.float32)
        heightmap[mask] = np.nan  # Mark as unknown

        # **Compute Gradients (Slope)**
        grad_x = cv2.Sobel(heightmap, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(heightmap, cv2.CV_32F, 0, 1, ksize=3)
        gradient_mag = np.sqrt(grad_x**2 + grad_y**2)

        # **Compute Terrain Stability (Sudden Changes)**
        laplacian = cv2.Laplacian(heightmap, cv2.CV_32F, ksize=3)
        stability = np.abs(laplacian)

        # **Compute Total Cost**
        gradient_cost = (gradient_mag / 255.0) * gradient_scale
        stability_cost = (stability / 255.0) * stability_scale
        total_cost = gradient_cost + stability_cost
        total_cost = np.clip(total_cost, 0, 100)
        total_cost[np.isnan(total_cost)] = -1
        total_cost = total_cost.astype(np.int8)

        # **Compute Directional Costs (cost_x and cost_y)**
        gradient_cost_x = (np.abs(grad_x) / 255.0) * gradient_scale
        gradient_cost_y = (np.abs(grad_y) / 255.0) * gradient_scale

        cost_x = gradient_cost_x + stability_cost
        cost_x = np.clip(cost_x, 0, 100)
        cost_x[np.isnan(cost_x)] = -1
        cost_x = cost_x.astype(np.int8)

        cost_y = gradient_cost_y + stability_cost
        cost_y = np.clip(cost_y, 0, 100)
        cost_y[np.isnan(cost_y)] = -1
        cost_y = cost_y.astype(np.int8)

        # **Define Function to Save Costs with Pixel Indices**
        def save_cost_with_indices(filename, cost_grid):
            height, width = cost_grid.shape
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write header: empty cell followed by x-axis indices
                header = [''] + [str(i) for i in range(width)]
                writer.writerow(header)
                # Write rows: y-axis index followed by cost values
                for y in range(height):
                    row = [str(y)] + [str(int(cost)) for cost in cost_grid[y]]
                    writer.writerow(row)

        # **Save All Three Costs to CSV Files**
        save_cost_with_indices('cost_x.csv', cost_x)
        save_cost_with_indices('cost_y.csv', cost_y)
        save_cost_with_indices('total_cost.csv', total_cost)  # Added for total_cost

        # **Log the Save Location**
        current_dir = os.getcwd()
        rospy.loginfo(f"Saved cost_x.csv, cost_y.csv, and total_cost.csv in {current_dir}")

        # **Visualize All Three Cost Maps**
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        cmap = plt.cm.hot
        cmap.set_under('gray')  # Color for invalid values (-1)

        # Plot total_cost
        axes[0].imshow(total_cost, cmap=cmap, vmin=0, vmax=100)
        axes[0].set_title('Total Cost')

        # Plot cost_x
        axes[1].imshow(cost_x, cmap=cmap, vmin=0, vmax=100)
        axes[1].set_title('Cost X')

        # Plot cost_y
        axes[2].imshow(cost_y, cmap=cmap, vmin=0, vmax=100)
        axes[2].set_title('Cost Y')

        # Add a shared colorbar
        fig.colorbar(axes[0].images[0], ax=axes, orientation='vertical', fraction=0.05, pad=0.05)

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        HeightmapConverter()
    except rospy.ROSInterruptException:
        pass
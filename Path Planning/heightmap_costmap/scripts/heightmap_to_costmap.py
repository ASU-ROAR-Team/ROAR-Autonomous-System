#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid

class HeightmapConverter:
    def __init__(self):
        rospy.init_node('heightmap_to_costmap')
        
        # Parameters
        image_path = rospy.get_param('~image_path', 'heightmap.png')
        resolution = rospy.get_param('~resolution', 0.1)  # meters/pixel
        origin_x = rospy.get_param('~origin_x', -5.0)     # Map origin (meters)
        origin_y = rospy.get_param('~origin_y', -5.0)
        gradient_scale = rospy.get_param('~gradient_scale', 50.0)  # Slope penalty
        stability_scale = rospy.get_param('~stability_scale', 30.0) # Terrain change penalty

        # Load PNG heightmap (grayscale)
        heightmap = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if heightmap is None:
            rospy.logerr(f"Failed to load image: {image_path}")
            return

        # Mask out white background (pixel>250)
        mask = (heightmap > 250)
        heightmap = heightmap.astype(np.float32)
        heightmap[mask] = np.nan  # Mark as unknown

        # Compute gradients (slope)
        grad_x = cv2.Sobel(heightmap, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(heightmap, cv2.CV_32F, 0, 1, ksize=3)
        gradient_mag = np.sqrt(grad_x**2 + grad_y**2)

        # Compute terrain stability (sudden changes)
        laplacian = cv2.Laplacian(heightmap, cv2.CV_32F, ksize=3)
        stability = np.abs(laplacian)

        # Normalize and combine costs
        gradient_cost = (gradient_mag / 255.0) * gradient_scale  # 0-50
        stability_cost = (stability / 255.0) * stability_scale    # 0-30
        total_cost = np.clip(gradient_cost + stability_cost, 0, 100).astype(np.int8)

        # Mark white areas as unknown (-1)
        total_cost[mask] = -1

        # Publish as global costmap
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = resolution
        grid_msg.info.width = heightmap.shape[1]
        grid_msg.info.height = heightmap.shape[0]
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = total_cost.flatten().tolist()
        
        pub = rospy.Publisher('/global_costmap', OccupancyGrid, queue_size=1, latch=True)
        pub.publish(grid_msg)
        rospy.loginfo("Global costmap published!")

if __name__ == '__main__':
    HeightmapConverter()
    rospy.spin()
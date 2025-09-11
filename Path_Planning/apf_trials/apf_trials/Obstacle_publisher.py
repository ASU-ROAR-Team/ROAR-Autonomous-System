#!/usr/bin/env python3
"""
This file has a obstacle publisher used for placing obstacles
in the path of the rover
"""
import rclpy
from rclpy.node import Node
from roar_msgs.msg import Obstacle
from std_msgs.msg import Header, Float32, Int32
from geometry_msgs.msg import PoseStamped


class ObstaclePublisher(Node):
    """
    Publishes different obstacles
    """
    
    def __init__(self):
        super().__init__("obstacle_publisher")
        
        # Declare parameters with default values
        self.declare_parameter("obstacle_x", 0.0)
        self.declare_parameter("obstacle_y", 13.0)
        self.declare_parameter("obstacle_radius", 0.5)
        self.declare_parameter("obstacle_id", 1)
        
        # Create publisher
        self.pub = self.create_publisher(Obstacle, "obstacle_topic", 10)
        
        # Create timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_obstacle)
        
        self.get_logger().info("Obstacle Publisher Node Started")
    
    def publish_obstacle(self):
        # Fetch updated values from parameters
        x = self.get_parameter("obstacle_x").get_parameter_value().double_value
        y = self.get_parameter("obstacle_y").get_parameter_value().double_value
        radius = self.get_parameter("obstacle_radius").get_parameter_value().double_value
        obstacleId = self.get_parameter("obstacle_id").get_parameter_value().integer_value

        msg = Obstacle()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.position = PoseStamped()
        msg.position.header = msg.header
        msg.position.pose.position.x = x
        msg.position.pose.position.y = y
        msg.position.pose.position.z = 0.0

        msg.radius = Float32(data=radius)
        msg.id = Int32(data=obstacleId)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        obstacle_publisher = ObstaclePublisher()
        rclpy.spin(obstacle_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
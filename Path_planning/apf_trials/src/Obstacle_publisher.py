#!/usr/bin/env python3
"""
This file has a obstacle publisher used for placing obstacles
in the path of the rover
"""
import rospy
from roar_msgs.msg import Obstacle
from std_msgs.msg import Header, Float32, Int32
from geometry_msgs.msg import PoseStamped


def obstaclePublisher() -> None:
    """
    Publishes different obstacles
    """
    rospy.init_node("obstacle_publisher", anonymous=True)
    pub = rospy.Publisher("obstacle_topic", Obstacle, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    # Set initial parameters
    rospy.set_param("obstacle_x", 0.0)
    rospy.set_param("obstacle_y", 13.0)
    rospy.set_param("obstacle_radius", 0.5)
    rospy.set_param("obstacle_id", 1)

    while not rospy.is_shutdown():
        # Fetch updated values from ROS parameters
        x = rospy.get_param("obstacle_x")
        y = rospy.get_param("obstacle_y")
        radius = rospy.get_param("obstacle_radius")
        obstacleId = rospy.get_param("obstacle_id")

        msg = Obstacle()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.position = PoseStamped()
        msg.position.header = msg.header
        msg.position.pose.position.x = x
        msg.position.pose.position.y = y
        msg.position.pose.position.z = 0.0

        msg.radius = Float32(radius)
        msg.id = Int32(obstacleId)

        pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        obstaclePublisher()
    except rospy.ROSInterruptException:
        pass

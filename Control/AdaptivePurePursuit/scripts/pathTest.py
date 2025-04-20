#!/usr/bin/env python3

# pylint: disable=all
# mypy: ignore-errors

import rospy
from turtlebot3_msgs.msg import wp_list  # Import your custom message
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

DS = 0.05
R = 1
LH = 5  ##horizontal distance
LV = 1  ##vertical distance
x_way_points_list = []
y_way_points_list = []


def generate_track():
    global x_way_points_list
    global y_way_points_list
    y_l1 = np.linspace(0, LH, int(LH / DS))
    x_l1 = np.zeros(len(y_l1))
    theta = np.linspace(0, np.pi, int((np.pi * R) / DS))
    x_l2 = R - R * np.cos(theta)
    y_l2 = R * np.sin(theta) + LH
    y_l3 = np.linspace(LH, 0, int(LH / DS))
    x_l3 = np.full(len(y_l3), 2 * R)
    x_l4 = np.linspace(2 * R, 2 * R + LV, int(5 / DS))
    y_l4 = np.zeros(len(x_l4))
    x_way_points_list = np.concatenate((x_l1, x_l2, x_l3, x_l4))
    y_way_points_list = np.concatenate((y_l1, y_l2, y_l3, y_l4))
    print(
        f"x_1: {len(x_l1)}, len y_1: {len(y_l1)}\n x_1: {len(x_l2)}, len y_1: {len(y_l2)}\n x_1: {len(x_l3)}, len y_1: {len(y_l3)}\n x_1: {len(x_l4)}, len y_1: {len(y_l4)}\n "
    )
    plt.plot(x_way_points_list, y_way_points_list)
    plt.show()


def path_publisher():
    generate_track()
    rospy.init_node("tuple_list_publisher", anonymous=True)
    pub = rospy.Publisher("/Path", Path, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz
    global x_way_points_list
    global y_way_points_list
    msg = Path()
    for i in range(len(x_way_points_list)):
        point = PoseStamped()
        point.pose.position.x = x_way_points_list[i]
        point.pose.position.y = y_way_points_list[i]
        msg.poses.append(point)
    while not rospy.is_shutdown():
        # Publish the message
        pub.publish(msg)
        rospy.loginfo("Published a path")
        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("tuple_list_publisher", anonymous=True)
        # rospy.on_shutdown(plt.show)
        while not rospy.is_shutdown():
            path_publisher()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

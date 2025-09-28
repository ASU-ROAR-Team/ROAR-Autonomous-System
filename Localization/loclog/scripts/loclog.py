#!/usr/bin/env python

import rospy
import csv
import time
from nav_msgs.msg import Odometry

# Open CSV file in append mode
csv_file = open("filtered_state_xy.csv", mode="a")
csv_writer = csv.writer(csv_file)

# Write header only once if file is empty
if csv_file.tell() == 0:
    csv_writer.writerow(["time", "x", "y"])

last_saved_time = 0.0  # track last save time (wall clock)

def callback(msg):
    global last_saved_time

    # Current wall-clock time
    now = time.time()

    # Save only if 0.5s have passed since last save
    if now - last_saved_time >= 0.5:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        csv_writer.writerow([rospy.get_time(), x, y])  # use ROS time for consistency
        csv_file.flush()  # write immediately
        rospy.loginfo("Saved: t=%.2f x=%.3f, y=%.3f" % (rospy.get_time(), x, y))

        last_saved_time = now

def listener():
    rospy.init_node("filtered_state_logger", anonymous=True)
    rospy.Subscriber("/filtered_state", Odometry, callback)
    rospy.spin()
    csv_file.close()

if __name__ == "__main__":
    listener()

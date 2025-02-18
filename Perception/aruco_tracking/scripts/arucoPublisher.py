#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from roar_msgs.msg import Landmark
from roar_msgs.msg import LandmarkArray

def publisher():
    rospy.init_node('landmark_publisher', anonymous=True)
    pub = rospy.Publisher('landmark_topic', LandmarkArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # Create PoseStamped for each landmark
        pose1 = PoseStamped()
        pose1.header.stamp = rospy.Time.now()
        pose1.header.frame_id = "map"
        pose1.pose.position.x = 1.2
        pose1.pose.position.y = 2.3
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.w = 1.0
        
        pose2 = PoseStamped()
        pose2.header.stamp = rospy.Time.now()
        pose2.header.frame_id = "map"
        pose2.pose.position.x = -0.5
        pose2.pose.position.y = 1.8
        pose2.pose.position.z = 0.0
        pose2.pose.orientation.w = 1.0

        # Create Landmark messages
        landmark1 = Landmark()
        landmark1.header = Header()
        landmark1.header.stamp = rospy.Time.now()
        landmark1.header.frame_id = "map"
        landmark1.id = 1
        landmark1.pose = pose1
        
        landmark2 = Landmark()
        landmark2.header = Header()
        landmark2.header.stamp = rospy.Time.now()
        landmark2.header.frame_id = "map"
        landmark2.id = 2
        landmark2.pose = pose2
        
        # Create LandmarkArray
        landmark_array = LandmarkArray()
        landmark_array.landmarks = [landmark1, landmark2]
        
        rospy.loginfo("Publishing LandmarkArray: %s", landmark_array)
        pub.publish(landmark_array)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from cv_bridge.boost import create_images
from roar_msgs import LandmarkArray
from roar_msgs import Landmark

def publisher():
    rospy.init_node('landmark_publisher', anonymous=True)
    pub = rospy.Publisher('landmark_topic', LandmarkArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        landmark1 = Landmark(name="Tree", x=1.2, y=2.3, z=0.0, confidence=0.9)
        landmark2 = Landmark(name="Rock", x=-0.5, y=1.8, z=0.0, confidence=0.8)
        
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
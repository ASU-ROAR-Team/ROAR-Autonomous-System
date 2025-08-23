#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist:
    """
    ROS node to convert joystick commands (Joy messages) into
    robot velocity commands (Twist messages).
    """

    def __init__(self):
        """
        Initializes the node, publishers, subscribers, and parameters.
        """
        # Initialize the ROS node
        rospy.init_node('joy_to_twist_node', anonymous=True)
        
        rospy.loginfo("Joy to Twist Node is starting...")

        # Load ROS parameters for configuration. These can be set via a launch file
        # or on the command line.
        self.linear_speed_max = rospy.get_param('~linear_speed', 0.5)  # Max forward/backward speed (m/s)
        self.angular_speed_max = rospy.get_param('~angular_speed', 1.0) # Max angular speed (rad/s)
        self.linear_axis = rospy.get_param('~linear_axis', 1)  # Index of the joystick axis for linear control
        self.angular_axis = rospy.get_param('~angular_axis', 0)  # Index of the joystick axis for angular control

        # Create a publisher for the Twist message on the /cmd_vel topic
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to the Joy message on the /joy topic
        rospy.Subscriber('/joystick/raw_input', Joy, self.joy_callback)

        rospy.loginfo("Joy to Twist Node has started and is ready to receive commands.")

    def joy_callback(self, data: Joy):
        """
        Callback function for the Joy subscriber. It processes the joystick
        data and publishes a Twist message.
        """
        # Create a new Twist message
        twist_msg = Twist()

        # Get the linear velocity from the specified linear axis.
        # The value is multiplied by the max linear speed to scale it
        # from the joystick's [-1.0, 1.0] range to the desired speed range.
        twist_msg.linear.x = data.axes[self.linear_axis] * self.linear_speed_max
        
        # Get the angular velocity from the specified angular axis.
        # The value is multiplied by the max angular speed to scale it.
        twist_msg.angular.z = data.axes[self.angular_axis] * self.angular_speed_max

        # Log the velocities for debugging
        # rospy.loginfo(f"Publishing Twist: linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}")

        # Publish the constructed Twist message
        self.twist_pub.publish(twist_msg)

    def run(self):
        """
        Main loop for the node.
        """
        # Spin keeps the node alive, processing callbacks.
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of the class and run it
        joy_node = JoyToTwist()
        joy_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joy to Twist Node has been shut down.")

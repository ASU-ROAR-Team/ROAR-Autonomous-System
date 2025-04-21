#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Control action parser module for encoding ROS keyboard twist teleop commands to CAN frames.
This module provides a parser for encoding keyboard control commands into CAN frames.
It is used for sending control commands to test the rover via CAN bus.
"""
from typing import Dict, List, Optional
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from math import pi
import rospy


class KeyboardControlParser:  # pylint: disable=too-few-public-methods
    """Parser for encoding keyboard control commands into CAN frames.

    This module provides a parser for encoding keyboard control commands into CAN frames.
    It is used for sending control commands to test the rover via CAN bus.

    Methods
    -------
    parse(msg: Twist) -> Frame:
        Parse a ROS keyboard twist teleop command into a CAN frame.
    """

    def parse(self, msg: Twist) -> Frame:
        """
        Parse a ROS keyboard twist teleop command into a CAN frame.
        Parameters
        ----------
        msg : Twist
            ROS keyboard twist teleop command message.
        Returns
        -------
        List[int]
            Encoded CAN frame data.
        Raises
        ------
        Exception
            If an error occurs during encoding.

        """
        try:
            # Extract linear and angular velocities from the Twist message
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z

            # Convert velocities to motor RPMs using differential drive kinematics and limit to -10 to 10
            left_motor_rpm = int((linear_velocity - angular_velocity) * 10)
            right_motor_rpm = int((linear_velocity + angular_velocity) * 10)
            left_motor_rpm = max(-10, min(10, left_motor_rpm))
            right_motor_rpm = max(-10, min(10, right_motor_rpm))
            # Create the CAN frame data
            frame_data = [
                int(abs(left_motor_rpm * 2**4)),
                int(abs(left_motor_rpm * 2**4)),
                int(abs(left_motor_rpm * 2**4)),
                int(abs(right_motor_rpm * 2**4)),
                int(abs(right_motor_rpm * 2**4)),
                int(abs(right_motor_rpm * 2**4)),
                (left_motor_rpm < 0) * 0b11100000 | (right_motor_rpm < 0) * 0b00011100,
                0,
            ]

            # Create the CAN frame
            can_frame = Frame()
            can_frame.header.stamp = rospy.Time.now()
            can_frame.header.frame_id = "keyboard_control"
            can_frame.is_rtr = False
            can_frame.is_extended = False
            can_frame.is_error = False
            can_frame.id = 0x6A5
            can_frame.dlc = len(frame_data)
            can_frame.data = frame_data
            return can_frame

        except IndexError as e:
            print(f"Error parsing Testing frame: {e}")
            return None

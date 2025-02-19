#!/usr/bin/env python3

"""
ROS-CAN bridge node.
"""
from typing import Dict
from can_msgs.msg import Frame
from sensor_msgs.msg import Imu, NavSatFix
from roar_msgs.msg import EncoderStamped
from roscan import ImuParser, GpsParser, EncoderParser
import rospy


class RosCanBridge:
    """Main class for the ROS-CAN bridge.

    Attributes
    ----------
    imuPub : rospy.Publisher
        ROS publisher for IMU sensor data.
    gpsPub : rospy.Publisher
        ROS publisher for GPS sensor data.
    encoderPub : rospy.Publisher
        ROS publisher for encoder data.
    parsers : Dict[int, CanFrameParser]
        Dictionary of CAN frame parsers.
    canSub : rospy.Subscriber
        ROS subscriber for CAN frames.

    Methods
    -------
    canFrameCallback(msg: Frame)
        Callback for processing CAN frames.
    publishSensorData(frameId: int, data: Dict[str, float])
        Publish sensor data based on the CAN frame ID.
    """

    def __init__(self) -> None:
        # Initialize ROS publishers
        self.imuPub = rospy.Publisher("imuData", Imu, queue_size=10)
        self.gpsPub = rospy.Publisher("gpsData", NavSatFix, queue_size=10)
        self.encoderPub = rospy.Publisher("encoderData", EncoderStamped, queue_size=10)

        # Initialize CAN frame parsers
        self.parsers = {
            0x001: EncoderParser(),
            0x002: ImuParser(),
            0x003: GpsParser(),
            # Add more parsers as needed
        }

        # Subscribe to CAN frames from socketcan_bridge
        self.canSub = rospy.Subscriber("receivedMessages", Frame, self.canFrameCallback)

    def canFrameCallback(self, msg: Frame) -> None:
        """Callback for processing CAN frames
        Parameters
        ----------
        msg : Frame
            Received CAN frame.

        Returns
        -------
        None
        """
        if msg.id in self.parsers:
            parser = self.parsers[msg.id]
            parsedData = parser.parse(msg.data)
            if parsedData:
                self.publishSensorData(msg.id, parsedData)
            else:
                rospy.logerr(f"Failed to parse CAN frame with ID {hex(msg.id)}")

    def publishSensorData(self, frameId: int, data: Dict[str, float]) -> None:
        """Publish sensor data based on the CAN frame ID

        Parameters
        ----------
        frameId : int
            CAN frame ID.
        data : Dict[str, float]
            Parsed sensor data.

        Returns
        -------
        None
        """

        if frameId == 0x002:  # IMU
            imuMsg = Imu()
            imuMsg.header.stamp = rospy.Time.now()
            imuMsg.linear_acceleration.x = data.get("linearAccelerationX", 0.0)
            imuMsg.linear_acceleration.y = data.get("linearAccelerationY", 0.0)
            imuMsg.angular_velocity.z = data.get("yawRate", 0.0)
            imuMsg.orientation.z = data.get("yawAngle", 0.0)
            self.imuPub.publish(imuMsg)
        elif frameId == 0x003:  # GPS
            gpsMsg = NavSatFix()
            gpsMsg.header.stamp = rospy.Time.now()
            gpsMsg.latitude = data.get("latitude", 0.0)
            gpsMsg.longitude = data.get("longitude", 0.0)
            self.gpsPub.publish(gpsMsg)
        elif frameId == 0x001:  # encoder
            encoderMsg = EncoderStamped()
            encoderMsg.header.stamp = rospy.Time.now()
            encoderMsg.data[0] = data.get("encoder1", 0.0)
            encoderMsg.data[1] = data.get("encoder2", 0.0)
            encoderMsg.data[2] = data.get("encoder3", 0.0)
            encoderMsg.data[3] = data.get("encoder4", 0.0)
            encoderMsg.data[4] = data.get("encoder5", 0.0)
            encoderMsg.data[5] = data.get("encoder6", 0.0)
            self.encoderPub.publish(encoderMsg)


if __name__ == "__main__":
    rospy.init_node("rosCanBridge")
    bridge = RosCanBridge()
    rospy.spin()

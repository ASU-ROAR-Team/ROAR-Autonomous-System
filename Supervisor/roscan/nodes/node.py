#!/usr/bin/env python3
"""
ROS-CAN bridge node using python-can directly.
"""
from typing import Dict, Optional
import can
import threading
import rospy
from can_msgs.msg import Frame # Still used for keyboardControlCallback's output
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from roar_msgs.msg import EncoderStamped
from roscan import ImuParser, GpsParser, EncoderParser, TestParser, KeyboardControlParser
from geometry_msgs.msg import Twist


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
    can_bus : can.interface.Bus
        The python-can bus object for direct CAN communication.
    can_rx_thread : threading.Thread
        Thread for receiving CAN frames.
    running : bool
        Flag to control the CAN reception thread.

    Methods
    -------
    can_rx_loop()
        Main loop for receiving CAN frames in a separate thread.
    process_can_frame(msg: can.Message)
        Processes a received python-can Message.
    publishSensorData(frameId: int, data: Dict[str, float])
        Publish sensor data based on the CAN frame ID.
    """

    def __init__(self) -> None:
        rospy.init_node("rosCanBridge")
	#rospy.loginfo("Initializing RosCanBridge...")
        self.rate = rospy.Rate(10)

        # Initialize ROS publishers
        self.imuPub = rospy.Publisher("imuData", Imu, queue_size=10)
        self.gpsPub = rospy.Publisher("gpsData", NavSatFix, queue_size=10)
        self.encoderPub = rospy.Publisher("encoderData", EncoderStamped, queue_size=10)
        self.testPub = rospy.Publisher("testCanData", String, queue_size=10)
        self.heartbeatPub = rospy.Publisher("/system/heartbeat", String, queue_size=10)

        # Initialize CAN frame parsers
        self.parsers = {
            0x001: EncoderParser(),
            0x002: ImuParser(),
            0x003: GpsParser(),
            0x6A5: TestParser()
            # Add more parsers as needed
        }

        # Setup direct CAN communication
        try:
            self.can_bus = can.interface.Bus('can1', bustype='socketcan')
            rospy.loginfo(f"Successfully connected to CAN device: {self.can_bus.channel_info}")
        except Exception as e:
            rospy.logfatal(f"Failed to connect to CAN device 'can0': {e}")
            rospy.signal_shutdown(f"CAN bus connection failed: {e}")
            return

        # Start CAN reception thread
        self.running = True
        self.can_rx_thread = threading.Thread(target=self.can_rx_loop)
        self.can_rx_thread.daemon = True # Allow main program to exit even if thread is running
        self.can_rx_thread.start()
        rospy.loginfo("CAN reception thread started.")

        # Subscribe to ROS keyboard control messages (for sending CAN frames)
        self.keyboardControlSub = rospy.Subscriber("/cmd_vel", Twist, self.keyboardControlCallback)
        rospy.loginfo("RosCanBridge initialized.")

    def can_rx_loop(self) -> None:
        """
        Main loop for receiving CAN frames. Runs in a separate thread.
        Continuously listens for messages on the CAN bus.
        """
        rospy.loginfo("CAN reception loop started.")
        while self.running and not rospy.is_shutdown():
            try:
                msg: Optional[can.Message] = self.can_bus.recv(timeout=0.1) # Timeout for graceful shutdown check
                if msg is not None:
                    self.process_can_frame(msg)
            except can.CanError as e:
                rospy.logerr(f"CAN reception error: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error in CAN reception loop: {e}")
        rospy.loginfo("CAN reception loop stopped.")


    def process_can_frame(self, msg: can.Message) -> None:
        """
        Processes a received python-can Message.
        This replaces the canFrameCallback that was subscribed to /recieved_messages.
        """
        # Convert python-can Message ID to standard int for parser lookup
        frame_id = msg.arbitration_id
        
        # Check if it's a remote frame (RTR) - we usually don't process these as data
        if msg.is_remote_frame:
            rospy.logdebug(f"Received remote frame with ID: {hex(frame_id)}. Ignoring.")
            return

        # Check if it's an extended ID (if your parsers expect standard 11-bit IDs, adjust logic)
        # Your parsers use 0x... format, which implies 11-bit standard IDs, but it's good to be aware.
        if msg.is_extended_id:
            rospy.logwarn(f"Received extended CAN frame with ID: {hex(frame_id)}. Ensure parsers handle extended IDs if necessary.")


        if frame_id in self.parsers:
            parser = self.parsers[frame_id]
            
            # The parse method expects bytes-like object or list of int (byte array)
            # msg.data from python-can is already a bytearray, which works for can_msgs.msg.Frame.data
            # and should work for your parser if it expects a list of integers.
            parsedData = parser.parse(list(msg.data)) # Convert bytearray to list of ints for parser compatibility
            
            if parsedData:
                self.publishSensorData(frame_id, parsedData)
            else:
                rospy.logerr(f"Failed to parse CAN frame with ID {hex(frame_id)}")
        else:
            rospy.logdebug(f"Received CAN frame with unknown ID: {hex(frame_id)}")


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
            # rospy.logdebug(f"Published IMU data: {imuMsg.linear_acceleration.x}, {imuMsg.angular_velocity.z}")
        elif frameId == 0x003:  # GPS
            gpsMsg = NavSatFix()
            gpsMsg.header.stamp = rospy.Time.now()
            gpsMsg.latitude = data.get("latitude", 0.0)
            gpsMsg.longitude = data.get("longitude", 0.0)
            self.gpsPub.publish(gpsMsg)
            # rospy.logdebug(f"Published GPS data: {gpsMsg.latitude}, {gpsMsg.longitude}")
        elif frameId == 0x001:  # encoder
            encoderMsg = EncoderStamped()
            encoderMsg.header.stamp = rospy.Time.now()
            # Ensure encoderMsg.data has enough elements if it's a fixed-size array
            # Assuming EncoderStamped.data is a list/array that can be extended or has a fixed size you're filling.
            # If `data` field in EncoderStamped is a fixed-size array, you need to ensure it's initialized correctly.
            # For simplicity, assuming it's a list or similar:
            encoderMsg.data = [
                data.get("encoder1", 0.0),
                data.get("encoder2", 0.0),
                data.get("encoder3", 0.0),
                data.get("encoder4", 0.0),
                data.get("encoder5", 0.0),
                data.get("encoder6", 0.0)
            ]
            self.encoderPub.publish(encoderMsg)
            # rospy.logdebug(f"Published Encoder data: {encoderMsg.data}")
        elif frameId == 0x6A5:  # Test
            testMsg = String()
            testMsg.data = str(data)
            self.testPub.publish(testMsg)
            # rospy.logdebug(f"Published Test CAN data: {testMsg.data}")

    def keyboardControlCallback(self, msg: Twist) -> None:
        """Callback for processing keyboard control messages.
        Converts Twist message to CAN frame and sends it directly.

        Parameters
        ----------
        msg : Twist
            ROS keyboard twist teleop command message.

        Returns
        -------
        None
        """
        parser = KeyboardControlParser()
        # The parser is expected to return a can_msgs.msg.Frame
        can_frame_ros: Optional[Frame] = parser.parse(msg) 

        if can_frame_ros:
            try:
                # Convert can_msgs.msg.Frame to python-can Message
                can_message = can.Message(
                    arbitration_id=can_frame_ros.id,
                    is_extended_id=can_frame_ros.is_extended,
                    is_remote_frame=can_frame_ros.is_rtr,
                    dlc=len(can_frame_ros.data),
                    data=can_frame_ros.data # CORRECTED: Removed .tolist()
                )
                self.can_bus.send(can_message)
                rospy.logdebug(f"Sent CAN frame: ID=0x{can_frame_ros.id:X}, Data={can_frame_ros.data}")
            except can.CanError as e:
                rospy.logerr(f"Failed to send CAN frame via python-can: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error when sending CAN frame: {e}")
        else:
            rospy.logerr("Failed to parse keyboard control message into CAN frame.")

    def shutdown_hook(self):
        """Called when the ROS node is shutting down."""
        rospy.loginfo("Shutting down RosCanBridge...")
        self.running = False # Signal the CAN reception thread to stop
        if self.can_rx_thread.is_alive():
            self.can_rx_thread.join(timeout=1.0) # Wait for thread to finish
            if self.can_rx_thread.is_alive():
                rospy.logwarn("CAN reception thread did not terminate cleanly.")
        if hasattr(self, 'can_bus') and self.can_bus is not None:
            self.can_bus.shutdown() # Close the CAN bus interface
            rospy.loginfo("CAN bus closed.")
        rospy.loginfo("RosCanBridge shutdown complete.")


if __name__ == "__main__":
    try:
	#node = RosCanBridge()
	#rospy.on_shutdown(node.shutdown_hook)
#	rospy.on_shutdown(node.shutdown_hook) # Register shutdown hook

        node = RosCanBridge()

        rospy.on_shutdown(node.shutdown_hook)

        while not rospy.is_shutdown():
            node.heartbeatPub.publish("roscan:running")
            node.rate.sleep()

    except rospy.ROSInterruptException:
        pass

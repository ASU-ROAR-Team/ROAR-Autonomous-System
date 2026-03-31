#!/usr/bin/env python3
"""
IMU handler module.
"""
from typing import Dict, List, Optional
from .can_frame_parser import CanFrameParser


class ImuParser(CanFrameParser):  # pylint: disable=too-few-public-methods
    """Parser for IMU sensor data.

    Attributes
    ----------
    angularVelocityX : float
        Angular velocity around the X-axis.
    angularVelocityY : float
        Angular velocity around the Y-axis.
    angularVelocityZ : float
        Angular velocity around the Z-axis.

    Methods
    -------
    parse(frameData: List[int]) -> Optional[Dict[str, float]]:
        Parse CAN frame data into IMU values.
    """

    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame into a dictionary of IMU sensor data.

        Parameters
        ----------
        frameData : List[int]
            Raw data from the CAN frame.

        Returns
        -------
        Optional[Dict[str, float]]
            Parsed IMU sensor data, or None if parsing fails.

        Raises
        ------
        Exception
            If an error occurs during parsing.
        """
        try:
            # Combine bytes into 16-bit integers and process
            linAccX = ((frameData[1] << 8) | frameData[0]) / 100 - 360
            linAccY = ((frameData[3] << 8) | frameData[2]) / 100 - 360
            yawAngle = ((frameData[5] << 8) | frameData[4]) / 100 - 360
            yawRate = ((frameData[7] << 8) | frameData[6]) / 100 - 360

            return {
                "linearAccelerationX": linAccX,
                "linearAccelerationY": linAccY,
                "yawAngle": yawAngle,
                "yawRate": yawRate,
            }
        except IndexError as e:
            print(f"Error parsing IMU frame: {e}")
            return None

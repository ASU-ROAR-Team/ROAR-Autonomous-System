#!/usr/bin/env python3
"""
GPS handler module.
"""
from typing import Dict, List, Optional
from .can_frame_parser import CanFrameParser


class GpsParser(CanFrameParser):  # pylint: disable=too-few-public-methods
    """Parser for GPS sensor data.

    Attributes
    ----------
    CanFrameParser : class
        Base class for parsing CAN

    Methods
    -------
    parse(frameData: List[int]) -> Optional[Dict[str, float]]
        Parse a CAN frame into a dictionary of GPS data.
    """

    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame into a dictionary of GPS data.

        Parameters
        ----------
        frameData : List[int]
            Raw data from the CAN frame.

        Returns
        -------
        Optional[Dict[str, float]]
            Parsed GPS data, or None if parsing fails.

        Raises
        ------
        Exception
            Error parsing GPS frame
        """
        try:
            # Combine bytes into 16-bit integers
            latitude = ((frameData[1] << 8) | frameData[0]) / 100  # Assuming scaling factor of 100
            longitude = ((frameData[3] << 8) | frameData[2]) / 100
            altitude = ((frameData[5] << 8) | frameData[4]) / 100

            return {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
            }
        except IndexError as e:
            print(f"Error parsing GPS frame: {e}")
            return None

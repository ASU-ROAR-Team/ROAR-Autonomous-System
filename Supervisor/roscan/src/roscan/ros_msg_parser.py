#!/usr/bin/env python3
"""
Base classes for parsing ROS messages to CAN frames.
"""
from typing import Dict, List, Optional


class RosMsgParser:  # pylint: disable=too-few-public-methods
    """Base class for parsing ROS messages to CAN frames."""

    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a ROS message into a dictionary of sensor data.

        Args:
            frameData (List[int]): Raw data from the CAN frame.

        Returns:
            Optional[Dict[str, float]]: Parsed sensor data, or None if parsing fails.
        """
        raise NotImplementedError

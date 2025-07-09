#!/usr/bin/env python3
"""
Base classes for parsing CAN frames.
"""
from typing import Dict, List, Optional


class CanFrameParser:  # pylint: disable=too-few-public-methods
    """Base class for parsing CAN frames."""

    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame into a dictionary of sensor data.

        Args:
            frameData (List[int]): Raw data from the CAN frame.

        Returns:
            Optional[Dict[str, float]]: Parsed sensor data, or None if parsing fails.
        """
        raise NotImplementedError

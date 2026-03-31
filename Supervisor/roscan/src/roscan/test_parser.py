#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Test parser module for decoding CAN frame strings.
"""
from typing import Dict, List, Optional
from .can_frame_parser import CanFrameParser


class TestParser(CanFrameParser):  # pylint: disable=too-few-public-methods
    """Parser for decoding string data from CAN frames.

    A utility parser that converts CAN frame data into ASCII strings,
    useful for testing and debugging CAN communication.

    Methods
    -------
    parse(frameData: List[int]) -> Optional[str]:
        Parse CAN frame data into a readable ASCII string.
    """

    def parse(self, frameData: List[int]) -> Optional[str]:
        """
        Parse a CAN frame into a human-readable ASCII string.

        Parameters
        ----------
        frameData : List[int]
            Raw data from the CAN frame.

        Returns
        -------
        Optional[str]
            Decoded ASCII string from the CAN frame, or None if parsing fails.

        Raises
        ------
        Exception
            If an error occurs during string decoding.
        """
        try:
            # Convert frame bytes to ASCII string
            frame = "".join(format(x, "02x") for x in frameData)
            decoded_string = bytes.fromhex(frame).decode("ascii")
            return decoded_string

        except IndexError as e:
            print(f"Error parsing Testing frame: {e}")
            return None

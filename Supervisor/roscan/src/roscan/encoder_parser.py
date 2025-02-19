#!/usr/bin/env python3
"""
Encoder handler module.
"""
from typing import Dict, List, Optional
from .can_frame_parser import CanFrameParser


class EncoderParser(CanFrameParser):  # pylint: disable=too-few-public-methods
    """Parser for encoder data.

    Attributes
    ----------


    Methods
    -------
    parse(frameData: List[int]) -> Optional[Dict[str, float]]:
        Parse CAN frame data into encoder values.
    """

    def __init__(self) -> None:
        self.encodersMap = -16  # Adjustment value for encoder readings

    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame into a dictionary of encoder values.

        Parameters
        ----------
        frameData : List[int]
            Raw data from the CAN frame.

        Returns
        -------
        Optional[Dict[str, float]]
            Parsed encoder values, or None if parsing fails.

        Raises
        ------
        Exception
            If an error occurs during parsing.
        """

        try:
            # Extract encoder readings (first 6 bytes)
            encoderReadings = [reading + self.encodersMap for reading in frameData[:6]]
            return {
                "encoder1": encoderReadings[0],
                "encoder2": encoderReadings[1],
                "encoder3": encoderReadings[2],
                "encoder4": encoderReadings[3],
                "encoder5": encoderReadings[4],
                "encoder6": encoderReadings[5],
            }
        except IndexError as e:
            print(f"Error parsing encoder frame: {e}")
            return None

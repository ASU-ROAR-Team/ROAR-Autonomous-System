#!/usr/bin/env python

"""Module for calibrating map images using orange spots to transform between
pixel and real coordinates."""

import ast
import csv
from collections import namedtuple
from typing import List, Tuple, Optional, Any
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText
import matplotlib.patheffects as path_effects

# Data structures for grouped attributes
Origin = namedtuple("Origin", ["index", "pixel", "real"])
CsvPaths = namedtuple("CsvPaths", ["input", "output"])
PlotObjects = namedtuple("PlotObjects", ["figure", "axes"])


class MapCalibration:
    """Calibrates map images by detecting markers for coordinate transformations."""

    def __init__(self) -> None:
        """Initialize ROS node and class attributes."""
        rospy.init_node("map_calibration_node", anonymous=True)

        # Initialize attributes with type annotations
        self.image: Optional[np.ndarray] = None
        self.realCoords: List[Tuple[float, float]] = []
        self.csvPaths: CsvPaths = CsvPaths(None, None)
        self.pixelCoords: List[Tuple[int, int]] = []
        self.origin: Optional[Origin] = None
        self.scale: float = 1.0
        self.plot: Optional[PlotObjects] = None

        self._loadParameters()
        self._processImage()
        self._setupVisualization()

    def _loadParameters(self) -> None:
        """Load and validate parameters from ROS server."""
        imagePath = rospy.get_param("~image_path")
        self.image = cv2.imread(imagePath)
        self.realCoords = ast.literal_eval(rospy.get_param("~real_coords"))

        # Handle CSV parameters
        inputCsv = rospy.get_param("~input_csv", None) or None
        outputCsv = rospy.get_param("~output_csv", None) or None
        self.csvPaths = CsvPaths(inputCsv, outputCsv)

    def _processImage(self) -> None:
        """Process image to detect markers and calculate calibration."""
        if self.image is None:
            raise ValueError("Image not loaded")

        hsvImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvImage, np.array([5, 100, 100]), np.array([15, 255, 255]))

        contours = self._findContours(mask)
        self.pixelCoords = self._calculateCentroids(contours)
        self._calculateCalibration()

    def _findContours(self, mask: np.ndarray) -> List[Any]:
        """Find and sort contours in thresholded image."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return sorted(contours, key=cv2.contourArea, reverse=True)[:3]

    def _calculateCentroids(self, contours: List[Any]) -> List[Tuple[int, int]]:
        """Calculate centroids from contours and sort them."""
        centroids: List[Tuple[int, int]] = []
        for contour in contours:
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                centroidX = int(moments["m10"] / moments["m00"])
                centroidY = int(moments["m01"] / moments["m00"])
                centroids.append((centroidX, centroidY))
        return sorted(centroids, key=lambda p: (p[1], p[0]))

    def _calculateCalibration(self) -> None:
        """Calculate origin and scaling factors."""
        if not self.pixelCoords or not self.realCoords:
            raise ValueError("No coordinates detected")

        yCoords = [p[1] for p in self.pixelCoords]
        originIdx = np.argmax(yCoords)
        self.origin = Origin(
            index=originIdx, pixel=self.pixelCoords[originIdx], real=self.realCoords[originIdx]
        )

        scales: List[float] = []
        for idx, (pixelPoint, realPoint) in enumerate(zip(self.pixelCoords, self.realCoords)):
            if idx == self.origin.index:
                continue
            realYDelta = realPoint[1] - self.origin.real[1]
            pixelYDelta = self.origin.pixel[1] - pixelPoint[1]
            if realYDelta != 0:
                scales.append(pixelYDelta / realYDelta)

        self.scale = np.mean(scales) if scales else 1.0

    def _setupVisualization(self) -> None:
        """Initialize the calibration visualization display."""
        if self.image is None or self.origin is None:
            raise ValueError("Calibration not complete")

        plt.ion()
        figure, axes = plt.subplots()
        self.plot = PlotObjects(figure, axes)

        self.plot.axes.imshow(cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY), cmap="gray")
        self.plot.axes.grid(True, color="blue", linestyle="--", linewidth=0.5)
        self.plot.axes.set_title("Calibration Visualization")

        # Plot markers
        self.plot.axes.plot(self.origin.pixel[0], self.origin.pixel[1], "ro", markersize=10)
        self.plot.axes.text(self.origin.pixel[0], self.origin.pixel[1], "(0,0)", color="red")

        for idx, (pixelPoint, realPoint) in enumerate(zip(self.pixelCoords, self.realCoords)):
            if idx == self.origin.index:
                continue
            adjReal = (realPoint[0] - self.origin.real[0], realPoint[1] - self.origin.real[1])
            self.plot.axes.plot(pixelPoint[0], pixelPoint[1], "yo", markersize=10)
            self.plot.axes.text(
                pixelPoint[0], pixelPoint[1], f"({adjReal[0]}, {adjReal[1]})", color="yellow"
            )

        # Add help text
        self.plot.axes.add_artist(
            AnchoredText(
                "Enter:\n- 'r x,y' for real→pixel\n- 'p x,y' for pixel→real", loc="upper right"
            )
        )
        plt.draw()

    def realToPixel(self, realX: float, realY: float) -> Tuple[float, float]:
        """Convert real-world coordinates to pixel coordinates."""
        if self.origin is None:
            raise ValueError("Calibration not complete")
        return (
            self.origin.pixel[0] + self.scale * realX,
            self.origin.pixel[1] - self.scale * realY,
        )

    def pixelToReal(self, pixelX: float, pixelY: float) -> Tuple[float, float]:
        """Convert pixel coordinates to real-world coordinates."""
        if self.origin is None:
            raise ValueError("Calibration not complete")
        return (
            (pixelX - self.origin.pixel[0]) / self.scale,
            (self.origin.pixel[1] - pixelY) / self.scale,
        )

    def processCsv(self) -> None:
        """Convert pixel coordinates in CSV to real-world coordinates."""
        if not self.csvPaths.input or not self.csvPaths.output:
            rospy.logwarn("Missing CSV paths, skipping processing.")
            return

        try:
            with open(self.csvPaths.input, "r", encoding="utf-8") as infile, open(
                self.csvPaths.output, "w", encoding="utf-8"
            ) as outfile:

                reader = csv.DictReader(infile)
                if (
                    reader.fieldnames is None
                    or "x_pixel" not in reader.fieldnames
                    or "y_pixel" not in reader.fieldnames
                ):
                    rospy.logerr("CSV missing required columns")
                    return

                writer = csv.DictWriter(outfile, list(reader.fieldnames) + ["real_x", "real_y"])
                writer.writeheader()

                for row in reader:
                    try:
                        xPixel = int(row["x_pixel"])
                        yPixel = int(row["y_pixel"])
                    except ValueError as e:
                        rospy.logwarn(f"Skipping invalid row: {e}")
                        continue

                    realX, realY = self.pixelToReal(xPixel, yPixel)
                    row["real_x"] = f"{realX:.4f}"
                    row["real_y"] = f"{realY:.4f}"
                    writer.writerow(row)

                rospy.loginfo(f"Processed CSV saved to {self.csvPaths.output}")

        except csv.Error as e:
            rospy.logerr(f"CSV processing error: {str(e)}")
        except IOError as e:
            rospy.logerr(f"File I/O error: {str(e)}")

    def _processCommandPart(self, cmdPart: str) -> None:
        """Process individual command part with reduced complexity."""
        try:
            cmdType, coordStr = cmdPart.split(maxsplit=1)
            coordVals = list(map(float, coordStr.split(",")))

            if cmdType.lower() == "r" and len(coordVals) == 2:
                self._handleRealToPixel(coordVals)
            elif cmdType.lower() == "p" and len(coordVals) == 2:
                self._handlePixelToReal(coordVals)
            else:
                raise ValueError("Invalid command format")
        except ValueError as e:
            print(f"Error processing '{cmdPart}': {str(e)}")

    def _handleRealToPixel(self, coordVals: List[float]) -> None:
        """Handle real-to-pixel conversion with plotting."""
        realX, realY = coordVals
        pixelX, pixelY = self.realToPixel(realX, realY)
        print(f"[R→P] ({realX},{realY}) → ({pixelX:.2f}, {pixelY:.2f})")

        if self.plot:
            self.plot.axes.plot(pixelX, pixelY, "go", markersize=10)
            label = self.plot.axes.text(
                pixelX, pixelY, f"R({realX},{realY})", color="green", fontsize=6
            )
            label.set_path_effects(
                [path_effects.Stroke(linewidth=1, foreground="black"), path_effects.Normal()]
            )

    def _handlePixelToReal(self, coordVals: List[float]) -> None:
        """Handle pixel-to-real conversion with plotting."""
        pixelX, pixelY = coordVals
        realX, realY = self.pixelToReal(pixelX, pixelY)
        print(f"[P→R] ({pixelX},{pixelY}) → ({realX:.2f}, {realY:.2f})")

        if self.plot:
            self.plot.axes.plot(pixelX, pixelY, "ms", markersize=10)
            self.plot.axes.text(pixelX, pixelY, f"P({realX:.1f},{realY:.1f})", color="magenta")

    def run(self) -> None:
        """Main execution loop handling user input and processing."""
        if self.csvPaths.input and self.csvPaths.output:
            self.processCsv()

        print("Ready for coordinate conversions (commands separated by ';')...")
        while not rospy.is_shutdown():
            try:
                userInput = input("Enter command(s): ").strip()
                if not userInput:
                    continue

                for cmdPart in userInput.split(";"):
                    processedCmd = cmdPart.strip()
                    if processedCmd:
                        self._processCommandPart(processedCmd)

                if self.plot:
                    plt.draw()

            except KeyboardInterrupt:
                rospy.signal_shutdown("User exit")
            except Exception as e:  # pylint: disable=broad-except
                print(f"Unexpected error: {str(e)}")

        plt.show(block=True)


if __name__ == "__main__":
    try:
        calibrator = MapCalibration()
        calibrator.run()
    except rospy.ROSInterruptException:
        pass

"""
Module for calibrating map images using horizontally-aligned red spots to
transform between pixel and real coordinates. The origin is the leftmost marker.
"""

import ast
import csv
from collections import namedtuple
from itertools import combinations
from typing import List, Tuple, Optional, Any
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText
import matplotlib.patheffects as path_effects

# Data structures for grouped attributes
Origin = namedtuple("Origin", ["index", "pixel", "real"])
CsvPaths = namedtuple("CsvPaths", ["input", "output"])
PlotObjects = namedtuple("PlotObjects", ["figure", "axes"])


class MapCalibration(Node):
    """Calibrates map images by detecting markers for coordinate transformations."""

    def __init__(self) -> None:
        """Initialize ROS node and class attributes."""
        super().__init__("map_calibration_node")

        # Declare parameters
        self.declare_parameter("image_path", "")
        self.declare_parameter("real_coords", "")
        self.declare_parameter("input_csv", "")
        self.declare_parameter("output_csv", "")

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
        self._calculateCalibration()
        self._setupVisualization()

    def _loadParameters(self) -> None:
        """Load and validate parameters from ROS server."""
        imagePath = self.get_parameter("image_path").get_parameter_value().string_value
        self.image = cv2.imread(imagePath)
        if self.image is None:
            raise IOError(f"Failed to load image from {imagePath}")

        real_coords_str = self.get_parameter("real_coords").get_parameter_value().string_value
        self.realCoords = ast.literal_eval(real_coords_str)

        # Handle CSV parameters
        inputCsv = self.get_parameter("input_csv").get_parameter_value().string_value
        outputCsv = self.get_parameter("output_csv").get_parameter_value().string_value
        self.csvPaths = CsvPaths(
            inputCsv if inputCsv else None,
            outputCsv if outputCsv else None
        )

    def _processImage(self) -> None:
        """
        Process the image to find the best horizontally-aligned markers.
        This method has been updated to find red markers.
        """
        if self.image is None:
            raise ValueError("Image not loaded")

        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
        # Define two HSV ranges for the color red (as it wraps around 0/180)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        
        # Combine masks to detect red color across the spectrum
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size and find their centroids
        centroids = []
        min_area, max_area = 10, 1000  # Filter out noise and large blobs
        for contour in contours:
            if min_area < cv2.contourArea(contour) < max_area:
                moments = cv2.moments(contour)
                if moments["m00"] > 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    centroids.append((cx, cy))
        
        num_markers = len(self.realCoords)
        if len(centroids) < num_markers:
            raise RuntimeError(f"Could not find enough candidate markers. Found {len(centroids)}, expected {num_markers}.")

        # Find the combination of N markers that is most horizontal
        best_combo = None
        min_y_std_dev = float('inf')
        for combo in combinations(centroids, num_markers):
            y_coords = [p[1] for p in combo]
            std_dev = np.std(y_coords)
            if std_dev < min_y_std_dev:
                min_y_std_dev = std_dev
                best_combo = combo

        # Sort the final list of markers by their x-coordinate
        self.pixelCoords = sorted(list(best_combo), key=lambda p: p[0])

    def _calculateCalibration(self) -> None:
        """
        Calculate origin and scaling factor based on horizontal alignment.
        """
        if not self.pixelCoords or not self.realCoords:
            raise ValueError("No coordinates available for calibration.")

        # The origin is the leftmost marker (minimum x pixel coordinate)
        origin_idx = 0  # Since pixelCoords are now sorted by x, the first one is the origin.
        self.origin = Origin(
            index=origin_idx,
            pixel=self.pixelCoords[origin_idx],
            real=self.realCoords[origin_idx],
        )

        # Calculate scale based on the X-axis distances
        scales: List[float] = []
        for idx, (pixelPoint, realPoint) in enumerate(zip(self.pixelCoords, self.realCoords)):
            if idx == self.origin.index:
                continue
            
            # Calculate pixel and real-world deltas from the origin along the x-axis
            real_x_delta = realPoint[0] - self.origin.real[0]
            pixel_x_delta = pixelPoint[0] - self.origin.pixel[0]
            
            if real_x_delta != 0:
                scales.append(pixel_x_delta / real_x_delta)

        if not scales:
            raise RuntimeError("Could not calculate scale. Check real_coords for non-zero distances.")
             
        self.scale = np.mean(scales)

    def _setupVisualization(self) -> None:
        """Initialize the calibration visualization display."""
        if self.image is None or self.origin is None:
            raise ValueError("Calibration not complete for visualization")

        plt.ion()
        figure, axes = plt.subplots(figsize=(12, 8))
        self.plot = PlotObjects(figure, axes)

        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.plot.axes.imshow(rgb_image)
        
        self.plot.axes.grid(True, color="blue", linestyle="--", linewidth=0.5, alpha=0.7)
        self.plot.axes.set_title("Calibration Visualization")

        # Plot origin marker
        self.plot.axes.plot(self.origin.pixel[0], self.origin.pixel[1], "ro", markersize=12, markeredgecolor='white')
        origin_text = self.plot.axes.text(self.origin.pixel[0] + 10, self.origin.pixel[1],
                           f"({self.origin.real[0]}, {self.origin.real[1]})", color="red",
                           fontsize=10, weight='bold')
        origin_text.set_path_effects([path_effects.Stroke(linewidth=2, foreground='white'), path_effects.Normal()])

        # Plot other calibration markers
        for idx, (pixelPoint, realPoint) in enumerate(zip(self.pixelCoords, self.realCoords)):
            if idx == self.origin.index:
                continue
            self.plot.axes.plot(pixelPoint[0], pixelPoint[1], "o", color='yellow', markersize=12, markeredgecolor='black')
            marker_text = self.plot.axes.text(
                pixelPoint[0] + 10, pixelPoint[1], f"({realPoint[0]}, {realPoint[1]})", color="yellow",
                fontsize=10, weight='bold'
            )
            marker_text.set_path_effects([path_effects.Stroke(linewidth=2, foreground='black'), path_effects.Normal()])

        # Add help text
        help_text = self.plot.axes.add_artist(
            AnchoredText(
                "Enter:\n- 'r x,y' for real→pixel\n- 'p x,y' for pixel→real", loc="upper right"
            )
        )
        help_text.patch.set_boxstyle("round,pad=0.5")
        help_text.patch.set_facecolor("wheat")
        help_text.patch.set_alpha(0.8)

        plt.tight_layout()
        plt.draw()

    def realToPixel(self, realX: float, realY: float) -> Tuple[float, float]:
        """
        Convert real-world coordinates to pixel coordinates.
        Updated for horizontal system with Y-axis increasing downwards.
        """
        if self.origin is None:
            raise ValueError("Calibration not complete")
            
        adj_real_x = realX - self.origin.real[0]
        adj_real_y = realY - self.origin.real[1]
        
        pixelX = self.origin.pixel[0] + self.scale * adj_real_x
        pixelY = self.origin.pixel[1] + self.scale * adj_real_y
        
        return (pixelX, pixelY)

    def pixelToReal(self, pixelX: float, pixelY: float) -> Tuple[float, float]:
        """
        Convert pixel coordinates to real-world coordinates.
        Updated for horizontal system with Y-axis increasing downwards.
        """
        if self.origin is None:
            raise ValueError("Calibration not complete")
            
        real_x_delta = (pixelX - self.origin.pixel[0]) / self.scale
        real_y_delta = (pixelY - self.origin.pixel[1]) / self.scale
        
        realX = self.origin.real[0] + real_x_delta
        realY = self.origin.real[1] + real_y_delta

        return (realX, realY)

    def processCsv(self) -> None:
        """Convert pixel coordinates in CSV to real-world coordinates."""
        if not self.csvPaths.input or not self.csvPaths.output:
            self.get_logger().warn("Missing CSV paths, skipping processing.")
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
                    self.get_logger().error("CSV missing required columns")
                    return

                writer = csv.DictWriter(outfile, list(reader.fieldnames) + ["real_x", "real_y"])
                writer.writeheader()

                for row in reader:
                    try:
                        xPixel = int(row["x_pixel"])
                        yPixel = int(row["y_pixel"])
                    except ValueError as e:
                        self.get_logger().warn(f"Skipping invalid row: {e}")
                        continue

                    realX, realY = self.pixelToReal(xPixel, yPixel)
                    row["real_x"] = f"{realX:.4f}"
                    row["real_y"] = f"{realY:.4f}"
                    writer.writerow(row)

                self.get_logger().info(f"Processed CSV saved to {self.csvPaths.output}")

        except csv.Error as e:
            self.get_logger().error(f"CSV processing error: {str(e)}")
        except IOError as e:
            self.get_logger().error(f"File I/O error: {str(e)}")

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
            self.plot.axes.plot(pixelX, pixelY, 'o', color='lime', markersize=12, markeredgecolor='black',
                               label=f"R→P: ({realX},{realY})")
            label = self.plot.axes.text(
                pixelX + 10, pixelY, f"({realX},{realY})", color='lime', fontsize=10, weight='bold'
            )
            label.set_path_effects(
                [path_effects.Stroke(linewidth=2, foreground='black'), path_effects.Normal()]
            )

    def _handlePixelToReal(self, coordVals: List[float]) -> None:
        """Handle pixel-to-real conversion with plotting."""
        pixelX, pixelY = coordVals
        realX, realY = self.pixelToReal(pixelX, pixelY)
        print(f"[P→R] ({pixelX},{pixelY}) → ({realX:.2f}, {realY:.2f})")

        if self.plot:
            self.plot.axes.plot(pixelX, pixelY, 's', color='fuchsia', markersize=12, markeredgecolor='black',
                               label=f"P→R: ({pixelX},{pixelY})")
            label = self.plot.axes.text(pixelX + 10, pixelY, f"({realX:.1f},{realY:.1f})",
                                        color='fuchsia', fontsize=10, weight='bold')
            label.set_path_effects(
                [path_effects.Stroke(linewidth=2, foreground='black'), path_effects.Normal()]
            )

    def run(self) -> None:
        """Main execution loop handling user input and processing."""
        if self.csvPaths.input and self.csvPaths.output:
            self.processCsv()

        self.get_logger().info("Calibration complete. Ready for coordinate conversions.")
        self.get_logger().info(f"Detected Pixel Coords: {self.pixelCoords}")
        self.get_logger().info(f"Calculated Scale: {self.scale:.4f} pixels per unit")
        self.get_logger().info(f"Origin Pixel: {self.origin.pixel}, Origin Real: {self.origin.real}")
        
        print("\nReady for coordinate conversions (commands separated by ';')...")
        print("Example: p 150,250 ; r 5.5,12.0")
        
        while rclpy.ok():
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

            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("User requested exit.")
                break
            except Exception as e:
                print(f"An unexpected error occurred: {str(e)}")

        plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibrator = MapCalibration()
        calibrator.run()
    except (RuntimeError, ValueError, IOError) as e:
        print(f"Calibration failed: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
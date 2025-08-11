#!/usr/bin/env python3

"""Converts a heightmap image to a costmap considering terrain slope and stability."""

import csv
import os
from typing import Optional, NamedTuple
import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy


class GradientData(NamedTuple):
    """Container for gradient calculations."""

    x: np.ndarray
    y: np.ndarray
    mag: np.ndarray
    laplacian: np.ndarray


class CostData(NamedTuple):
    """Container for cost calculation results."""

    total: np.ndarray
    x: np.ndarray
    y: np.ndarray


class HeightmapConverter:
    """Converts heightmap images to costmaps for robotic path planning."""

    def __init__(self) -> None:
        rospy.init_node("heightmap_to_costmap")
        self.imagePath: str = rospy.get_param("~image_path", "heightmap.png")
        self.gradientScale: float = rospy.get_param("~gradient_scale", 150.0)
        self.stabilityScale: float = rospy.get_param("~stability_scale", 90.0)

        self.heightmap: Optional[np.ndarray] = self._loadHeightmap()
        self.gradientData: Optional[GradientData] = None
        self.costData: Optional[CostData] = None

    def run(self) -> None:
        """Main execution method."""
        if self.heightmap is None:
            return

        self._processHeightmap()
        self._computeCosts()
        self._saveCosts()
        self._visualize()

    def getCostData(self) -> Optional[CostData]:
        """Public method to access cost data."""
        return self.costData

    def _loadHeightmap(self) -> Optional[np.ndarray]:
        """Load and preprocess the heightmap image."""
        heightmap = cv2.imread(self.imagePath, cv2.IMREAD_GRAYSCALE)
        if heightmap is None:
            rospy.logerr(f"Failed to load image: {self.imagePath}")
            return None

        mask = heightmap > 250
        heightmap = heightmap.astype(np.float32)
        heightmap[mask] = np.nan
        return heightmap

    def _processHeightmap(self) -> None:
        """Compute gradient and stability metrics from the heightmap."""
        if self.heightmap is None:
            raise ValueError("Heightmap not loaded")

        gradX = cv2.Sobel(self.heightmap, cv2.CV_32F, 1, 0, ksize=3)
        gradY = cv2.Sobel(self.heightmap, cv2.CV_32F, 0, 1, ksize=3)
        gradientMag = np.sqrt(gradX**2 + gradY**2)
        laplacian = cv2.Laplacian(self.heightmap, cv2.CV_32F, ksize=3)

        self.gradientData = GradientData(gradX, gradY, gradientMag, laplacian)

    def _computeCosts(self) -> None:
        """Calculate cost components and total cost."""
        if self.gradientData is None:
            raise ValueError("Gradient data not initialized")

        stability = np.abs(self.gradientData.laplacian)
        gradientCost = (self.gradientData.mag / 255.0) * self.gradientScale
        stabilityCost = (stability / 255.0) * self.stabilityScale

        totalCost = np.clip(gradientCost + stabilityCost, 0, 100)
        totalCost[np.isnan(totalCost)] = -1

        costX = np.clip(
            (np.abs(self.gradientData.x) / 255.0 * self.gradientScale) + stabilityCost, 0, 100
        )
        costY = np.clip(
            (np.abs(self.gradientData.y) / 255.0 * self.gradientScale) + stabilityCost, 0, 100
        )

        self.costData = CostData(
            totalCost.astype(np.int8), costX.astype(np.int8), costY.astype(np.int8)
        )

    def _saveCosts(self) -> None:
        """Save computed costs to CSV files."""
        if self.costData is None:
            raise ValueError("Cost data not initialized")

        self._saveCostGrid("cost_x.csv", self.costData.x)
        self._saveCostGrid("cost_y.csv", self.costData.y)
        self._saveCostGrid("total_cost.csv", self.costData.total)
        rospy.loginfo(f"Cost files saved in: {os.getcwd()}")

    def _saveCostGrid(self, filename: str, grid: np.ndarray) -> None:
        """Save cost grid data with positional indices."""
        with open(filename, "w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([""] + [str(i) for i in range(grid.shape[1])])
            for yIdx, row in enumerate(grid):
                writer.writerow([str(yIdx)] + [str(int(v)) for v in row])

    def _visualize(self) -> None:
        """Generate visualization plots for cost components."""
        if self.costData is None:
            raise ValueError("Cost data not initialized")

        _, axes = plt.subplots(1, 3, figsize=(15, 5))
        cmap = plt.cm.hot
        cmap.set_under("gray")

        for axis, data, title in zip(
            axes.flat,
            [self.costData.total, self.costData.x, self.costData.y],
            ["Total Cost", "X Cost", "Y Cost"],
        ):
            axis.imshow(data, cmap=cmap, vmin=0, vmax=100)
            axis.set_title(title)

        plt.colorbar(axes[0].images[0], ax=axes, orientation="horizontal")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    try:
        converter = HeightmapConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass

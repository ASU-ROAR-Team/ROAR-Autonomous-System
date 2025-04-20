#!/usr/bin/env python3

"""Path planning using A* algorithm for optimal path selection in ROS."""

# pylint: disable=too-many-instance-attributes, too-many-public-methods

import os
import csv
import heapq
import itertools
from math import sqrt
from typing import List, Tuple, Dict, Set, Optional, Any
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import rospy
from nav_msgs.msg import OccupancyGrid


class OptimalPathPlanner:
    """Implements an optimal path planner using A* algorithm with terrain awareness."""

    def __init__(self) -> None:
        rospy.init_node("optimal_point_selector")
        plt.ion()
        self.fig, self.axis = plt.subplots(figsize=(12, 8))

        self.loadParameters()
        self.loadCostmap()
        self.allPoints = self.getWaypoints()

        self.accessibilityScores: np.ndarray = self.calculateAccessibility()
        self.costMatrix: np.ndarray
        self.distanceMatrix: np.ndarray
        self.costMatrix, self.distanceMatrix = self.precomputeAllPairs()
        self.bestCombination: Optional[Dict[str, Any]] = self.findOptimalCombination()

        if self.bestCombination:
            fullPath: List[int] = self.generateCombinationPath(self.bestCombination)
            smoothedPath: List[int] = self.smoothPath(fullPath)
            self.visualizeResults(smoothedPath, self.bestCombination)
            self.publishCostmap()

        plt.ioff()
        plt.show()
        rospy.spin()

    def loadParameters(self) -> None:
        """Load parameters from ROS parameter server."""
        self.totalCostCsv: str = rospy.get_param("~total_cost_csv", "total_cost.csv")
        self.resolution: float = rospy.get_param("~resolution", 0.0586901)
        self.originX: float = rospy.get_param("~origin_x", 0.0)
        self.originY: float = rospy.get_param("~origin_y", 0.0)
        self.outputCsv: str = rospy.get_param("~output_csv", "optimal_path.csv")
        self.start: Tuple[int, int] = (
            rospy.get_param("~start_x", 380),
            rospy.get_param("~start_y", 50),
        )
        self.viaPoints: List[Tuple[int, int]] = [
            (rospy.get_param(f"~via{i}_x", 100), rospy.get_param(f"~via{i}_y", 100))
            for i in range(1, 10)
        ]

    def calculateAccessibility(self, radius: int = 5) -> np.ndarray:
        """Calculate accessibility scores based on surrounding terrain costs."""
        scores: List[float] = []
        for (xCoord, yCoord) in self.allPoints:
            xMin: int = max(0, xCoord - radius)
            xMax: int = min(self.width, xCoord + radius)
            yMin: int = max(0, yCoord - radius)
            yMax: int = min(self.height, yCoord + radius)

            region: np.ndarray = self.costmap[yMin:yMax, xMin:xMax]
            validCosts: np.ndarray = region[region != -1]

            if len(validCosts) == 0:
                score = 100.0  # Inaccessible
            else:
                score = float(np.mean(validCosts) + 0.5 * np.std(validCosts))

            scores.append(score)
        return np.array(scores)

    def getWaypoints(self) -> List[Tuple[int, int]]:
        """Get list of all waypoints including start and via points."""
        return [self.start] + self.viaPoints

    def pixelToIndex(self, xCoord: int, yCoord: int) -> Optional[int]:
        """Convert pixel coordinates to costmap index."""
        if 0 <= xCoord < self.width and 0 <= yCoord < self.height:
            return yCoord * self.width + xCoord
        return None

    def loadCostmap(self) -> None:
        """Load costmap from CSV file."""
        self.costmap: np.ndarray = np.loadtxt(self.totalCostCsv, delimiter=",").astype(np.int8)
        self.height: int
        self.width: int
        self.height, self.width = self.costmap.shape

    def precomputeAllPairs(self) -> Tuple[np.ndarray, np.ndarray]:
        """Precompute both cost and distance between all points."""
        numPoints: int = len(self.allPoints)
        costMatrix: np.ndarray = np.full((numPoints, numPoints), np.inf)
        distanceMatrix: np.ndarray = np.full((numPoints, numPoints), np.inf)
        pointIndices: List[Optional[int]] = [self.pixelToIndex(x, y) for (x, y) in self.allPoints]

        for i in tqdm(range(numPoints)):
            for j in range(numPoints):
                if i == j:
                    costMatrix[i][j] = 0
                    distanceMatrix[i][j] = 0
                    continue

                startIdx = pointIndices[i]
                goalIdx = pointIndices[j]
                if startIdx is None or goalIdx is None:
                    continue

                path, _, cost = self.astar(startIdx, goalIdx, visualize=False)
                if path:
                    costMatrix[i][j] = cost
                    distanceMatrix[i][j] = self.calculatePathDistance(path)

        return costMatrix, distanceMatrix

    def findOptimalCombination(self) -> Optional[Dict[str, Any]]:
        """Find optimal 4-point combination considering accessibility and path costs."""
        bestScore: float = float("inf")
        bestCombo: Optional[Dict[str, Any]] = None
        viaIndices: List[int] = list(range(1, 10))
        combinations: List[Tuple[int, ...]] = list(itertools.combinations(viaIndices, 4))

        for combo in tqdm(combinations):
            points: List[int] = [0] + list(combo)
            try:
                accessScore: float = np.mean(self.accessibilityScores[list(combo)])
                minCost: float
                minDist: float
                bestSeq: Optional[List[int]]
                minCost, minDist, bestSeq = self.findBestPermutation(points)

                if bestSeq is None:
                    continue

                score: float = (0.7 * minCost) + (0.3 * minDist) + (0.3 * accessScore)
                if score < bestScore:
                    bestScore = score
                    bestCombo = {
                        "points": combo,
                        "sequence": bestSeq,
                        "cost": minCost,
                        "distance": minDist,
                        "accessibility": accessScore,
                    }
            except Exception as err:  # pylint: disable=broad-except
                rospy.logwarn(f"Skipping combination {combo}: {str(err)}")

        if bestCombo:
            rospy.loginfo(f"Best sequence: {bestCombo['sequence']}")
        return bestCombo

    def findBestPermutation(self, points: List[int]) -> Tuple[float, float, Optional[List[int]]]:
        """Find best path sequence for a combination of points based on distance."""
        minDist: float = float("inf")
        minCost: float = float("inf")
        bestSeq: Optional[List[int]] = None

        for perm in itertools.permutations(points[1:]):
            sequence: List[int] = [0] + list(perm) + [0]
            totalDist: float = 0.0
            totalCost: float = 0.0
            valid: bool = True

            for i in range(len(sequence) - 1):
                fromIdx: int = sequence[i]
                toIdx: int = sequence[i + 1]

                if self.distanceMatrix[fromIdx][toIdx] == np.inf:
                    valid = False
                    break
                totalDist += self.distanceMatrix[fromIdx][toIdx]
                totalCost += self.costMatrix[fromIdx][toIdx]

            if valid and totalDist < minDist:
                minDist = totalDist
                minCost = totalCost
                bestSeq = sequence

        return minCost, minDist, bestSeq

    def calculatePathDistance(self, path: List[int]) -> float:
        """Calculate actual traveled distance from path indices."""
        distance: float = 0.0
        for i in range(1, len(path)):
            xPrev: int = path[i - 1] % self.width
            yPrev: int = path[i - 1] // self.width
            xCurr: int = path[i] % self.width
            yCurr: int = path[i] // self.width
            distance += sqrt((xCurr - xPrev) ** 2 + (yCurr - yPrev) ** 2) * self.resolution
        return distance

    def generateCombinationPath(self, combination: Dict[str, Any]) -> List[int]:
        """Generate full path for the best combination."""
        fullPath: List[int] = []
        sequence: List[int] = combination["sequence"]

        for i in range(len(sequence) - 1):
            startIdx: Optional[int] = self.pixelToIndex(*self.allPoints[sequence[i]])
            goalIdx: Optional[int] = self.pixelToIndex(*self.allPoints[sequence[i + 1]])

            if startIdx is None or goalIdx is None:
                continue

            path, _, _ = self.astar(startIdx, goalIdx, visualize=True)
            if path:
                fullPath.extend(path[1:] if fullPath else path)
        return fullPath

    def astar(
        self, startIdx: int, goalIdx: int, visualize: bool = True
    ) -> Tuple[Optional[List[int]], Set[int], float]:
        """A* pathfinding algorithm with terrain cost consideration."""
        openHeap: List[Tuple[float, float, int]] = []
        heapq.heappush(openHeap, (0.0, 0.0, startIdx))
        cameFrom: Dict[int, int] = {}
        gScore: Dict[int, float] = {startIdx: 0.0}
        closedSet: Set[int] = set()

        while openHeap:
            # Use underscore for unused variables
            _, _, current = heapq.heappop(openHeap)

            if current == goalIdx:
                return self.reconstructPath(cameFrom, current), closedSet, gScore[current]

            if current in closedSet:
                continue
            closedSet.add(current)

            if visualize and len(closedSet) % 100 == 0:
                self.updateVisualization(closedSet)

            for neighbor, cost in self.getNeighbors(current):
                if neighbor in closedSet:
                    continue

                tentativeG: float = gScore[current] + cost + self.getTerrainCost(neighbor)
                if tentativeG < gScore.get(neighbor, float("inf")):
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentativeG
                    hScore: float = self.heuristic(neighbor, goalIdx)
                    heapq.heappush(openHeap, (tentativeG + hScore, hScore, neighbor))

        return None, closedSet, float("inf")

    def reconstructPath(self, cameFrom: Dict[int, int], current: int) -> List[int]:
        """Reconstruct path from cameFrom dictionary."""
        path: List[int] = [current]
        while current in cameFrom:
            current = cameFrom[current]
            path.append(current)
        return path[::-1]

    def smoothPath(self, path: List[int]) -> List[int]:
        """Smooth path using line-of-sight check."""
        if len(path) < 2:
            return path

        smoothed: List[int] = [path[0]]
        currentIdx: int = 0
        stepSize: int = 2

        while currentIdx < len(path) - 1:
            maxReach: int = min(currentIdx + stepSize * 2, len(path) - 1)
            bestIdx: int = currentIdx

            for testIdx in range(currentIdx + 1, maxReach + 1):
                currentX: int = smoothed[-1] % self.width
                currentY: int = smoothed[-1] // self.width
                testX: int = path[testIdx] % self.width
                testY: int = path[testIdx] // self.width

                if self.hasLineOfSight((currentX, currentY), (testX, testY)):
                    bestIdx = testIdx

            if bestIdx > currentIdx:
                smoothed.append(path[bestIdx])
                currentIdx = bestIdx
            else:
                currentIdx += 1

        return smoothed

    def updateVisualization(
        self, closedSet: Set[int], currentPath: Optional[List[int]] = None
    ) -> None:
        """Update matplotlib visualization."""
        self.axis.clear()
        self.axis.imshow(self.costmap, cmap="gray", origin="upper", vmin=-1, vmax=100)

        if closedSet:
            exploredX: List[int] = [idx % self.width for idx in closedSet]
            exploredY: List[int] = [idx // self.width for idx in closedSet]
            self.axis.scatter(exploredX, exploredY, c="blue", s=1, alpha=0.1, label="Explored")

        if currentPath:
            pathX: List[int] = [idx % self.width for idx in currentPath]
            pathY: List[int] = [idx // self.width for idx in currentPath]
            self.axis.plot(pathX, pathY, "r-", linewidth=2, label="Current Path")

        wayX: List[int] = [p[0] for p in self.allPoints]
        wayY: List[int] = [p[1] for p in self.allPoints]
        self.axis.scatter(wayX, wayY, c="green", s=100, marker="o", label="Waypoints")
        self.axis.scatter(
            self.start[0], self.start[1], c="red", s=150, marker="*", label="Start/End"
        )

        self.axis.set_title("Optimal Path Planning")
        self.axis.legend()
        plt.draw()
        plt.pause(0.001)

    def visualizeResults(self, path: List[int], combination: Dict[str, Any]) -> None:
        """Visualize results with accessibility and path."""
        self.axis.clear()
        self.axis.imshow(self.costmap, cmap="gray", origin="upper", vmin=-1, vmax=100, alpha=0.7)

        for idx, score in enumerate(self.accessibilityScores):
            xCoord, yCoord = self.allPoints[idx]
            self.axis.add_patch(
                plt.Circle((xCoord, yCoord), 10, color=plt.cm.RdYlGn_r(score / 100), alpha=0.3)
            )

        allX: List[int] = [p[0] for p in self.allPoints]
        allY: List[int] = [p[1] for p in self.allPoints]
        self.axis.scatter(
            allX,
            allY,
            c=self.accessibilityScores,
            cmap="RdYlGn_r",
            s=100,
            edgecolors="k",
            vmin=0,
            vmax=100,
            label="Waypoints",
        )

        selPoints: List[Tuple[int, int]] = [self.allPoints[i] for i in combination["points"]]
        selX: List[int] = [p[0] for p in selPoints]
        selY: List[int] = [p[1] for p in selPoints]
        self.axis.scatter(selX, selY, c="blue", s=200, marker="s", edgecolors="k", label="Selected")

        pathX: List[int] = [idx % self.width for idx in path]
        pathY: List[int] = [idx // self.width for idx in path]
        self.axis.plot(pathX, pathY, "r-", linewidth=2, label="Path")

        self.axis.scatter(
            self.start[0], self.start[1], c="red", s=300, marker="*", label="Start/End"
        )

        plt.colorbar(plt.cm.ScalarMappable(cmap="RdYlGn_r"), label="Accessibility Score")
        self.axis.set_title(
            f"Optimal Path\nCost: {combination['cost']:.1f}, Dist: {combination['distance']:.1f}m"
        )
        self.axis.legend()

        plt.draw()
        plt.savefig(f"{os.path.splitext(self.outputCsv)[0]}.png")
        plt.pause(2)
        self.savePathToCsv(path)

    def getNeighbors(self, idx: int) -> List[Tuple[int, float]]:
        """Get valid neighbors for a given index."""
        xCoord: int = idx % self.width
        yCoord: int = idx // self.width
        neighbors: List[Tuple[int, float]] = []
        for deltaX in [-1, 0, 1]:
            for deltaY in [-1, 0, 1]:
                if deltaX == 0 and deltaY == 0:
                    continue
                nextX: int = xCoord + deltaX
                nextY: int = yCoord + deltaY
                if 0 <= nextX < self.width and 0 <= nextY < self.height:
                    nidx: int = nextY * self.width + nextX
                    if self.costmap[nextY, nextX] != -1:
                        cost: float = 1.414 if (deltaX and deltaY) else 1.0
                        neighbors.append((nidx, cost))
        return neighbors

    def heuristic(self, aIdx: int, bIdx: int) -> float:
        """Calculate Euclidean distance heuristic."""
        xStart: int = aIdx % self.width
        yStart: int = aIdx // self.width
        xEnd: int = bIdx % self.width
        yEnd: int = bIdx // self.width
        return sqrt((xEnd - xStart) ** 2 + (yEnd - yStart) ** 2)

    def getTerrainCost(self, idx: int) -> float:
        """Get terrain cost for a given index."""
        return float(self.costmap[idx // self.width, idx % self.width] / 5.0)

    def hasLineOfSight(self, startPoint: Tuple[int, int], endPoint: Tuple[int, int]) -> bool:
        """Check line-of-sight between two points."""
        startX: int
        startY: int
        startX, startY = startPoint
        endX: int
        endY: int
        endX, endY = endPoint

        deltaX: int = abs(endX - startX)
        deltaY: int = abs(endY - startY)
        stepX: int = 1 if startX < endX else -1
        stepY: int = 1 if startY < endY else -1
        error: int = deltaX - deltaY

        currentX: int = startX
        currentY: int = startY

        while currentX != endX or currentY != endY:
            if self.costmap[currentY, currentX] == -1:
                return False
            error2: int = 2 * error
            if error2 > -deltaY:
                error -= deltaY
                currentX += stepX
            if error2 < deltaX:
                error += deltaX
                currentY += stepY

            if not (0 <= currentX < self.width and 0 <= currentY < self.height):
                return False

        return True

    def savePathToCsv(self, path: List[int]) -> None:
        """Save path to CSV file."""
        with open(self.outputCsv, "w", encoding="utf-8") as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(["index", "x_pixel", "y_pixel", "cost"])
            for i, idx in enumerate(path):
                xCoord: int = idx % self.width
                yCoord: int = idx // self.width
                cost: int = self.costmap[yCoord, xCoord]
                writer.writerow([i, xCoord, yCoord, cost])

    def publishCostmap(self) -> None:
        """Publish costmap as ROS OccupancyGrid message."""
        gridMsg = OccupancyGrid()
        gridMsg.header.frame_id = "map"
        gridMsg.info.resolution = self.resolution
        gridMsg.info.width = self.width
        gridMsg.info.height = self.height
        gridMsg.info.origin.position.x = self.originX
        gridMsg.info.origin.position.y = self.originY
        gridMsg.data = self.costmap.flatten().tolist()

        pub = rospy.Publisher("/global_costmap", OccupancyGrid, queue_size=1, latch=True)
        pub.publish(gridMsg)


if __name__ == "__main__":
    try:
        OptimalPathPlanner()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import os
import numpy as np
from nav_msgs.msg import OccupancyGrid
import heapq 
import csv
import itertools
import matplotlib.pyplot as plt
from math import sqrt
from tqdm import tqdm
from sklearn.cluster import KMeans


class OptimalPathPlanner:
    def __init__(self):
        rospy.init_node('optimal_point_selector')
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        
        self.load_parameters()
        self.load_costmap()
        self.all_points = self.get_waypoints()
        
        # Analyze terrain accessibility
        self.accessibility_scores = self.calculate_accessibility()
        
        # Precompute costs with terrain awareness
        self.cost_matrix, self.distance_matrix = self.precompute_all_pairs()
        
        # Find optimal combination
        self.best_combination = self.find_optimal_combination()
        
        # Generate and visualize path
        if self.best_combination:
            full_path = self.generate_combination_path(self.best_combination)
            smoothed_path = self.smooth_path(full_path)
            self.visualize_results(smoothed_path, self.best_combination)
            self.publish_costmap()
        
        plt.ioff()
        plt.show()
        rospy.spin()

    def load_parameters(self):
        self.total_cost_csv = rospy.get_param('~total_cost_csv', 'total_cost.csv')
        self.resolution = rospy.get_param('~resolution', 0.0586901)
        self.origin_x = rospy.get_param('~origin_x', 0.0)
        self.origin_y = rospy.get_param('~origin_y', 0.0)
        self.output_csv = rospy.get_param('~output_csv', 'optimal_path.csv')
        
        # Load 9 possible via points (index 0 is start)
        self.start = (rospy.get_param('~start_x', 380), rospy.get_param('~start_y', 50))
        self.via_points = [
            (rospy.get_param(f'~via{i}_x', 100), rospy.get_param(f'~via{i}_y', 100))
            for i in range(1, 10)
        ]

    def calculate_accessibility(self, radius=5):
        """Calculate accessibility scores based on surrounding terrain costs"""
        scores = []
        for (x, y) in self.all_points:
            x_min = max(0, x - radius)
            x_max = min(self.width, x + radius)
            y_min = max(0, y - radius)
            y_max = min(self.height, y + radius)
            
            region = self.costmap[y_min:y_max, x_min:x_max]
            valid_costs = region[region != -1]
            
            if len(valid_costs) == 0:
                score = 100  # Inaccessible
            else:
                score = np.mean(valid_costs) + 0.5 * np.std(valid_costs)
            
            scores.append(score)
        return np.array(scores)

    def get_waypoints(self):
        return [self.start] + self.via_points

    def pixel_to_index(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return y * self.width + x
        return None

    def load_costmap(self):
        self.costmap = np.loadtxt(self.total_cost_csv, delimiter=',').astype(np.int8)
        self.height, self.width = self.costmap.shape

    def precompute_all_pairs(self):
        """Precompute both cost and distance between all points"""
        n = len(self.all_points)
        cost_matrix = np.full((n, n), np.inf)
        distance_matrix = np.full((n, n), np.inf)
        point_indices = [self.pixel_to_index(x, y) for (x, y) in self.all_points]
        
        for i in tqdm(range(n)):
            for j in range(n):
                if i == j:
                    cost_matrix[i][j] = 0
                    distance_matrix[i][j] = 0
                    continue
                
                path, _, cost = self.astar(point_indices[i], point_indices[j], visualize=False)
                if path:
                    cost_matrix[i][j] = cost
                    distance_matrix[i][j] = self.calculate_path_distance(path)
        
        return cost_matrix, distance_matrix

    def find_optimal_combination(self):
        """Find optimal 4-point combination considering both accessibility and path costs"""
        best_score = float('inf')
        best_combination = None
        
        # Generate all combinations of 4 via points from 9
        all_via_indices = list(range(1, 10))
        combinations = list(itertools.combinations(all_via_indices, 4))
        
        for combo in tqdm(combinations):
            points = [0] + list(combo)  # Start + 4 via points
            try:
                # Calculate combination accessibility score
                access_score = np.mean(self.accessibility_scores[list(combo)])
                
                # Find best permutation for this combination
                min_cost, min_dist, best_sequence = self.find_best_permutation(points)
                
                if best_sequence is None:
                    continue  # Skip invalid combinations
                    
                # Combined score (cost prioritized over distance)
                score = (0.7 * min_cost) + (0.3 * min_dist) + (0.3 * access_score)
                
                if score < best_score:
                    best_score = score
                    best_combination = {
                        'points': combo,
                        'sequence': best_sequence,  # Store optimal sequence
                        'cost': min_cost,
                        'distance': min_dist,
                        'accessibility': access_score
                    }
            except Exception as e:
                rospy.logwarn(f"Skipping combination {combo}: {str(e)}")
        
        rospy.loginfo(f"Best sequence: {best_combination['sequence']}")
        return best_combination
    
    def find_best_permutation(self, points):
        """Find best path sequence for a combination of points based on distance"""
        min_dist = float('inf')
        min_cost = float('inf')
        best_sequence = None
        
        for perm in itertools.permutations(points[1:]):  # All permutations of via points
            sequence = [0] + list(perm) + [0]
            total_dist = 0
            total_cost = 0
            valid = True
            
            for i in range(len(sequence)-1):
                from_idx = sequence[i]
                to_idx = sequence[i+1]
                
                if self.distance_matrix[from_idx][to_idx] == np.inf:
                    valid = False
                    break
                total_dist += self.distance_matrix[from_idx][to_idx]
                total_cost += self.cost_matrix[from_idx][to_idx]
            
            if valid and total_dist < min_dist:
                min_dist = total_dist
                min_cost = total_cost
                best_sequence = sequence  # Store the optimal sequence
        
        return min_cost, min_dist, best_sequence

    
    def calculate_path_distance(self, path):
        """Calculate actual traveled distance from path indices"""
        distance = 0
        for i in range(1, len(path)):
            x1, y1 = path[i-1] % self.width, path[i-1] // self.width
            x2, y2 = path[i] % self.width, path[i] // self.width
            distance += sqrt((x2-x1)**2 + (y2-y1)**2) * self.resolution
        return distance


    def generate_combination_path(self, combination):
        full_path = []
        # Use the precomputed optimal sequence
        sequence = combination['sequence']
        
        for i in range(len(sequence)-1):
            start_idx = self.pixel_to_index(*self.all_points[sequence[i]])
            goal_idx = self.pixel_to_index(*self.all_points[sequence[i+1]])
            
            path, _, _ = self.astar(start_idx, goal_idx, visualize=True)
            if path:
                # Merge paths while avoiding duplicates
                if full_path:
                    full_path.extend(path[1:])  # Skip overlapping point
                else:
                    full_path.extend(path)
        return full_path
    
    def astar(self, start_idx, goal_idx, visualize=True):
        open_heap = []
        heapq.heappush(open_heap, (0, 0, start_idx))
        came_from = {}
        g_score = {start_idx: 0}
        closed_set = set()
        tiebreaker = 1
        last_update = 0
        
        while open_heap:
            current_f, _, current = heapq.heappop(open_heap)
            
            if current == goal_idx:
                return self.reconstruct_path(came_from, current), closed_set, g_score[current]
                
            if current in closed_set:
                continue
            closed_set.add(current)
            
            if visualize and len(closed_set) % 100 == 0:
                self.update_visualization(closed_set)
                last_update = len(closed_set)
            
            for neighbor, cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                    
                tentative_g = g_score[current] + cost + self.get_terrain_cost(neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_score = self.heuristic(neighbor, goal_idx)
                    tiebreaker += 1
                    heapq.heappush(open_heap, (tentative_g + h_score, h_score, neighbor))
                    
        return None, closed_set, float('inf')

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def smooth_path(self, path):
        if len(path) < 2:
            return path
            
        smoothed = [path[0]]
        current_idx = 0
        step_size = 2  # Maintain 2 pixel spacing
        
        while current_idx < len(path) - 1:
            max_reach = min(current_idx + step_size * 2, len(path)-1)
            best_idx = current_idx
            
            for test_idx in range(current_idx + 1, max_reach + 1):
                # Convert indices to x,y coordinates
                current_x = smoothed[-1] % self.width
                current_y = smoothed[-1] // self.width
                test_x = path[test_idx] % self.width
                test_y = path[test_idx] // self.width
                
                if self.has_line_of_sight((current_x, current_y), (test_x, test_y)):
                    best_idx = test_idx
                    
            if best_idx > current_idx:
                smoothed.append(path[best_idx])
                current_idx = best_idx
            else:
                current_idx += 1
                    
        return smoothed

    def update_visualization(self, closed_set, current_path=None):
        self.ax.clear()
        self.ax.imshow(self.costmap, cmap='gray', origin='upper', vmin=-1, vmax=100)
        
        if closed_set:
            ex_x = [idx % self.width for idx in closed_set]
            ex_y = [idx // self.width for idx in closed_set]
            self.ax.scatter(ex_x, ex_y, c='blue', s=1, alpha=0.1, label='Explored')
        
        if current_path:
            px = [idx % self.width for idx in current_path]
            py = [idx // self.width for idx in current_path]
            self.ax.plot(px, py, 'r-', linewidth=2, label='Current Path')
        
        way_x = [p[0] for p in self.all_points]
        way_y = [p[1] for p in self.all_points]
        self.ax.scatter(way_x, way_y, c='green', s=100, marker='o', label='Waypoints')
        self.ax.scatter(self.start[0], self.start[1], c='red', s=150, marker='*', label='Start/End')
        
        self.ax.set_title('Optimal Path Planning')
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)

    def visualize_results(self, path, combination):
        """Enhanced visualization showing accessibility and selection"""
        self.ax.clear()
        
        # Draw costmap with accessibility overlay
        self.ax.imshow(self.costmap, cmap='gray', origin='upper', vmin=-1, vmax=100, alpha=0.7)
        
        # Draw accessibility zones
        for idx, score in enumerate(self.accessibility_scores):
            x, y = self.all_points[idx]
            self.ax.add_patch(plt.Circle((x, y), 10, color=plt.cm.RdYlGn_r(score/100), alpha=0.3))
        
        # Draw all points
        all_x = [p[0] for p in self.all_points]
        all_y = [p[1] for p in self.all_points]
        self.ax.scatter(all_x, all_y, c=self.accessibility_scores, cmap='RdYlGn_r', 
                    s=100, edgecolors='k', vmin=0, vmax=100, label='Waypoints')
        
        # Highlight selected points
        sel_points = [self.all_points[i] for i in combination['points']]
        sel_x = [p[0] for p in sel_points]
        sel_y = [p[1] for p in sel_points]
        self.ax.scatter(sel_x, sel_y, c='blue', s=200, marker='s', edgecolors='k', label='Selected')
        
        # Draw path
        px = [idx % self.width for idx in path]
        py = [idx // self.width for idx in path]
        self.ax.plot(px, py, 'r-', linewidth=2, label='Path')
        
        # Start point
        self.ax.scatter(self.start[0], self.start[1], c='red', s=300, marker='*', label='Start/End')
        
        # Add colorbar and labels
        plt.colorbar(plt.cm.ScalarMappable(cmap='RdYlGn_r'), label='Accessibility Score')
        self.ax.set_title(f"Optimal Path\nCost: {combination['cost']:.1f}, Dist: {combination['distance']:.1f}m")
        self.ax.legend()
        
        plt.draw()
        plt.savefig(f"{os.path.splitext(self.output_csv)[0]}.png")
        plt.pause(2)
        
        # Save to CSV
        self.save_path_to_csv(path)
        
        # Terminal visualization
        rospy.loginfo(f"Final path length: {len(path)} points")
        
        # Add sequence labels
        for i, idx in enumerate(combination['sequence'][1:-1]):
            x, y = self.all_points[idx]
            self.ax.text(x, y, str(i+1), color='white', 
                        ha='center', va='center', fontsize=12)
        
    def get_neighbors(self, idx):
        x, y = idx % self.width, idx // self.width
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    nidx = ny * self.width + nx
                    if self.costmap[ny, nx] != -1:
                        cost = 1.414 if (dx and dy) else 1
                        neighbors.append((nidx, cost))
        return neighbors

    def heuristic(self, a, b):
        return sqrt((a%self.width - b%self.width)**2 + (a//self.width - b//self.width)**2)

    def get_terrain_cost(self, idx):
        return self.costmap[idx // self.width, idx % self.width] / 5.0

    def has_line_of_sight(self, start_point, end_point):
        x0, y0 = start_point
        x1, y1 = end_point
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while x != x1 or y != y1:
            if self.costmap[y, x] == -1:
                return False
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
            # Check if we're still within bounds
            if not (0 <= x < self.width and 0 <= y < self.height):
                return False
                
        return True

    def save_path_to_csv(self, path):
        with open(self.output_csv, 'w') as f:
            writer = csv.writer(f)
            # You could add more columns if needed
            writer.writerow(['index', 'x_pixel', 'y_pixel', 'cost'])
            for i, idx in enumerate(path):
                x = idx % self.width
                y = idx // self.width
                cost = self.costmap[y, x]  # Get the cost value at this pixel
                writer.writerow([i, x, y, cost])

    def publish_costmap(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.data = self.costmap.flatten().tolist()
        
        pub = rospy.Publisher('/global_costmap', OccupancyGrid, queue_size=1, latch=True)
        pub.publish(grid_msg)
    

if __name__ == '__main__':
    try:
        OptimalPathPlanner()
    except rospy.ROSInterruptException:
        pass
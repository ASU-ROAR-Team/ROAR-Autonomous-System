# heightmap_costmap ROS Package

This package provides a ROS node to convert heightmap images into costmaps, which are essential for robotic path planning. It analyzes terrain slope and stability to generate a comprehensive cost representation of the environment.
It takes the RGB map aerial image for calibration node that offers seamless conversion between pixel and real coordiantes throughout the map ;also takes the grayscale flattened image for costmap generaion.Then takes the costmap and given waypoints to calculate the best sequence between choosen 4 waypoints out of the 9 waypoints and draws the best path between them.

## Features

- **Heightmap to Costmap Conversion:** Transforms grayscale heightmap images into traversability costmaps.
- **Terrain Analysis:** Calculates costs based on terrain gradient (slope) and Laplacian (stability/roughness).
- **Configurable Parameters:** Allows customization of scaling factors for gradient and stability costs, as well as image path, resolution, and origin.
- **CSV Output:** Saves computed cost data (total cost, X-axis cost, Y-axis cost) to CSV files for further analysis or use.
- **Visualization:** Generates plots to visualize the total cost and its components (X and Y costs).

## Nodes

### `map_calibration.py & calibration.launch`

This is the starter node of the package. It performs the following functions:

- Initializes a ROS node named `calibration_node`.
- Loads a RGB map image from the specified path.
- Processes the 3 horizontal orange spots for caalibration purposes.
- sets the most leftward orange spot to be as the origin point.
- calculates the resolution & offers seamless conversion betwwen pixel and real world coordinates.
- convert the path.csv if present in the package folder to be real_path.csv to have the real world coordiantes .

### `heightmap_to_costmap.py & costmap.launch`

This is the costmap calculation of the package. It performs the following functions:

- Initializes a ROS node named `heightmap_to_costmap`.
- Loads a heightmap image from the specified path.
- Processes the heightmap to compute gradient and stability metrics.
- Calculates total cost and its X and Y components based on these metrics.
- Saves the computed costs to CSV files.
- Displays a visualization of the costmaps.

### `a_star.py & pathgen1.launch`

This is the main node of the package. It performs the following functions:

- Initializes a ROS node named `optimal_planner`.
- takes 9 hardcoded waypoints from the launch file to select the best combination of these points.
- the waypoints are pixel coordinates that can be minimized to 4 waypoints hardcoded if needed.
- Calculates the best path according to the costmap values and selected waypoints.
- calculates total cost and total distance covered by the final path.
- Displays a visualization of the waypoints and paths.

### Parameters

#### The `map_calibration` node can be configured using the following ROS parameters:

- `~image_path` (string, required): The absolute path to the image file used for calibration. This image must contain distinct, horizontally-aligned red markers.
- `~real_coords` (string, required): A string representation of a list containing the known, real-world coordinates of the markers. These coordinates must be provided in the same left-to-right order as the markers appear in the image.
- `~input_csv` (string, optional): The path to a CSV file containing pixel coordinates that need to be converted to real-world coordinates. The file must include `x_pixel` and `y_pixel` columns. If not provided, this feature is skipped.
- `~output_csv` (string, optional): The path where the output CSV file will be saved after batch processing. The output file will contain the original data plus new `real_x` and `real_y` columns.

#### The heightmap_to_costmap node can be configured using the following ROS parameters:

- `~image_path` (string, default: `$(find heightmap_costmap)/maps/heightmap.png`): The absolute path to the heightmap image file.
- `~gradient_scale` (float, default: `150.0`): A scaling factor applied to the gradient cost component. Higher values increase the impact of slope on the total cost.
- `~stability_scale` (float, default: `90.0`): A scaling factor applied to the stability cost component. Higher values increase the impact of terrain roughness on the total cost.
- `~resolution` (float, default: `0.05`): The resolution of the costmap in meters per pixel.
- `~origin_x` (float, default: `0`): The X-coordinate of the origin of the costmap in meters.
- `~origin_y` (float, default: `0`): The Y-coordinate of the origin of the costmap in meters.

#### The optimal_planner node can be configured using the following ROS parameters:

-   `~total_cost_csv` (string, default: `total_cost.csv`): The absolute path to the input CSV file containing the grid of terrain costs used for pathfinding.
-   `~resolution` (float, default: `0.0586901`): The resolution of the costmap in meters per pixel, used for converting pixel distances to real-world distances.
-   `~origin_x` (float, default: `0.0`): The X-coordinate of the costmap's origin in the world frame (in meters).
-   `~origin_y` (float, default: `0.0`): The Y-coordinate of the costmap's origin in the world frame (in meters).
-   `~start_x` (int, default: `380`): The X pixel coordinate of the primary start and end point for the mission.
-   `~start_y` (int, default: `50`): The Y pixel coordinate of the primary start and end point for the mission.
-   `~via<N>_x` (int, default: `100`): The X pixel coordinate for a potential intermediate waypoint, where `<N>` is a number from 1 to 9.
-   `~via<N>_y` (int, default: `100`): The Y pixel coordinate for a potential intermediate waypoint, where `<N>` is a number from 1 to 9. The planner selects the optimal subset from these points.
-   `~output_csv` (string, default: `optimal_path.csv`): The absolute path for the output CSV file where the final, smoothed path coordinates will be saved.

### Published Topics

This node does not publish any ROS topics directly. It outputs data to CSV files and generates visualizations.

### Subscribed Topics

This node does not subscribe to any ROS topics. It processes a static image file.

## Usage

To run the node and generate a costmap from a heightmap, it is recommended to use the provided launch file:

```bash
roslaunch heightmap_costmap calibration.launch
roslaunch heightmap_costmap costmap.launch
roslaunch heightmap_costmap pathgen1.launch
```



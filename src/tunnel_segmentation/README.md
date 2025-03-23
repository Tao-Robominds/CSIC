# Tunnel Segmentation

This ROS2 package provides tools for visualizing and segmenting tunnel point cloud data from the CSIC dataset.

## Features

- Point cloud conversion from text files to ROS2 messages
- Basic segmentation methods for tunnel point clouds
- Visualization in RViz

## Prerequisites

- ROS2 (tested with Humble)
- PCL library
- RViz2

## Building the Package

```bash
cd /path/to/your/ros2_ws
colcon build --packages-select tunnel_segmentation
source install/setup.bash
```

## Usage

### Visualizing a Point Cloud File

```bash
ros2 launch tunnel_segmentation tunnel_visualization.py data_file:=/path/to/your/point/cloud/file.txt
```

### Parameters

- `data_file`: Path to the point cloud text file
- `frame_id`: Frame ID for the point cloud (default: "map")
- `publish_rate`: Rate at which to publish the point cloud in Hz (default: 1.0)

## Point Cloud Format

The point cloud text files should contain lines with the following space-separated values:
```
x y z intensity label
```

Where:
- `x`, `y`, `z` are the 3D coordinates
- `intensity` is the scanner intensity
- `label` is a class label (0 means unlabeled)

## Advanced Segmentation

The current implementation includes basic segmentation techniques:
1. Downsampling using voxel grid filter
2. Plane segmentation to identify planar surfaces (walls, ground, etc.)
3. Euclidean clustering for grouping points

Future implementations will include more advanced techniques from the referenced papers.

## Visualization

The point clouds are visualized in RViz2 with two displays:
- Original point cloud colored by intensity
- Segmented point cloud with clusters colored differently

## Contributing

Contributions to improve the segmentation methods based on the CSIC research papers are welcome.

## License

This project is licensed under the MIT License - see the LICENSE file for details. 
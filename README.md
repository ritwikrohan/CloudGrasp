# ROS2 Manipulation Project with Vision-Based Object Detection and Grasping

## Project Overview

This project implements an autonomous pick-and-place system using a UR5 robotic arm equipped with a Robotiq gripper and RGB-D camera. The system leverages computer vision techniques including **RANSAC plane segmentation** and **Euclidean clustering** to detect and localize objects in 3D space without hardcoding positions, enabling dynamic object manipulation in both simulated and real environments.

## Demo

### Vision-Based Pick and Place Demonstration

<div align="center">

![Pick and Place Demo](https://i.imgflip.com/a49foz.gif)

*Autonomous object detection and manipulation using RANSAC plane segmentation and Euclidean clustering*

</div>

The demonstration validates the system's ability to autonomously detect, localize, and manipulate objects placed anywhere in the robot's workspace without requiring predetermined positions or manual calibration.

## Key Features

### Vision-Based Perception Pipeline

The core innovation of this project lies in its robust perception pipeline that segments point clouds into objects and support surfaces using advanced filtering techniques:

#### **RANSAC Plane Segmentation**
- **Purpose**: Identifies and removes horizontal support surfaces (tables, shelves) from the point cloud
- **Implementation**: Uses PCL's SACMODEL_PLANE with RANSAC optimization to find planar surfaces
- **Process**:
  1. Iteratively extracts the largest plane from the point cloud
  2. Checks plane orientation using the normal vector's angle with the Z-axis
  3. Horizontal planes (angle < 0.15 rad) are classified as support surfaces
  4. Non-horizontal planes are preserved for potential object segmentation
  5. Continues until no significant planes remain (threshold: 1/8 of points)

#### **Euclidean Clustering**
- **Purpose**: Groups remaining points into distinct object clusters after plane removal
- **Implementation**: Uses PCL's EuclideanClusterExtraction with configurable parameters
- **Key Parameters**:
  - `cluster_tolerance`: 0.01m (minimum separation between objects)
  - `cluster_min_size`: 50 points (filters out noise)
- **Process**:
  1. After removing support surfaces, applies Euclidean distance-based clustering
  2. Each cluster represents a potential graspable object
  3. Calculates cluster centroids to determine which support surface each object rests on

### Object Pose Estimation

The system automatically estimates object poses through geometric analysis:

1. **Shape Fitting**: Fits primitive shapes (boxes, cylinders) to each cluster
2. **Pose Extraction**: Computes object position and orientation from the fitted shape
3. **Transform Chain**: Maintains proper transformations from camera → base_link → world frame
4. **Dynamic Adaptation**: No hardcoded positions - objects can be placed anywhere within the camera's field of view

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    RGB-D Camera                          │
│                 (Point Cloud Input)                      │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Preprocessing Pipeline                      │
│  • Voxel Grid Filtering (5mm resolution)                │
│  • Range Filtering (0-2.5m)                             │
│  • Transform to base_link frame                         │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            RANSAC Plane Segmentation                     │
│  • Extract horizontal support surfaces                   │
│  • Preserve non-horizontal planes                        │
│  • Iterative plane removal                              │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            Euclidean Clustering                          │
│  • Group remaining points into objects                   │
│  • Filter by minimum cluster size                        │
│  • Associate objects with support surfaces               │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│           Shape Extraction & Pose Estimation             │
│  • Fit bounding boxes/cylinders to clusters             │
│  • Extract 6DOF pose (position + orientation)           │
│  • Generate grasp candidates                            │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              MoveIt Motion Planning                      │
│  • Plan collision-free trajectories                     │
│  • Execute pick and place operations                    │
└─────────────────────────────────────────────────────────┘
```

## Package Structure

```
manipulation_project/
├── get_cube_pose/              # Perception client package
│   ├── src/
│   │   └── get_pose_client.cpp # Action client for object detection
│   └── launch/
│       └── get_pose_client.launch.py
│
├── moveit2_scripts/            # Motion planning and control
│   ├── src/
│   │   ├── pick_and_place_perception.cpp      # Simulation pick-place
│   │   ├── pick_and_place_perception_real.cpp # Real robot pick-place
│   │   └── collision_objects.cpp              # Scene management
│   └── launch/
│       ├── pick_and_place_perception.launch.py
│       └── pick_and_place_perception_real.launch.py
│
├── my_moveit_config/           # MoveIt configuration for simulation
└── real_moveit_config/         # MoveIt configuration for real robot

simple_grasping_ros2/           # Core perception library
└── simple_grasping/
    ├── include/
    │   ├── object_support_segmentation.h
    │   ├── shape_extraction.h
    │   └── shape_grasp_planner.h
    └── src/
        ├── basic_grasping_perception.cpp  # Main perception node
        ├── object_support_segmentation.cpp # RANSAC & clustering
        └── shape_extraction.cpp            # Pose estimation
```

## Running the System

### Simulation Mode

1. **Launch Gazebo simulation:**
```bash
ros2 launch my_moveit_config demo.launch.py
```

2. **Start perception and pick-place pipeline:**
```bash
ros2 launch moveit2_scripts pick_and_place_perception.launch.py
```

### Real Robot Mode

1. **Connect to UR5 robot:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py
```

2. **Launch camera driver:**
```bash
ros2 launch realsense2_camera rs_launch.py
```

3. **Start perception and control:**
```bash
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py
```

### Visualization and Debugging

Enable debug topics to visualize segmentation results:
```bash
ros2 run simple_grasping basic_grasping_perception_node --ros-args -p debug_topics:=true
```

This publishes:
- `/object_cloud`: Colored point cloud of detected objects
- `/support_cloud`: Colored point cloud of support surfaces

## Key Advantages of This Approach

### No Hardcoded Positions
- Objects can be placed anywhere in the workspace
- System adapts to different object arrangements
- Robust to environmental changes

### Multi-Object Handling
- Can detect and segment multiple objects simultaneously
- Each object is assigned to its supporting surface
- Prioritizes objects based on distance from robot

### Robust Perception
- Voxel grid filtering reduces computational load
- RANSAC handles noisy sensor data effectively
- Euclidean clustering separates touching objects

### Flexible Grasping
- Generates multiple grasp candidates per object
- Plans grasps based on object shape and orientation
- Adapts gripper opening to object dimensions

## Configuration Parameters

### Perception Parameters
```yaml
# Voxel Grid Filter
voxel_leaf_size: 0.005  # 5mm resolution
voxel_limit_min: -1.0    # Z-axis lower bound
voxel_limit_max: 1.8     # Z-axis upper bound

# Clustering
cluster_tolerance: 0.01   # 10mm minimum separation
cluster_min_size: 50      # Minimum points per object

# RANSAC Plane Segmentation
max_iterations: 100
distance_threshold: 0.01  # 10mm plane fitting tolerance
```

### Grasp Planning Parameters
```yaml
gripper:
  max_opening: 0.110      # Maximum gripper aperture
  finger_depth: 0.02      # Finger insertion depth
  approach:
    min: 0.1             # Minimum approach distance
    desired: 0.15        # Desired approach distance
```

## System Requirements

- **ROS2 Humble**
- **MoveIt2**
- **PCL (Point Cloud Library) 1.11+**
- **Gazebo (for simulation)**
- **Real hardware (optional):**
  - UR5/UR5e robot
  - Robotiq 85 gripper
  - Intel RealSense or compatible RGB-D camera


The RANSAC and Euclidean clustering algorithms provide robust, real-time object detection without requiring pre-defined object positions, making this system adaptable to various manipulation tasks in dynamic environments.

## Contact

**Ritwik Rohan**  
Robotics Engineer | Johns Hopkins MSE '25  
Email: ritwikrohan7@gmail.com  
LinkedIn: [linkedin.com/in/ritwik-rohan](https://linkedin.com/in/ritwik-rohan)

---

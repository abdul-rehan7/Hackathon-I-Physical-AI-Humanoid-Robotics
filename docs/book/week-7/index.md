---
title: Week 7 - Navigation and Path Planning
---

# Week 7: Navigation and Path Planning

## The ROS 2 Navigation Stack (Nav2)
### Nav2 Architecture and Core Components
The ROS 2 Navigation Stack (Nav2) is a powerful and flexible framework for enabling autonomous navigation in mobile robots. It's a complete rewrite of the original ROS 1 navigation stack, designed to leverage the advantages of ROS 2, such as improved performance, security, and real-time capabilities.

**Nav2's Modular Architecture:**
Nav2 is built on a highly modular architecture, allowing developers to easily swap out components and customize the navigation behavior. Key components include:
-   **Behavior Tree:** At the heart of Nav2's decision-making process is a behavior tree, which orchestrates the various navigation tasks (e.g., `FollowPath`, `Spin`, `Wait`). This provides a flexible and human-readable way to define complex robot behaviors.
-   **Planner Server:** Responsible for generating global paths from the robot's current position to a goal. It uses various planning algorithms (e.g., A*, Dijkstra, RRT*) to find optimal or near-optimal paths while considering obstacles.
-   **Controller Server:** Executes the global path by generating local velocity commands for the robot. It uses local planning algorithms (e.g., DWA, TEB) to avoid dynamic obstacles and maintain smooth motion.
-   **Smoother Server:** Optimizes the generated paths for smoothness and efficiency, making them more suitable for robot execution.
-   **Recovery Behaviors:** A set of predefined actions (e.g., `Spin`, `ClearCostmap`) that the robot can execute when it gets stuck or encounters an unexpected situation, allowing it to recover and continue navigation.
-   **Costmap 2D:** A 2D grid map that represents the environment, including static obstacles (from a map) and dynamic obstacles (from sensor readings). It assigns costs to cells, indicating their traversability.
-   **Lifecycle Manager:** Manages the lifecycle of all Nav2 nodes, ensuring proper startup, shutdown, and state transitions.

**How Nav2 Works (Simplified Flow):**
1.  **Goal Reception:** A navigation goal is sent to Nav2.
2.  **Global Planning:** The Planner Server generates a global path from the robot's current location to the goal.
3.  **Local Control:** The Controller Server continuously receives the global path and sensor data, generating local velocity commands to follow the path while avoiding obstacles.
4.  **Feedback and Recovery:** The Behavior Tree monitors the navigation process, providing feedback and triggering recovery behaviors if necessary.

Nav2's robust and extensible architecture makes it a powerful tool for developing advanced autonomous navigation capabilities in a wide range of robotic applications.
### Configuring and Launching Nav2
Setting up and launching Nav2 involves configuring various parameters and using ROS 2 launch files to start the necessary nodes. This section will guide you through the typical steps.

**1. Configuration Files (YAML):**
Nav2 components are highly configurable through YAML files. These files define parameters for:
-   **Costmaps:** `global_costmap.yaml`, `local_costmap.yaml` (e.g., robot radius, inflation layer, static map parameters, sensor sources).
-   **Planners:** `planner_server.yaml` (e.g., A* parameters, Dijsktra parameters).
-   **Controllers:** `controller_server.yaml` (e.g., DWA parameters, PID gains).
-   **Recovery Behaviors:** `recovery_server.yaml` (e.g., spin parameters, clear costmap parameters).
-   **Behavior Tree:** `bt_navigator.yaml` (defines the behavior tree XML file to use).

**Example `global_costmap.yaml` snippet:**
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_topic: map
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_range: 3.0
          obstacle_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
```

**2. Launch Files (Python):**
A typical Nav2 launch file orchestrates the startup of all Nav2 nodes, loads their configurations, and sets up necessary transformations.

**Example `nav2_bringup.launch.py` (simplified):**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='map.yaml',
        description='Full path to map yaml file to load')

    # Get the launch directory
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = PathJoinSubstitution([nav2_bringup_dir, 'launch'])

    # Include the Nav2 bringup launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_launch_dir, 'bringup_launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        map_yaml_cmd,
        nav2_launch
    ])
```
This simplified example shows how to include the main Nav2 bringup launch file and pass a map argument. By customizing these configuration and launch files, you can adapt Nav2 to various robot platforms and environments.

## SLAM and Localization
### SLAM Basics
Simultaneous Localization and Mapping (SLAM) is a fundamental problem in robotics where a robot builds a map of an unknown environment while simultaneously localizing itself within that map. It's a chicken-and-egg problem: you need a map to localize, and you need to localize to build a map.

**The SLAM Problem:**
Imagine a robot exploring a new building. It doesn't know where it is, nor does it have a map of the building. As it moves, it uses its sensors (e.g., LiDAR, cameras) to perceive its surroundings. SLAM algorithms process this sensor data to:
1.  **Build a map:** Create a representation of the environment (e.g., an occupancy grid, a point cloud).
2.  **Localize itself:** Determine its own position and orientation within that newly created map.

**Key Components of a SLAM System:**
-   **Front-end (Perception and Feature Extraction):** Processes raw sensor data to extract meaningful features (e.g., corners, edges, keypoints from images; points from LiDAR scans). It also performs data association, matching current observations with previous ones.
-   **Back-end (Optimization):** Takes the processed sensor data and robot motion estimates from the front-end and optimizes the robot's trajectory and the map simultaneously. This is often done using techniques like graph optimization, where robot poses and landmark locations are nodes in a graph, and sensor measurements and motion estimates are edges.
-   **Loop Closure Detection:** A critical component that recognizes when the robot has returned to a previously visited location. This helps correct for accumulated errors (drift) in the map and robot trajectory, leading to a globally consistent map.
-   **Map Representation:** The way the environment is represented. Common types include:
    -   **Occupancy Grid Maps:** 2D grid where each cell represents the probability of being occupied, free, or unknown.
    -   **Feature-based Maps:** Stores a sparse set of distinct features (landmarks) in the environment.
    -   **Point Cloud Maps:** A dense collection of 3D points representing the environment.

**Challenges in SLAM:**
-   **Sensor Noise:** All sensors have noise, leading to inaccuracies in measurements.
-   **Dynamic Environments:** Moving objects can confuse SLAM algorithms.
-   **Computational Complexity:** SLAM can be computationally intensive, especially in large environments.
-   **Data Association:** Correctly matching current observations with existing map features is crucial and challenging.

SLAM is a cornerstone of autonomous robotics, enabling robots to operate in unknown environments without prior knowledge. It's a continuously evolving field with new algorithms and techniques emerging regularly.
### Advanced Localization Techniques
While SLAM provides a way to build a map and localize simultaneously, dedicated localization techniques are often employed when a map is already available or when higher accuracy and robustness are required. These techniques focus solely on determining the robot's pose within a known map.

**Key Advanced Localization Techniques:**
-   **Monte Carlo Localization (MCL) / Particle Filter Localization:**
    -   Represents the robot's pose probability distribution as a set of weighted particles.
    -   Particles are updated based on motion models and sensor observations.
    -   Effective for global localization (kidnapped robot problem) and robust against sensor noise.
    -   Widely used in ROS 2 with the `amcl` (Adaptive Monte Carlo Localization) package.
-   **Kalman Filters (KF) and Extended Kalman Filters (EKF):**
    -   **Kalman Filter:** A recursive algorithm that estimates the state of a system (e.g., robot's pose) from a series of noisy measurements. Assumes linear system dynamics and Gaussian noise.
    -   **Extended Kalman Filter:** An extension of KF for non-linear systems, linearizing the system around the current state estimate. Commonly used for fusing data from multiple sensors (e.g., odometry, IMU, GPS).
-   **Unscented Kalman Filters (UKF):**
    -   Another extension of KF for non-linear systems that uses a deterministic sampling technique (unscented transform) to approximate the probability distribution, often providing better performance than EKF for highly non-linear systems.
-   **Graph-based Localization:**
    -   Similar to the back-end of graph-based SLAM, but with a fixed map. The robot's trajectory is optimized by minimizing errors between sensor observations and the known map.
-   **Visual Localization:**
    -   Uses camera images to localize the robot within a pre-built visual map. Techniques include feature matching (e.g., SIFT, ORB) against a database of images with known poses.
-   **LiDAR-based Localization:**
    -   Uses LiDAR scans to match against a pre-built LiDAR map. Techniques like ICP (Iterative Closest Point) are often used to align current scans with the map.

**Sensor Fusion for Robust Localization:**
Combining data from heterogeneous sensors (e.g., LiDAR, cameras, IMU, odometry, GPS) is crucial for achieving robust and accurate localization in diverse environments. Techniques like Kalman filters and particle filters are commonly used to fuse these sensor inputs, compensating for the limitations of individual sensors.

Advanced localization techniques are essential for ensuring that autonomous robots can reliably determine their position in the world, even in challenging and dynamic conditions, which is critical for safe and effective operation.

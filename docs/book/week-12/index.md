---
title: Week 12 - Capstone Project - Part 1
---

# Week 12: Capstone Project - Part 1

## Project Scoping and Setup
### Defining Project Goals and Requirements
The Capstone Project is an opportunity to apply the knowledge and skills gained throughout the course to a real-world robotics problem. The first crucial step is to clearly define the project's goals and requirements. A well-defined scope ensures that the project remains focused, achievable, and delivers meaningful results.

**1. Identify the Problem/Challenge:**
-   What specific robotics problem are you trying to solve? (e.g., autonomous navigation in a warehouse, object manipulation in a kitchen, human-robot collaboration in an assembly line).
-   What are the current limitations or pain points that your project aims to address?

**2. Define High-Level Goals:**
-   What is the ultimate objective of your project? (e.g., "Develop a robot that can autonomously deliver items in an office environment," "Create a system for a robotic arm to sort objects by color").
-   These should be broad statements that capture the essence of the project.

**3. Break Down into Specific Requirements:**
Translate high-level goals into measurable and verifiable requirements. These can be categorized as:

-   **Functional Requirements:** What the system *must do*.
    -   **Robot Capabilities:** (e.g., "The robot must be able to navigate autonomously," "The robot must be able to grasp objects of varying sizes").
    -   **Perception:** (e.g., "The robot must detect obstacles within 2 meters," "The robot must recognize specific objects").
    -   **Interaction:** (e.g., "The robot must respond to voice commands," "The robot must provide visual feedback").
-   **Non-Functional Requirements:** How well the system performs.
    -   **Performance:** (e.g., "The robot must complete the task within 5 minutes," "The navigation accuracy must be within 5 cm").
    -   **Reliability:** (e.g., "The system must operate without failure for 95% of the time").
    -   **Safety:** (e.g., "The robot must avoid collisions with humans and objects").
    -   **Usability:** (e.g., "The human-robot interface must be intuitive").
    -   **Scalability:** (e.g., "The system should be able to handle an increased number of objects/tasks").

**4. Define Scope Boundaries:**
-   **In-Scope:** Clearly list what features and functionalities will be included in the project.
-   **Out-of-Scope:** Explicitly state what will *not* be included. This helps manage expectations and prevents scope creep. (e.g., "Advanced machine learning model training will be out of scope; pre-trained models will be used").

**5. Success Criteria:**
-   How will you measure the success of your project? Define clear metrics. (e.g., "Successful delivery rate of 90%," "Average task completion time of X seconds").

**Example Project Idea:**
**Problem:** Manual delivery of small packages in a multi-floor office building is inefficient.
**High-Level Goal:** Develop an autonomous mobile robot to deliver packages between offices.

**Functional Requirements:**
-   Navigate autonomously between designated offices.
-   Detect and avoid static and dynamic obstacles.
-   Interact with an elevator system.
-   Notify sender/receiver upon delivery.

**Non-Functional Requirements:**
-   Delivery time: Max 10 minutes per delivery.
-   Collision rate: Less than 1 per 100 deliveries.
-   Payload: Up to 2 kg.

By meticulously defining these aspects, you lay a solid foundation for a successful Capstone Project.
### Setting Up the Development Environment and Tools
A well-configured development environment is crucial for the success of your Capstone Project. This involves installing the necessary software, configuring your workspace, and familiarizing yourself with the tools you'll be using.

**1. Operating System:**
-   **Ubuntu (20.04 LTS or newer):** Recommended for ROS 2 development due to its widespread support and compatibility.
-   **Windows/macOS:** While ROS 2 supports these platforms, Linux (Ubuntu) is generally preferred for robotics development.

**2. ROS 2 Installation:**
-   Install the appropriate ROS 2 distribution (e.g., Foxy, Galactic, Humble) based on your project requirements and hardware. Follow the official ROS 2 documentation for installation instructions.
-   Ensure you source your ROS 2 setup script in your `.bashrc` or `.zshrc` file.

**3. Integrated Development Environment (IDE):**
-   **VS Code:** A popular choice for ROS 2 development due to its extensive extensions for C++, Python, Docker, and ROS.
    -   **Recommended Extensions:** `ms-vscode.cpptools`, `ms-python.python`, `ms-azuretools.vscode-docker`, `ms-iot.vscode-ros`.
-   **CLion (for C++):** A powerful IDE for C++ development, offering excellent debugging and code analysis features.

**4. Simulation Environment:**
-   **Gazebo:** For simulating robot dynamics, sensors, and environments. Ensure it's correctly integrated with your ROS 2 setup.
-   **NVIDIA Isaac Sim:** If your project involves advanced AI, GPU-accelerated simulation, or synthetic data generation, Isaac Sim is a powerful alternative or complement to Gazebo.

**5. Version Control:**
-   **Git:** Essential for managing your codebase, collaborating with teammates, and tracking changes.
-   **GitHub/GitLab/Bitbucket:** Use a remote repository to store your code and facilitate collaboration.

**6. Robotics-Specific Tools:**
-   **RViz:** For visualizing robot models, sensor data, and navigation plans.
-   **RQt:** A suite of GUI tools for debugging and introspection of ROS 2 systems (e.g., `rqt_graph`, `rqt_plot`, `rqt_console`).
-   **Colcon:** The build tool for ROS 2 packages.
-   **ROS 2 CLI Tools:** Familiarize yourself with commands like `ros2 run`, `ros2 topic`, `ros2 node`, `ros2 param`, etc.

**7. Programming Languages and Libraries:**
-   **Python:** Widely used for high-level control, scripting, and AI development in ROS 2.
-   **C++:** For performance-critical components and low-level control.
-   **OpenCV:** For computer vision tasks.
-   **NumPy/SciPy:** For numerical operations.
-   **TensorFlow/PyTorch:** For deep learning models.

**Setting up your Workspace:**
1.  **Create a ROS 2 workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
2.  **Clone your project repository:**
    ```bash
    git clone <your_repository_url>
    ```
3.  **Build your workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
4.  **Source the setup file:**
    ```bash
    source install/setup.bash
    ```
By carefully setting up your development environment, you'll have all the necessary tools at your disposal to efficiently develop and test your Capstone Project.

## Implementing the Perception and Planning Pipeline
### Setting Up Sensors and Data Acquisition
The perception pipeline is the robot's window to the world. For your Capstone Project, setting up and correctly configuring the sensors and data acquisition is paramount. This involves both hardware (real or simulated) and software components to get reliable data into your ROS 2 system.

**1. Sensor Selection and Integration:**
-   **Review Requirements:** Based on your project goals, identify the necessary sensors (e.g., cameras for object detection, LiDAR for mapping, IMU for localization).
-   **Hardware Integration (Real Robot):**
    -   Physically mount sensors on the robot.
    -   Connect sensors to the robot's computing platform (e.g., NVIDIA Jetson, industrial PC).
    -   Install necessary drivers and ROS 2 packages for each sensor (e.g., `ros2_intel_realsense`, `velodyne_driver`).
-   **Simulation Integration (Gazebo/Isaac Sim):**
    -   Modify your robot's URDF/XACRO model to include the simulated sensors (as covered in Week 4).
    -   Ensure the Gazebo/Isaac Sim plugins for these sensors are correctly configured to publish data to ROS 2 topics.

**2. Data Acquisition and ROS 2 Topics:**
-   **Identify Sensor Topics:** Each sensor will publish its data to specific ROS 2 topics. Use `ros2 topic list` to identify them (e.g., `/camera/image_raw`, `/scan`, `/imu/data`).
-   **Verify Data Streams:** Use `ros2 topic echo <topic_name>` to inspect the data being published by each sensor.
-   **Visualization in RViz:** Launch RViz and add appropriate displays (e.g., `Image`, `LaserScan`, `PointCloud2`, `IMU`) to visualize the sensor data in real-time. This is crucial for debugging and ensuring sensors are working as expected.

**3. Data Preprocessing:**
Raw sensor data often needs preprocessing before it can be used by perception algorithms.
-   **Image Processing:**
    -   **Debayering:** For raw camera images.
    -   **Rectification:** Correcting lens distortion using camera calibration parameters.
    -   **Filtering:** Noise reduction (e.g., Gaussian blur).
-   **LiDAR/Point Cloud Processing:**
    -   **Filtering:** Removing outliers, downsampling (e.g., VoxelGrid filter).
    -   **Ground Plane Removal:** Separating the ground from objects.
-   **IMU Processing:**
    -   **Filtering:** Complementary filters or Kalman filters to fuse IMU data with other sources for robust orientation estimation.

**4. Time Synchronization:**
-   Ensure all sensor data is properly time-synchronized. ROS 2 uses timestamps in messages, and tools like `ros2 bag` can record synchronized data.
-   NTP (Network Time Protocol) can be used to synchronize clocks across multiple computers in a distributed robotics system.

**Example: Launching a RealSense Camera in ROS 2:**
```bash
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true enable_imu:=true
```
This command launches the RealSense camera node, publishing color, depth, and IMU data to various ROS 2 topics.

By meticulously setting up your sensors and data acquisition pipeline, you provide the foundation for your robot's perception capabilities, which is critical for any autonomous task.
### Integrating Perception with Navigation and Manipulation
The true power of a robotic system emerges when its perception capabilities are seamlessly integrated with its planning and action modules. For your Capstone Project, this means connecting the sensor data and processed environmental understanding to drive the robot's navigation and manipulation behaviors.

**1. Perception for Navigation:**
-   **Obstacle Detection:** Process LiDAR and depth camera data to identify obstacles and update the Nav2 costmaps. This allows the robot to avoid collisions during navigation.
-   **Localization:** Use sensor data (e.g., LiDAR scans, visual features) to localize the robot within a pre-built map or to perform SLAM in unknown environments.
-   **Semantic Information:** If your project involves semantic navigation (e.g., "go to the kitchen"), perception modules can identify semantic landmarks or regions in the environment.

**Integration Steps:**
-   **ROS 2 Topics:** Ensure your perception nodes publish processed data (e.g., `sensor_msgs/PointCloud2`, `sensor_msgs/LaserScan`) to the topics that Nav2's costmap layers subscribe to.
-   **TF (Transformations):** Maintain an accurate TF tree to ensure all sensor data and robot poses are in a consistent coordinate frame.
-   **Nav2 Configuration:** Configure Nav2's costmap plugins to use your sensor sources for obstacle detection and clearing.

**2. Perception for Manipulation:**
-   **Object Detection and Pose Estimation:** Use computer vision techniques (e.g., deep learning models) to detect objects of interest and estimate their 6D poses (position and orientation) relative to the robot.
-   **Grasping Point Detection:** Based on the object's pose and geometry, determine suitable grasping points for the robot's end-effector.
-   **Collision Avoidance:** Continuously monitor the manipulation workspace for obstacles (including the robot's own body) using perception data to prevent collisions during grasping and placing.

**Integration Steps:**
-   **MoveIt Planning Scene:** Publish detected objects and their poses to the MoveIt Planning Scene. MoveIt can then use this information for collision checking and motion planning.
-   **Inverse Kinematics:** Use the estimated object poses as targets for inverse kinematics solvers to calculate the required joint configurations for grasping.
-   **Feedback Loop:** After a manipulation attempt, use perception to verify the success of the grasp or placement and trigger recovery behaviors if necessary.

**3. High-Level Planning with LLMs/VLMs:**
-   **Natural Language Grounding:** If your project uses LLMs or VLMs, perception provides the visual context for grounding natural language commands (e.g., identifying "the red block" from an image).
-   **State Updates:** Perception continuously updates the LLM/VLM with the current state of the environment, allowing for dynamic replanning and adaptation.

**Example: Object Detection to Grasping Workflow:**
1.  **Camera captures image.**
2.  **Object detection model identifies "blue_block" and its 2D bounding box.**
3.  **Depth camera data is used to estimate the 3D pose of "blue_block".**
4.  **MoveIt receives the 3D pose of "blue_block" and plans a collision-free path for the robot arm to grasp it.**
5.  **Robot executes the grasping trajectory.**
6.  **Camera verifies successful grasp.**

By effectively integrating these perception and planning components, your Capstone Project robot will be able to intelligently interact with its environment, fulfilling its defined goals autonomously.

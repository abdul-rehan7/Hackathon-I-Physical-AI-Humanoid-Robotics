---
title: Week 4 - Sensors and Perception in Simulation
---

# Week 4: Sensors and Perception in Simulation

## Simulating Cameras, LiDAR, and IMUs
### Simulating Cameras in Gazebo
Simulating cameras in Gazebo is crucial for developing and testing computer vision algorithms for robotics. Gazebo provides various camera types, including RGB, depth, and stereo cameras, each with configurable parameters to mimic real-world sensors.

**Key Camera Parameters:**
-   **`image_size`:** Resolution of the captured image (width, height).
-   **`horizontal_fov`:** Horizontal field of view.
-   **`near` and `far` clip:** Minimum and maximum distances for rendering.
-   **`update_rate`:** How frequently the camera publishes new images.
-   **`camera_info`:** Publishes camera calibration parameters.

**Adding a Camera to a URDF/XACRO Model:**
Cameras are typically added as a link and joint to your robot's URDF or XACRO description. A Gazebo plugin is then used to simulate the camera's behavior and publish its data.

**Example Camera Definition in XACRO:**
```xml
<xacro:macro name="camera_macro" params="prefix parent_link x_offset y_offset z_offset">
  <link name="${prefix}_camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="${prefix}_camera_joint" type="fixed">
    <parent link="${parent_link}"/>
    <child link="${prefix}_camera_link"/>
    <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 0 0"/>
  </joint>

  <gazebo reference="${prefix}_camera_link">
    <sensor name="${prefix}_camera" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="${prefix}_camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>${prefix}</namespace>
          <argument>--ros-args --remap __tf_prefix:=$(arg tf_prefix)</argument>
        </ros>
        <camera_name>${prefix}_camera</camera_name>
        <frame_name>${prefix}_camera_link</frame_name>
        <hack_baseline>0.07</hack_baseline>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```
This XACRO macro defines a camera link, joint, and a Gazebo sensor plugin that publishes camera data to ROS 2 topics. This setup allows your robot to "see" its simulated environment, enabling the development of perception algorithms.
### Simulating LiDAR and IMUs in Gazebo
Beyond cameras, LiDAR (Light Detection and Ranging) and IMUs (Inertial Measurement Units) are critical sensors for robot navigation and state estimation. Gazebo provides robust simulation capabilities for both.

**Simulating LiDAR:**
LiDAR sensors provide 3D point cloud data, essential for mapping and obstacle avoidance. Gazebo's `ray` sensor type, combined with a `libgazebo_ros_ray_sensor.so` plugin, can simulate LiDAR.

**Example LiDAR Definition in XACRO:**
```xml
<xacro:macro name="lidar_macro" params="prefix parent_link x_offset y_offset z_offset">
  <link name="${prefix}_lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="${prefix}_lidar_joint" type="fixed">
    <parent link="${parent_link}"/>
    <child link="${prefix}_lidar_link"/>
    <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 0 0"/>
  </joint>

  <gazebo reference="${prefix}_lidar_link">
    <sensor name="${prefix}_lidar" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14</min_angle>
            <max_angle>3.14</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="${prefix}_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <argument>~/out</argument>
          <namespace>${prefix}</namespace>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>${prefix}_lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

**Simulating IMUs:**
IMUs provide linear acceleration and angular velocity data, crucial for dead reckoning and sensor fusion. Gazebo's `imu` sensor type, combined with a `libgazebo_ros_imu_sensor.so` plugin, simulates IMU data.

**Example IMU Definition in XACRO:**
```xml
<xacro:macro name="imu_macro" params="prefix parent_link x_offset y_offset z_offset">
  <link name="${prefix}_imu_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>

  <joint name="${prefix}_imu_joint" type="fixed">
    <parent link="${parent_link}"/>
    <child link="${prefix}_imu_link"/>
    <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 0 0"/>
  </joint>

  <gazebo reference="${prefix}_imu_link">
    <sensor name="${prefix}_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100.0</update_rate>
      <plugin name="${prefix}_imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>${prefix}</namespace>
          <argument>~/out</argument>
        </ros>
        <frame_name>${prefix}_imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```
These examples illustrate how to integrate simulated LiDAR and IMU sensors into your robot models, providing the necessary data streams for developing advanced perception and navigation systems.

## Data Collection and Visualization with RViz
### Collecting Sensor Data from Gazebo
Once sensors are integrated into your Gazebo robot model, they will publish data to specific ROS 2 topics. Collecting and understanding this data is the first step in developing perception algorithms.

**Key ROS 2 Topics for Sensor Data:**
-   **Camera:**
    -   `/camera/image_raw`: Raw image data (sensor_msgs/Image).
    -   `/camera/camera_info`: Camera calibration parameters (sensor_msgs/CameraInfo).
    -   `/camera/depth/image_raw`: Raw depth image data (sensor_msgs/Image).
    -   `/camera/depth/points`: 3D point cloud from depth camera (sensor_msgs/PointCloud2).
-   **LiDAR:**
    -   `/lidar/scan`: 2D laser scan data (sensor_msgs/LaserScan).
    -   `/lidar/point_cloud`: 3D point cloud data (sensor_msgs/PointCloud2).
-   **IMU:**
    -   `/imu/data`: IMU data (sensor_msgs/Imu) including orientation, angular velocity, and linear acceleration.

**Subscribing to Sensor Topics (Python Example):**
You can create ROS 2 nodes to subscribe to these topics and process the incoming data.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge # For converting ROS Image messages to OpenCV images

class SensorDataCollector(Node):
    def __init__(self):
        super().__init__('sensor_data_collector')
        self.bridge = CvBridge()

        # Image subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.image_subscription  # prevent unused variable warning

        # LiDAR subscriber
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )
        self.lidar_subscription # prevent unused variable warning

        # IMU subscriber
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.imu_subscription # prevent unused variable warning

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info('Received image frame')
            # Process image here (e.g., save, display, run CV algorithm)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def lidar_callback(self, msg):
        self.get_logger().info(f'Received LiDAR scan with {len(msg.ranges)} ranges')
        # Process LiDAR data here (e.g., obstacle detection, mapping)

    def imu_callback(self, msg):
        self.get_logger().info(f'Received IMU data: Orientation={msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}')
        # Process IMU data here (e.g., state estimation)

def main(args=None):
    rclpy.init(args=args)
    sensor_data_collector = SensorDataCollector()
    rclpy.spin(sensor_data_collector)
    sensor_data_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This node demonstrates how to subscribe to various sensor topics and provides placeholders for processing the data. This forms the basis for any robot perception system.
### Visualizing Sensor Data in RViz
RViz (ROS Visualization) is a 3D visualizer for displaying sensor data, robot models, and other information from a ROS 2 system. It's an indispensable tool for debugging, monitoring, and understanding the state of your robot and its environment.

**Key RViz Features for Sensor Data Visualization:**
-   **RobotModel:** Displays the robot's URDF/XACRO model.
-   **Image:** Visualizes camera feeds.
-   **LaserScan:** Displays 2D LiDAR data.
-   **PointCloud2:** Visualizes 3D point cloud data from depth cameras or 3D LiDAR.
-   **IMU:** Shows IMU data, including orientation.
-   **TF (Transformations):** Displays the coordinate frames of the robot and its environment.

**Steps to Visualize Sensor Data in RViz:**
1.  **Launch RViz:**
    ```bash
    rviz2
    ```
2.  **Add Displays:** In the RViz interface, click "Add" in the "Displays" panel.
    -   Select "RobotModel" and ensure your robot's URDF is loaded (often handled by a launch file).
    -   For camera images, select "Image" and set the "Image Topic" to `/camera/image_raw`.
    -   For LiDAR data, select "LaserScan" or "PointCloud2" and set the appropriate topic (e.g., `/lidar/scan` or `/lidar/point_cloud`).
    -   For IMU data, select "IMU" and set the "Topic" to `/imu/data`.
3.  **Configure Topics and Frames:** Ensure the correct ROS 2 topics are selected for each display and that the "Fixed Frame" in RViz is set to a common frame (e.g., `odom` or `map`).

By visualizing sensor data in RViz, you can gain immediate insights into your robot's perception capabilities, verify sensor configurations, and debug issues effectively. This visual feedback loop is critical for developing robust robotic systems.

---
title: Week 3 - Introduction to Simulation with Gazebo
---

# Week 3: Introduction to Simulation with Gazebo

## Building a Simulated World
### Gazebo Basics and World Creation
Gazebo is a powerful 3D robotics simulator that allows you to accurately and efficiently test algorithms, design robots, and perform regression testing. It offers the ability to simulate complex environments, including physics, sensors, and objects.

**Key features of Gazebo:**
- **Physics Engine:** Simulates rigid body dynamics, gravity, friction, and collisions.
- **Sensors:** Supports a wide range of simulated sensors, including cameras, LiDAR, IMUs, and more.
- **Models:** Allows importing and creating 3D models of robots and environments.
- **Plugins:** Extensible architecture for custom functionality and integration with ROS 2.

**Creating a Simple World:**
Gazebo worlds are defined using SDF (Simulation Description Format) files. A basic world file (`my_world.sdf`) can define the ground plane, lighting, and simple objects:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```
This SDF defines a world with a sun, a ground plane, and a simple 1x1x1 meter box. You can launch this world in Gazebo using `gazebo my_world.sdf`. This provides a foundational understanding for building more complex simulated environments.
### Advanced World Features and Plugins
Beyond basic static objects, Gazebo allows for the creation of highly interactive and dynamic simulated environments. This includes:

-   **Dynamic Objects:** Incorporating objects that can be manipulated by the robot or interact with the physics engine (e.g., spheres, cylinders, custom meshes).
-   **Environmental Effects:** Simulating various environmental conditions like wind, fog, or different lighting scenarios to test robot robustness.
-   **Sensors in World Files:** Directly embedding sensor definitions within the SDF world file to quickly set up and test sensor configurations without modifying robot models.
-   **Gazebo Plugins:** Extending Gazebo's functionality through plugins. These are shared libraries that can be loaded into Gazebo to add custom behaviors, control physics, or interface with external systems. Common types include:
    -   **Model Plugins:** Attach to specific models to control their behavior (e.g., a plugin to make a door open when a robot approaches).
    -   **World Plugins:** Affect the entire simulation world (e.g., a plugin to generate random obstacles).
    -   **System Plugins:** Provide global control over Gazebo (e.g., a plugin for data logging).

**Example of a simple plugin integration (conceptual):**
```xml
<model name="my_robot">
  <plugin name="my_robot_controller" filename="libmy_robot_controller.so"/>
  <!-- ... other model definitions ... -->
</model>
```
This conceptual example shows how a plugin (`libmy_robot_controller.so`) could be loaded with a robot model to add custom control logic. Mastering these advanced features allows for the creation of realistic and challenging simulation scenarios for agentic AI and robotics development.

## Adding and Configuring Robot Models
### URDF for Robot Modeling
The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. It's a powerful way to define the robot's kinematic and dynamic properties, visual appearance, and collision geometry. URDF files are essential for visualizing robots in tools like RViz and for simulating them in Gazebo.

**Key elements of a URDF file:**
-   **`<robot>`:** The root element, defining the robot's name.
-   **`<link>`:** Represents a rigid body part of the robot (e.g., base, arm segment, wheel). Each link has:
    -   **`<visual>`:** Defines the visual properties (geometry, material, origin).
    -   **`<collision>`:** Defines the collision properties (geometry, origin).
    -   **`<inertial>`:** Defines the mass, center of mass, and inertia matrix.
-   **`<joint>`:** Represents a connection between two links, defining their relative motion (e.g., revolute, prismatic, fixed). Each joint has:
    -   **`parent` and `child` links:** Specifies which links it connects.
    -   **`type`:** Defines the joint's degree of freedom.
    -   **`<origin>`:** Defines the joint's position and orientation relative to the parent link.
    -   **`<axis>`:** Defines the axis of rotation or translation for revolute/prismatic joints.
    -   **`<limit>`:** Defines the joint's movement limits (e.g., position, velocity, effort).

**Example of a simple URDF link and joint:**
```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="link_1">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_link_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="100"/>
  </joint>
</robot>
```
This URDF defines a robot with a `base_link` and a `link_1` connected by a revolute joint. This foundational knowledge is crucial for accurately representing and simulating robots.
### XACRO for Modular Robot Descriptions
While URDF is powerful, it can become cumbersome for complex robots with many repetitive structures or when you want to reuse components. XACRO (XML Macros) is an XML macro language that allows you to write more concise and modular robot descriptions by using macros, properties, and mathematical expressions. XACRO files are processed to generate a standard URDF file.

**Key advantages of XACRO:**
-   **Modularity:** Define reusable components (e.g., a wheel, a sensor) as macros and include them multiple times.
-   **Parameters:** Use properties to define parameters (e.g., link dimensions, joint limits) that can be easily changed.
-   **Mathematical Expressions:** Perform calculations within the XML to derive values, reducing manual errors.
-   **Readability:** Improves the readability and maintainability of complex robot descriptions.

**Example of a simple XACRO macro:**
```xml
<?xml version="1.0"?>
<robot name="my_modular_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="PI" value="3.14159265359"/>
  <xacro:property name="wheel_radius" value="0.03"/>
  <xacro:property name="wheel_width" value="0.02"/>

  <xacro:macro name="wheel_macro" params="prefix parent_link x_offset y_offset z_offset">
    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent_link}"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="${PI/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <link name="base_link"/>

  <xacro:wheel_macro prefix="left" parent_link="base_link" x_offset="0" y_offset="0.1" z_offset="0"/>
  <xacro:wheel_macro prefix="right" parent_link="base_link" x_offset="0" y_offset="-0.1" z_offset="0"/>

</robot>
```
This XACRO example defines a `wheel_macro` that can be reused to add multiple wheels to a robot, demonstrating the power of modularity. XACRO is an indispensable tool for managing complex robot models in simulation and real-world applications.

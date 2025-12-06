---
title: Week 1 - Intro to Agentic AI & Robotics
---

# Week 1: Introduction to Agentic AI and Robotics

## The Modern Robotics Stack
### Core Components of the Robotics Stack
The modern robotics stack is a complex interplay of hardware and software components designed to enable autonomous operation. At its core, it typically includes:
- **Perception:** Sensors (cameras, LiDAR, IMUs) gather data about the environment.
- **Localization and Mapping:** Algorithms process sensor data to determine the robot's position and create a map of its surroundings (SLAM).
- **Path Planning:** Based on the map and goal, the robot plans a collision-free path.
- **Motion Control:** Executes the planned path by sending commands to actuators (motors, servos).
- **Human-Robot Interaction (HRI):** Interfaces for humans to interact with and supervise the robot.
- **Artificial Intelligence/Machine Learning:** Increasingly, AI/ML models are integrated for advanced perception, decision-making, and task execution.
### Evolution of Robotics and Agentic AI
Robotics has evolved significantly from industrial manipulators performing repetitive tasks to complex autonomous systems operating in dynamic environments. This evolution has been driven by advancements in:
- **Computational Power:** More powerful processors and GPUs enable sophisticated algorithms.
- **Sensor Technology:** Miniaturized and more accurate sensors provide richer environmental data.
- **Artificial Intelligence:** Machine learning, especially deep learning, has revolutionized perception, decision-making, and control.
- **Agentic AI:** The emergence of agentic AI, where AI systems can reason, plan, and act autonomously to achieve goals, is pushing the boundaries of what robots can do. This involves integrating large language models (LLMs) and vision-language models (VLMs) into robotic systems for higher-level cognitive abilities and more natural human-robot interaction.

## Introduction to ROS 2
### Core Concepts of ROS 2
ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. Key concepts include:
- **Nodes:** Executable processes that perform computation (e.g., a node to read sensor data, a node to control motors).
- **Topics:** A mechanism for nodes to exchange messages asynchronously. Nodes publish messages to topics, and other nodes subscribe to those topics to receive the messages.
- **Services:** A request/reply mechanism for synchronous communication between nodes. A client node sends a request, and a service node processes it and sends back a response.
- **Actions:** A long-running, goal-oriented communication mechanism, often used for tasks like navigating to a goal or performing a complex manipulation.
- **Messages:** Data structures used for communication between nodes.
- **Packages:** The primary unit for organizing ROS 2 software, containing nodes, libraries, configuration files, and other resources.
### Advantages of ROS 2
ROS 2 offers several significant advantages over its predecessor, ROS 1, making it more suitable for modern robotics applications, especially those involving agentic AI:
- **Quality of Service (QoS) Policies:** ROS 2 provides fine-grained control over communication reliability, latency, and durability, crucial for real-time and safety-critical systems.
- **Distributed Architecture:** Built on DDS (Data Distribution Service), ROS 2 inherently supports distributed systems, allowing for better scalability and deployment across multiple machines or even different networks.
- **Security:** ROS 2 includes built-in security features (authentication, encryption, access control) to protect against unauthorized access and tampering, vital for robust robotic deployments.
- **Real-time Support:** Designed with real-time capabilities in mind, ROS 2 can meet stricter timing requirements for critical operations.
- **Multi-platform Support:** Broader support for various operating systems, including Windows, macOS, and embedded systems, in addition to Linux.
- **Improved Tooling:** Enhanced development tools and a more consistent API across different client libraries.

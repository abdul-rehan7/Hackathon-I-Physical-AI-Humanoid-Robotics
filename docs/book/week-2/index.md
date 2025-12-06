---
title: Week 2 - ROS 2 Deep Dive
---

# Week 2: ROS 2 Deep Dive

## Creating ROS 2 Packages and Nodes
### ROS 2 Package Structure
A ROS 2 package is the fundamental unit for organizing ROS 2 code. It encapsulates all the necessary files for a specific piece of functionality. A typical ROS 2 package structure includes:
- **`package.xml`:** Defines metadata about the package, such as its name, version, description, maintainers, licenses, and dependencies.
- **`CMakeLists.txt`:** The build configuration file for C++ packages, used by `colcon` (the ROS 2 build tool) to compile source code, link libraries, and install executables.
- **`setup.py`:** The build configuration file for Python packages, used by `colcon` to install Python modules and scripts.
- **`src/`:** Contains the source code for nodes, libraries, and other executables (e.g., C++ files, Python scripts).
- **`include/`:** (For C++ packages) Contains header files.
- **`launch/`:** Contains launch files (typically `.py` or `.xml`) for starting multiple nodes and configuring their parameters.
- **`config/`:** Contains configuration files (e.g., `.yaml`) for node parameters.
- **`msg/`, `srv/`, `action/`:** (Optional) Directories for custom message, service, and action definitions.

Understanding this structure is crucial for creating, building, and managing ROS 2 projects effectively.
### Creating a Simple ROS 2 Node
A ROS 2 node is an executable process that performs computation. Nodes communicate with each other using topics, services, and actions. Here's how to create a simple Python node that publishes a "Hello World" message to a topic:

1.  **Create a new package:**
    ```bash
    ros2 pkg create --build-type ament_python my_ros2_pkg
    ```

2.  **Create a Python script (`my_publisher.py`) in `my_ros2_pkg/my_ros2_pkg/`:**
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MyPublisher(Node):
        def __init__(self):
            super().__init__('my_publisher')
            self.publisher_ = self.create_publisher(String, 'my_topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        my_publisher = MyPublisher()
        rclpy.spin(my_publisher)
        my_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Add an entry point in `my_ros2_pkg/setup.py`:**
    ```python
    entry_points={
        'console_scripts': [
            'my_publisher = my_ros2_pkg.my_publisher:main',
        ],
    },
    ```

4.  **Build the package:**
    ```bash
    colcon build --packages-select my_ros2_pkg
    ```

5.  **Source the setup files and run the node:**
    ```bash
    source install/setup.bash
    ros2 run my_ros2_pkg my_publisher
    ```
This example demonstrates the basic steps to create a publisher node that sends messages over a topic. Similar principles apply to creating subscriber nodes and other communication patterns.

## Services, Actions, and Launch Files
### ROS 2 Services
ROS 2 Services provide a synchronous request/reply communication mechanism between nodes. Unlike topics, which are asynchronous and one-to-many, services are one-to-one and block the client until a response is received. They are ideal for operations that require an immediate result, such as querying a sensor for a single reading or triggering a specific action.

**Defining a Service:**
Service definitions are stored in `.srv` files within a package's `srv` directory. For example, a simple `AddTwoInts.srv` might look like this:
```
int64 a
int64 b
---
int64 sum
```
This defines a service that takes two `int64` integers (`a` and `b`) as a request and returns their `sum` as a response.

**Implementing a Service Server (Python):**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your service type

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Implementing a Service Client (Python):**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your service type
import sys

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Services are a fundamental building block for creating interactive and responsive robotic applications in ROS 2.
### ROS 2 Actions and Launch Files
**ROS 2 Actions:**
Actions in ROS 2 are a higher-level communication type used for long-running, goal-oriented tasks. They provide feedback during execution and allow for preemption (canceling a goal). An action consists of a goal, feedback, and a result.

**Defining an Action:**
Action definitions are stored in `.action` files within a package's `action` directory. For example, a simple `Fibonacci.action` might look like this:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
- `order`: The goal (e.g., compute Fibonacci sequence up to this order).
- `sequence`: The result (the completed Fibonacci sequence).
- `partial_sequence`: Feedback during execution (the sequence computed so far).

**ROS 2 Launch Files:**
Launch files are used to start and configure multiple ROS 2 nodes simultaneously. They provide a convenient way to define the entire system's startup behavior, including node parameters, remappings, and conditional execution. Launch files are typically written in Python or XML.

**Example Python Launch File:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ros2_pkg',
            executable='my_publisher',
            name='my_publisher_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'timer_period': 1.0}
            ]
        ),
        Node(
            package='my_ros2_pkg',
            executable='my_subscriber',
            name='my_subscriber_node',
            output='screen',
            emulate_tty=True
        )
    ])
```
This launch file starts two nodes: `my_publisher` and `my_subscriber`, both from `my_ros2_pkg`. It also sets a parameter for `my_publisher`. Launch files are essential for managing complex ROS 2 applications.

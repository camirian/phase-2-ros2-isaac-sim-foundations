# Phase 2: ROS 2 & Isaac Sim Foundations

This repository documents the hands-on development portion of Phase 2, focusing on mastering core robotics competencies. It contains a foundational ROS 2 package and scripts for controlling the NVIDIA Isaac Sim environment.

For definitions of key terms used in this project, please see my central **[AI & Robotics Glossary](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md)**.

---

## ✅ Skills Demonstrated

-   **ROS 2 Development:** Creating a ROS 2 [Package](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#package) from scratch using [Colcon](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#colcon) and the [Ament](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#ament) build system.
-   **Multi-Language Proficiency:** Implementing ROS 2 [Nodes](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#node) in both **Python** ([rclpy](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#rclcpp--rclpy)) and **C++** ([rclcpp](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#rclcpp--rclpy)).
-   **ROS 2 Communications:** Understanding the publisher/subscriber pattern using ROS 2 [Topics](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#topic).
-   **Isaac Sim Scripting:** Programmatically controlling the [Isaac Sim](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#isaac-sim) simulator with standalone Python scripts to create and manipulate objects.
-   **Version Control:** Maintaining a clean, professional [Git](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#git) repository for a public portfolio.

---

## 🚀 Projects

### 1. `core_robotics_package` (ROS 2)

A ROS 2 [Workspace](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#workspace) (`ros2-ws`) that holds the `core_robotics_package`. This package demonstrates a fundamental publisher/subscriber setup.

-   **Publisher (`simple_publisher.py`):** A Python node that publishes a string with a counter to the `/chatter` topic.
-   **Subscriber (`simple_subscriber.cpp`):** A C++ node that subscribes to `/chatter` and prints the messages.

#### How to Run (ROS 2)
1.  Navigate to `ros2-ws` and run `colcon build`.
2.  In one terminal, `source install/setup.bash` and run `ros2 run core_robotics_package simple_publisher.py`.
3.  In a second terminal, `source install/setup.bash` and run `ros2 run core_robotics_package simple_subscriber`.

### 2. Isaac Sim Scripts

These standalone Python scripts demonstrate programmatic control of the Isaac Sim environment.

-   **`simple_scene.py`:** Launches the simulator and adds a dynamic **red** cube that falls onto a ground plane.
-   **`add_prims.py`:** A similar script that launches the simulator and adds a dynamic **blue** cube, demonstrating the creation of different objects.

#### How to Run (Isaac Sim)
1.  Navigate to your Isaac Sim installation directory (e.g., `~/.local/share/ov/pkg/isaac-sim-2023.1.1/`).
2.  Execute a script using the bundled Python interpreter: `./python.sh /path/to/phase-2-ros2-isaac-sim-foundations/scripts/SCRIPT_NAME.py`.

---

## 📜 License

This project is licensed under the Apache 2.0 License. See the [`LICENSE`](./LICENSE) file for details.
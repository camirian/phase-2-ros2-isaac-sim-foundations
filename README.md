# Phase 2: ROS 2 & Isaac Sim Foundations

This repository documents the hands-on development portion of Phase 2, focusing on mastering core robotics competencies. It contains a foundational ROS 2 package and will later include scripts for controlling the NVIDIA Isaac Sim environment.

For definitions of key terms used in this project, please see my central **[AI & Robotics Glossary](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md)**.

---

## ✅ Skills Demonstrated

-   **ROS 2 Development:** Creating and building a ROS 2 [Package](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#package) from scratch using [Colcon](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#colcon) and the [Ament](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#ament) build system.
-   **Multi-Language Proficiency:** Implementing ROS 2 [Nodes](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#node) in both **Python** ([rclpy](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#rclcpp--rclpy)) and **C++** ([rclcpp](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#rclcpp--rclpy)) within a single package.
-   **ROS 2 Communications:** Demonstrating a clear understanding of the publisher/subscriber pattern using ROS 2 [Topics](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#topic).
-   **Version Control:** Maintaining a clean, professional [Git](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#git) repository with appropriate `.gitignore` and documentation practices for a public portfolio on [GitHub](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#github).

---

## 🚀 Project: `core_robotics_package`

This repository contains a ROS 2 [Workspace](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#workspace) (`ros2-ws`) that holds the `core_robotics_package`. This package demonstrates a fundamental publisher/subscriber setup.

-   **Publisher (`simple_publisher.py`):** A Python node that publishes a "Hello World" string with a counter to the `/chatter` topic every 0.5 seconds.
-   **Subscriber (`simple_subscriber.cpp`):** A C++ node that subscribes to the `/chatter` topic and prints the received messages to the console.

### How to Run

1.  Clone this repository.
2.  Navigate to the workspace directory: `cd ros2-ws`.
3.  Build the package: `colcon build`.
4.  Source the overlay in a new terminal: `source install/setup.bash`.
5.  Run the publisher: `ros2 run core_robotics_package simple_publisher.py`.
6.  Source the overlay in a second terminal: `source install/setup.bash`.
7.  Run the subscriber: `ros2 run core_robotics_package simple_subscriber`.

---

## 📜 License

This project is licensed under the Apache 2.0 License. See the [`LICENSE`](./LICENSE) file for details.
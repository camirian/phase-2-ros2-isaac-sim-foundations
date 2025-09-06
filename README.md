# Phase 2: Core Competencies in ROS 2 & Isaac Sim [COMPLETED]

This repository documents the hands-on development portion of Phase 2, focusing on mastering core robotics competencies. It contains foundational projects demonstrating communication with ROS 2 and control within the NVIDIA Isaac Sim environment.

For definitions of key terms used in this project, please see my central **[AI & Robotics Glossary](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md)**.

---

## ✅ Skills Demonstrated

-   **ROS 2 Development:** Creating multi-language **(Python & C++)** ROS 2 packages from scratch and building them with [Colcon](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#colcon).
-   **Robotics Communication:** Implementing the fundamental publisher/subscriber pattern for inter-node communication.
-   **Simulation & Control:** Programmatically controlling the [Isaac Sim](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#isaac-sim) simulator to spawn and command complex, articulated robots.
-   **ROS 2 Integration:** Establishing a robust communication bridge between Isaac Sim and ROS 2 for a full "sim-to-real" control and feedback loop.
-   **Visual Scripting:** Using NVIDIA's OmniGraph (Action Graph) to create a real-time data pipeline within the simulator.
-   **Systematic Debugging:** Diagnosing and solving build system, configuration, and API versioning issues.
-   **Version Control:** Maintaining a clean, professional [Git](https://github.com/caaren/phase-0-robotics-glossary/blob/main/GLOSSARY.md#git) repository for a public portfolio.

---

## 🚀 Completed Projects

This repository contains four distinct foundational projects, completed in logical order.

### Project 2.1: Standalone ROS 2 Publisher/Subscriber (Python)

A foundational ROS 2 package (`ros2-ws/src/py_pubsub`) that implements the "talker/listener" pattern in Python.

### Project 2.2: Standalone Isaac Sim Scripting

An advanced Python script (`scripts/franka_wave.py`) that demonstrates direct control over a simulated robot arm entirely within Isaac Sim.
-   **[▶️ Watch the robot in action on YouTube](https://youtu.be/MKuvEEEHLwQ)**

### Project 2.3: ROS 2 & Isaac Sim Bridge (OmniGraph)

This project demonstrates the fundamental workflow for connecting Isaac Sim to the ROS 2 ecosystem using **OmniGraph**.
-   **[▶️ Watch the live data stream on YouTube](https://youtu.be/YOUR_VIDEO_ID_HERE)**
![Final OmniGraph](./media/omnigraph.png)

### Project 2.4: Standalone ROS 2 Publisher/Subscriber (C++)

A high-performance ROS 2 package (`ros2-ws/src/cpp_pubsub`) that mirrors the Python version's functionality in C++. This demonstrates proficiency in both of the primary languages used in professional robotics.

---

## 🛠️ How to Build and Run

Detailed instructions for each project are contained within their respective package `README.md` files.

1.  Navigate to the ROS 2 workspace: `cd ~/dev/personal/ai-robotics-portfolio/phase-2-ros2-isaac-sim-foundations/ros2-ws`
2.  Build all packages: `colcon build`
3.  In separate, sourced terminals, run the nodes (e.g., `ros2 run py_pubsub talker` or `ros2 run cpp_pubsub talker`).

---

## 📜 License

This project is licensed under the Apache 2.0 License. See the [`LICENSE`](./LICENSE) file for details.
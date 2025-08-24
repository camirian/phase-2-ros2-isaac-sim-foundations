# Phase 2: ROS 2 & Isaac Sim Foundations

This repository documents the hands-on development portion of Phase 2, focusing on mastering core robotics competencies. It contains a foundational ROS 2 package and scripts for controlling the NVIDIA Isaac Sim environment.

For definitions of key terms used in this project, please see my central **[AI & Robotics Glossary](https://github.com/camirian/phase-0-robotics-glossary/blob/main/GLOSSARY.md)**.

---

## ✅ Skills Demonstrated

-   **ROS 2 Development:** Creating a multi-language (C++/Python) ROS 2 [Package](https://github.com/camirian/phase-0-robotics-glossary/blob/main/GLOSSARY.md#package) from scratch.
-   **Isaac Sim Scripting:** Programmatically controlling the [Isaac Sim](https://github.com/camirian/phase-0-robotics-glossary/blob/main/GLOSSARY.md#isaac-sim) simulator to spawn and manage objects.
-   **Articulated Robot Control:** Loading and commanding a complex robot manipulator using high-level APIs (`Franka` class) and explicit joint commands (`ArticulationAction`).
-   **Systematic Debugging:** Diagnosing and solving API versioning issues by analyzing log files and iterating on solutions.
-   **Version Control:** Maintaining a clean, professional [Git](https://github.com/camirian/phase-0-robotics-glossary/blob/main/GLOSSARY.md#git) repository for a public portfolio.

---

## 🚀 Projects

### 1. `core_robotics_package` (ROS 2)

A ROS 2 [Workspace](https://github.com/camirian/phase-0-robotics-glossary/blob/main/GLOSSARY.md#workspace) (`ros2-ws`) that holds a package demonstrating a fundamental publisher/subscriber setup.

#### How to Run (ROS 2)
1.  Navigate to `ros2-ws` and run `colcon build`.
2.  Run the publisher and subscriber nodes in separate, sourced terminals.

### 2. Isaac Sim Scripts

A collection of standalone Python scripts located in the `scripts/` directory.

-   **`simple_scene.py` / `add_prims.py`:** Foundational scripts that launch the simulator and add simple dynamic cubes.
-   **`franka_wave.py`:** An advanced script that loads a Franka Emika Panda robot and makes it perform a waving motion by sending continuous joint position commands.
    -   **[Watch a video of the robot in action on YouTube](https://youtu.be/MKuvEEEHLwQ)**

#### How to Run (Isaac Sim)
1.  Navigate to your Isaac Sim installation directory.
2.  Execute a script using the bundled Python interpreter: `./python.sh /path/to/phase-2-ros2-isaac-sim-foundations/scripts/SCRIPT_NAME.py`.

---

## 📜 License

This project is licensed under the Apache 2.0 License. See the [`LICENSE`](./
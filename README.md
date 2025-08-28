# Autonomous Drone Exploration System for GPS-Denied Environments
This repository contains the complete ROS-based software stack for an autonomous aerial robot designed for exploration and mapping in unknown, dynamic, and GPS-denied indoor environments. This project was developed as a Final Year Project, integrating state-of-the-art algorithms for perception, planning, and control into a cohesive, hierarchical system.

The system enables a drone equipped with an OAK-D stereo camera and a Raspberry Pi 5 to autonomously build a 3D map of its surroundings, make intelligent decisions about where to explore next, and generate smooth, safe, and dynamically feasible trajectories.

---
## 🚀 Key Features

-   🗺️ **Real-Time 3D SLAM:** Utilizes **RTAB-Map** for robust Visual-Inertial Odometry, enabling accurate localization and mapping without GPS.
-   🧠 **Dynamic World Modeling:** Builds a 3D voxel map using `map_manager` and incorporates an `onboard_detector` to track moving objects, preventing "ghost" obstacles from corrupting the map.
-   🧭 **Autonomous Exploration Planner:** Implements the **Dynamic Exploration Planner (DEP)**, which automatically identifies and navigates to frontiers (the edge of the known map) to efficiently explore unknown spaces.
-   ✈️ **Hierarchical Motion Planning:** Employs a two-layer planning strategy. The `DEP` finds a high-level geometric path, which is then passed to a **Polynomial Trajectory Optimizer** (`polyTraj`) to generate a smooth, minimum-snap trajectory that respects the drone's physical limits.
-   🖥️ **Web-Based Ground Control Station:** A custom-built, real-time web dashboard for mission supervision, telemetry monitoring, and high-level command and control.

---

## 🛠️ System Architecture

The software is designed as a modular, hierarchical pipeline within the ROS1 Noetic framework. This architecture decouples the core challenges of autonomy, allowing for specialized components to handle each task.
1.  **State Estimation Layer (Localization):**
    -   **Component:** `rtabmap_odom` & `rtabmap`
    -   **Function:** Fuses OAK-D stereo/IMU data for 6-DOF pose estimation.
    -   **Output:** `/odom` topic and a stable `map` -> `odom` TF transform.

2.  **World Modeling Layer (Mapping):**
    -   **Component:** `map_manager` & `onboard_detector`
    -   **Function:** Builds a 3D voxel map and tracks dynamic obstacles.
    -   **Output:** An inflated, dynamic-aware 3D map for collision checking.

3.  **Hierarchical Planning Layer:**
    -   **Component:** `autonomous_flight` node orchestrating `global_planner` (DEP) and `trajectory_planner` (`polyTraj`).
    -   **Function:** First, decides *where* to explore (DEP). Second, determines the smoothest, safest way to get there (`polyTraj`).
    -   **Output:** A time-parameterized trajectory of position, velocity, and acceleration setpoints.

4.  **Control & Supervision Layer:**
    -   **Component:** `tracking_controller` & Web GUI
    -   **Function:** The controller follows the trajectory by sending commands to the flight controller via MAVROS. The GUI allows an operator to monitor the entire process.

---

## 💻 Tech Stack

-   **Hardware:**
    -   Companion Computer: Raspberry Pi 5
    -   Perception Sensor: Luxonis OAK-D Pro W
    -   Flight Controller: Pixhawk 2.4.8 with ArduPilot
-   **Core Software:**
    -   OS: Ubuntu 20.04
    -   Middleware: ROS1 Noetic
    -   Primary Language: C++
-   **Key ROS Packages:**
    -   **Perception:** `rtabmap_ros`, `depthai_ros_driver`, `map_manager`, `onboard_detector`
    -   **Planning:** `global_planner`, `trajectory_planner`
    -   **Control:** `tracking_controller`, `mavros`
-   **GUI:**
    -   Backend: `rosbridge_server`
    -   Frontend: React.js, `roslibjs`, `Chart.js`

---

## ⚙️ Installation & Setup

### Prerequisites
- ROS1 Noetic installed on Ubuntu 20.04.
- A configured catkin workspace (e.g., `~/dai_ws`).
- All package-specific dependencies (can be installed by running `rosdep install --from-paths src --ignore-src -r -y`) [Follow depthai documentation for more details].

- ### Build Instructions
1.  **Clone the Repository:**
    ```bash
    cd ~/dai_ws/src
    git clone https://github.com/your-username/your-repo-name.git
    ```
2.  **Build the Workspace:**
    This project uses `catkin_make_isolated` due to its complex package dependencies.
    ```bash
    cd ~/dai_ws
    catkin_make_isolated
    ```
3.  **Source the Environment:**
    ```bash
    source ~/dai_ws/devel_isolated/setup.bash
    ```

### GUI Setup [Use it on a differnt machine not the pi]
1.  **Navigate to the GUI directory:**
    ```bash
    cd ~/dai_ws/src/your-repo-name/gui_directory
    ```
2.  **Install dependencies:**
    ```bash
    npm install
    ```

---

## ▶️ Usage - Running the System

The entire system is orchestrated by a master launch file.

1.  **Connect Hardware:** Ensure the Pixhawk is connected to the Raspberry Pi via the `/dev/ttyAMA0` serial port and the OAK-D is connected via USB3.
2.  **Launch MAVROS:** In a new terminal, start the MAVROS bridge to the flight controller.
    ```bash
    roslaunch my_drone mavros.launch
    ```
3.  **Launch the Main System:** In another terminal, run the primary launch file.
    ```bash
    roslaunch my_drone my_drone_main.launch
    ```
    This will start the camera, perception stack, planners, and RViz.
4.  **Launch the GUI:** On your ground machine, start the web server for the GUI.
    ```bash
    cd ~/dai_ws/src/your-repo-name/gui_directory
    npm start
    ```
    Now, open a web browser and navigate to `http://localhost:3000`.
5.  **Start the Mission:**
    -   Use a Ground Control Station (like QGroundControl) to verify all pre-arm checks have passed.
    -   Switch the drone to **GUIDED** mode.
    -   Arm the drone.
    -   From the web GUI, press the **[TAKEOFF]** button.
    -   Once hovering, press the **[EXPLORE]** button to begin the autonomous mission.

---


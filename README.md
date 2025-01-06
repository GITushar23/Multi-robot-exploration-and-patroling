# Collaborative Robotic Intelligence for Autonomous Environmental Understanding

This project introduces an intelligent, multi-agent system designed for comprehensive environmental mapping, dynamic surveillance, and interactive object handling within a simulated environment. Leveraging a team of agile TurtleBot3 robots operating within a shared virtual space, the system fosters a synergistic approach to environmental awareness. Each robot independently constructs a localized map, and these individual perspectives are then intelligently synthesized into a unified, global representation using a sophisticated map fusion mechanism.

The initial phase focuses on autonomous environmental mapping. Each robot is equipped with an intelligent exploration module, enabling it to autonomously navigate and delineate the boundaries of the operational space. Upon achieving complete coverage, the generated map serves as the foundation for subsequent operations. Powerful analytical tools are then applied to this map to strategically identify key locations suitable for establishing efficient patrol routes.

During the surveillance phase, the robotic team autonomously navigates these pre-defined patrol paths, maintaining vigilant oversight of the environment. Central to this phase is an integrated visual intelligence system. Each robot employs advanced object detection algorithms to identify and categorize objects encountered within its field of view. When an object of interest is detected, its location is precisely marked on the unified map, providing a comprehensive and real-time situational awareness display.

To facilitate dynamic interaction with the environment, an intelligent task assignment system is implemented. Upon the issuance of a specific task, the system dynamically assesses the proximity of each robot to the target object. The robot optimally positioned for the task is then autonomously dispatched to interact with the designated entity.

This sophisticated system is architected using a suite of cutting-edge robotic and AI technologies, including the ROS 2 (Humble) framework for distributed computation, the Gazebo simulator for realistic environment modeling, and the versatile TurtleBot3 platform for robot embodiment. The system further incorporates advanced machine learning techniques for autonomous exploration, robust object recognition, and efficient task allocation. Detailed, step-by-step instructions are provided below to facilitate the setup of the necessary development environment, encompassing ROS 2, Gazebo, Rviz2, TurtleBot3 drivers, Cartographer for SLAM, Nav2 for navigation, and CycleLearn for potential advanced learning tasks. Once the environment is configured, the entire system can be seamlessly launched using the `run.sh` script, initiating the full spectrum of multi-robot exploration, surveillance, object detection, and intelligent task handling.

---

## System Deployment: Installation Guide

The following instructions outline the procedure for installing the necessary software components and dependencies.

### 1. Install ROS 2 (Humble Hawksbill)
Adhere to the official ROS 2 Humble installation instructions. Ensure you install the 'desktop' variant and activate the environment:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Deploy Gazebo Simulator
Gazebo Classic provides the simulation environment. Install it via:

```bash
sudo apt update
sudo apt install gazebo
curl -sSL http://get.gazebosim.org | sh
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 3. Integrate Navigation Stack (Nav2)
Nav2 provides the autonomous navigation capabilities. Install the required packages:

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### 4. Incorporate TurtleBot3 Platform Support
Install the necessary packages for the TurtleBot3 robots:

```bash
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
```
Set the ROS domain ID for TurtleBot3:
```bash
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
```

### 5. Resolve Project Dependencies with rosdep
Initialize and utilize `rosdep` to manage project-specific dependencies:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 6. Enable Simultaneous Localization and Mapping (SLAM)
Due to the map merging approach, this project currently utilizes the well-established ROS 1 SLAM package, `slam_gmapping`. An adaptation for ROS 2 is required. This implementation has been validated with a specialized fork incorporating namespace support for multi-robot scenarios. Clone and build this package within your workspace:

```bash
cd <your/ros2_ws/src>
git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch
cd ..
colcon build --symlink-install --packages-up-to slam_gmapping
```

**Alternative SLAM Note:**  Experimental compatibility with `slam_toolbox` is under development in the [`feature/slam_toolbox_compat`](https://github.com/robo-friends/m-explore-ros2/tree/feature/slam_toolbox_compat) branch, offering a potential alternative.

### 7. Integrate and Compile the Project
Clone the project repository into your ROS 2 workspace and build the packages:

```bash
cd ~/ros2_ws/src
git clone <your_repository_url>
cd ~/ros2_ws
colcon build
source install/setup.bash
```
*(Note:  Minor warnings related to `.hpp` files can be safely ignored.)*

### 8. Prepare the Simulation Environment
For the `hospital_world.world` environment, execute the setup script and configure environment variables:

```bash
cd ~/ros2_ws/src/world_setup
chmod +x setup.sh
./setup.sh
export GAZEBO_MODEL_PATH=$(pwd)/models:$(pwd)/fuel_models:$GAZEBO_MODEL_PATH
```
*(Note: Minor warnings related to `.hpp` files can be safely ignored.)*

## Executing the System

With the environment properly configured, the system can be initiated using the provided script. Navigate to the script's location and execute:

```bash
./run.sh
```
This will launch the robot simulations, begin the exploration process, and activate the surveillance and object detection functionalities.

## Modular Execution of System Components

For fine-grained control, individual system components can be launched separately:

### 1. Initialize Multi-Robot Simulation:
Configure initial robot poses in `map_merge/launch/tb3_simulation/config/robot_poses.yaml` or disable pre-defined poses with `know_init_poses:=false`.

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True number_of_robots:=<number_of_robots>
```

### 2. Activate Map Fusion:
Launch the map merging node to combine individual robot maps.

```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

### 3. Visualize the Unified Map:
View the merged map in RViz2.

```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

### 4. Initiate Autonomous Exploration:
Start the exploration process for the specified number of robots.

```bash
ros2 launch explore_lite explore_launch.py num_robots:=<number_of_robots>
```

### 5. Persist the Generated Map:
Save the explored map to a file.

```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

### 6. Capture Camera Data (Optional):
Record camera feeds if needed.

```bash
ros2 launch scripts_pakages camera_launch.py
```

### 7. Deploy Patrol Behaviors:
Launch the patrolling nodes, specifying the map to use.

```bash
ros2 launch patroling patrol_launch.py map_file:=<map_name> map.yamal_file:=<map_name> num_robots:=<number_of_robots>
```

### 8. Start the ChromaDB Embedding Server:
Launch the ChromaDB server for managing object embeddings.

```bash
python server.py .chromadb/ 'clip1' --port 8000
```
```bash
Access at http://<your_server_ip>:8000
```

### 9. Launch the Vision Language Model (VLM):
Start the VLM server for object understanding.

```bash
vllm serve allenai/Molmo-7B-D-0924 --task generate \
  --trust-remote-code --max-model-len 4096 --limit-mm-per-prompt image=1 \
  --dtype bfloat16 --gpu-memory-utilization 0.5 --port 8081 \
```
```bash
Access at http://<your_server_ip>:8081
```

### 10. Initiate the Streamlit Interface:
Run the Streamlit application for user interaction and visualization.

```bash
streamlit run app.py
```
The Streamlit app publishes detected object positions, enabling task allocation.

### 11. Direct a Robot to a Specific Object:
Send a specific robot to interact with a designated object.

```bash
ros2 service call /send_robot_to_object patrolling_interfaces/srv/SendRobotToObject "{robot_name: 'robot1', object_name: 'object1'}"
```

## System Workflow Overview

1. **Robot Instantiation:** A team of TurtleBot3 robots is initialized within a customized Gazebo simulation environment.
2. **Collaborative Mapping:** Each robot autonomously generates a local map of its surroundings.
3. **Map Synthesis:** Individual maps are intelligently merged to create a comprehensive environmental representation.
4. **Autonomous Exploration:** Robots autonomously navigate and map the entirety of the operational area.
5. **Map Archival:** The completed map is saved for subsequent utilization.
6. **Strategic Patrol Planning:** Intelligent algorithms identify optimal locations for establishing patrol routes.
7. **Autonomous Surveillance:** Robots navigate the defined patrol paths, maintaining environmental awareness.
8. **Visual Object Recognition:** Integrated machine learning algorithms enable real-time object detection and identification.
9. **Object Tagging:** Detected objects are marked with persistent indicators on the unified map.
10. **Intelligent Task Assignment:**  The robot best suited for a given task (based on proximity) is autonomously dispatched.

## Contributing

Contributions to this project are highly encouraged!  Please fork the repository and submit pull requests, ensuring adherence to coding standards and including appropriate tests for any new features or modifications.

# GC Tech Low Prep

This project deploys multiple TurtleBot4 robots in a custom Ignition Gazebo world, with each robot operating within its own map. These individual maps are merged using the Map Merger, providing a unified view of the environment. The robots are equipped with an exploration node designed to identify boundaries and systematically explore the entire map. Upon completion of the exploration, the map is saved for further use.

The system is built using a variety of technologies, including ROS 2 (Humble), Ignition Gazebo for simulation, TurtleBot4 for robot control, and exploration algorithms. Detailed installation instructions are provided to set up the necessary environment, including ROS, Ignition Gazebo, Rviz2, TurtleBot4, Cartographer, and Nav2. Once the environment is set up, the project can be executed using the run.sh script, which initializes and launches all system nodes. This enables the full simulation of the multi-robot exploration system with map merging capabilities.

---

## Pre-setup: Installation Instructions

Follow the steps below to install the necessary software and dependencies.

### 1. Install ROS 2 Humble
Follow the official ROS 2 Humble installation guide. Ensure you install the desktop version and source the setup script:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Install Ignition Gazebo
Ignition Gazebo is required for simulation. To install it:

```bash
sudo apt update
sudo apt install ignition-fortress
sudo apt install ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge
```

### 3. Install Nav2
Nav2 is for navigation.

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### 4. Install TurtleBot4
TurtleBot4 is the robot platform used in this project. Install it with the following command:

```bash
sudo apt install ros-humble-turtlebot4-desktop
sudo apt install ros-humble-turtlebot4-simulator
sudo apt install ros-humble-turtlebot4-msgs
sudo apt install ros-humble-turtlebot4-navigation
```

Export the TurtleBot4 model:
```bash
echo 'export TURTLEBOT4_MODEL=standard #TURTLEBOT4' >> ~/.bashrc
```

### 5. Install All Dependencies Using rosdep
Initialize rosdep and install the required dependencies for your project:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y 
```

### 6. SLAM
Because of the logic that merges the maps, currently as a straightforward port to ROS2 from the ROS1 version, the SLAM needs to be done using the ROS1 defacto slam option which is [slam_gmapping](https://github.com/ros-perception/slam_gmapping), which hasn't been ported officially to ROS2 yet. There is an unofficial port but it lacks to pass a namespace to its launch file. For that, this repo was tested with one of the authors of this package's [fork](https://github.com/charlielito/slam_gmapping/tree/feature/namespace_launch). You'll need to git clone to your workspace and build it with colcon.

```bash
cd <your/ros2_ws/src>
git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch
cd ..
colcon build --symlink-install --packages-up-to slam_gmapping
```

**Note**: You could use [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) instead but you need to use this [experimental branch](https://github.com/robo-friends/m-explore-ros2/tree/feature/slam_toolbox_compat) which is still under development.

### 7. Install and build the package
Clone the package in ros2_ws and build it:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build
source install/setup.bash
```
Ignore the .hpp warning.

### 8. Add the map dependencies
For hospital_world.world:
```bash
cd ~/ros2_ws/src/world_setup
chmod +x setup.sh
./setup.sh
export IGN_GAZEBO_RESOURCE_PATH=$(pwd)/models:$(pwd)/fuel_models
```
Ignore the .hpp warning.

## Running the Project
Once all dependencies are installed and the environment is set up, you can start the nodes using the run.sh script. Enter into the directory where this file is present and run this command:

```bash
./run.sh
```
This script will spawn the robots and initiate the exploration process.

## Running individual nodes
If you want to run the individual nodes, follow these steps:

### 1. Running Multi-bot:
You can define initial position in map_merge/launch/tb4_simulation/config/robot_poses.yaml. There you can change the robot poses for the particular world or else just type know_init_poses:=false

```bash
export TURTLEBOT4_MODEL=standard
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot4_gazebo/models
ros2 launch multirobot_map_merge multi_tb4_simulation_launch.py slam_gmapping:=True number_of_robots:=<no of robots>
```

### 2. Map-merging
For merging maps from multiple robots:
```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

### 3. View map in rviz
Either you can directly open /map topic in rviz2 or run this command:
```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

### 4. Exploration
Launch the exploration node for a specific number of robots:
```bash
ros2 launch explore_lite explore_launch.py num_robots:=<no of robots>
```

### 5. Save map
Save the map generated during exploration with a specific name:
```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

## How it Works
1. **Robot Spawning**: Multiple TurtleBot4 robots are spawned in a custom Ignition Gazebo world.
2. **Map Merger**: The maps of the robots are merged into a single, unified map.
3. **Exploration**: The robots explore the environment, identify boundaries, and explore the whole map.
4. **Map Saving**: The map is saved after the exploration.

## Contributing
Feel free to fork the repository and submit pull requests. Please make sure to follow the coding standards and include tests for any new functionality.

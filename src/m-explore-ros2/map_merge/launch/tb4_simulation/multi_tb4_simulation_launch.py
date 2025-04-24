"""
Modified launch file to spawn multiple TurtleBot4 robots in Ignition Gazebo, with dynamic number of robots.
Handles both known and unknown initial poses, similar to the original code,
and keeps their positions the same in both cases.
Generates parameter files for each robot within the launch file if they do not exist.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Get directories for multirobot_map_merge package
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    launch_dir_map_merge = os.path.join(map_merge_dir, "launch", "tb4_simulation")
    launch_dir_tb4_model = os.path.join(map_merge_dir, "launch", "tb4_model")
    config_dir = os.path.join(launch_dir_map_merge, "config")

    # Define LaunchConfigurations
    world = LaunchConfiguration("world")
    simulator = LaunchConfiguration("simulator")
    map_yaml_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings")
    known_init_poses = LaunchConfiguration("known_init_poses")
    robot_sdf = LaunchConfiguration("robot_sdf")
    number_of_robots = LaunchConfiguration("number_of_robots")
    robot_poses_file = LaunchConfiguration("robot_poses_file")
    use_rviz_per_agent = LaunchConfiguration("use_rviz_per_agent")
    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(launch_dir_map_merge, "worlds", "walk_object.sdf"),
        description="Full path to world file to load",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="ign gazebo",
        description="The simulator to use (ign gazebo or ign gazebo -s)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "maps", "turtlebot4_world.yaml"),
        description="Full path to map file to load",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the stacks",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
        description="Full path to the RVIZ config file to use.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_log_settings_cmd = DeclareLaunchArgument(
        "log_settings",
        default_value="true",
        description="Whether to log settings",
    )

    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        "slam_toolbox",
        default_value="False",
        description="Whether to run SLAM toolbox",
    )

    declare_slam_gmapping_cmd = DeclareLaunchArgument(
        "slam_gmapping",
        default_value="False",
        description="Whether to run SLAM gmapping",
    )

    declare_known_init_poses_cmd = DeclareLaunchArgument(
        "known_init_poses",
        default_value="True",
        description="Known initial poses of the robots. If so, don't forget to declare them in the params.yaml file",
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(launch_dir_tb4_model, "standard.sdf"),
        description="Full path to robot sdf file to spawn the robot in Ignition Gazebo",
    )

    declare_number_of_robots_cmd = DeclareLaunchArgument(
        "number_of_robots",
        default_value="5",
        description="Number of robots to spawn",
    )

    declare_robot_poses_file_cmd = DeclareLaunchArgument(
        "robot_poses_file",
        default_value=os.path.join(launch_dir_map_merge, "config", "robot_poses.yaml"),
        description="Full path to the robot poses yaml file",
    )

    declare_use_rviz_per_agent_cmd = DeclareLaunchArgument(
        "use_rviz_per_agent",
        default_value="True",
        description="Whether to launch RVIZ for each agent",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_log_settings_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_slam_gmapping_cmd)
    ld.add_action(declare_known_init_poses_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_number_of_robots_cmd)
    ld.add_action(declare_robot_poses_file_cmd)
    ld.add_action(declare_use_rviz_per_agent_cmd)

    # Add an OpaqueFunction to generate the robot spawning and nav instances
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


def launch_setup(context, *args, **kwargs):
    # Get the LaunchConfiguration values
    robot_poses_file = LaunchConfiguration("robot_poses_file").perform(context)
    number_of_robots = int(LaunchConfiguration("number_of_robots").perform(context))
    use_rviz_per_agent = (
        LaunchConfiguration("use_rviz_per_agent").perform(context).lower()
    )
    use_rviz_global = LaunchConfiguration("use_rviz").perform(context).lower()
    known_init_poses = LaunchConfiguration("known_init_poses").perform(context).lower()
    log_settings = LaunchConfiguration("log_settings").perform(context).lower()
    slam_toolbox = LaunchConfiguration("slam_toolbox").perform(context)
    slam_gmapping = LaunchConfiguration("slam_gmapping").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    use_robot_state_pub = (
        LaunchConfiguration("use_robot_state_pub").perform(context).lower()
    )
    rviz_config_file = LaunchConfiguration("rviz_config").perform(context)
    map_yaml_file = LaunchConfiguration("map").perform(context)
    robot_sdf = LaunchConfiguration("robot_sdf").perform(context)
    simulator = LaunchConfiguration("simulator").perform(context)
    world = LaunchConfiguration("world").perform(context)

    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    launch_dir_map_merge = os.path.join(map_merge_dir, "launch", "tb4_simulation")
    config_dir = os.path.join(launch_dir_map_merge, "config")

    # Read robot poses from the robot_poses_file
    with open(robot_poses_file, "r") as f:
        robots_data = yaml.safe_load(f)

    robots_list = robots_data["robots"][:number_of_robots]

    # Create robots_known_poses and robots_unknown_poses, both same positions for this case
    robots_known_poses = robots_list
    robots_unknown_poses = robots_list

    # Prepare commands
    commands = []

    # Start Ignition Gazebo with bridge to ROS
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_ign_gazebo", "ign_gazebo.launch.py",
            "ign_args:=" + world,
        ],
        output="screen",
    )
    commands.append(start_gazebo_cmd)

    # Start the ROS-Ignition bridge
    start_bridge_cmd = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_ign_bridge", "parameter_bridge.launch.py",
            "config_file:=" + os.path.join(launch_dir_map_merge, "config", "tb4_bridge.yaml"),
        ],
        output="screen",
    )
    commands.append(start_bridge_cmd)

    # Ports for groot monitoring, starting from 1500
    groot_publisher_port = 1500
    groot_server_port = 1600
    groot_port_increment = 100  # for each robot, to avoid port conflicts

    # Base parameters file path
    base_params_file = os.path.join(config_dir, "nav2_multirobot_params_1.yaml")

    # Spawn robots and launch nav instances
    for idx, (robot_known, robot_unknown) in enumerate(
        zip(robots_known_poses, robots_unknown_poses)
    ):
        robot_name = robot_known["name"]
        x_pose = robot_known["x_pose"]
        y_pose = robot_known["y_pose"]
        z_pose = robot_known.get("z_pose", "0.01")  # default z_pose if not specified

        # Calculate unique groot ports for each robot
        groot_pub_port = groot_publisher_port + idx * groot_port_increment
        groot_serv_port = groot_server_port + idx * groot_port_increment

        # Generate parameter file for the robot if it does not exist
        robot_params_file = os.path.join(
            config_dir, f"nav2_multirobot_params_{robot_name}.yaml"
        )

        if not os.path.exists(robot_params_file):
            # Read base parameters
            with open(base_params_file, "r") as f:
                params = yaml.safe_load(f)

            # Update parameters specific to the robot
            # Update scan topics in local and global costmaps
            scan_topic = f"/{robot_name}/scan"
            params["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"][
                "scan"
            ]["topic"] = scan_topic
            params["local_costmap"]["local_costmap"]["ros__parameters"][
                "observation_sources"
            ] = "scan"
            params["global_costmap"]["global_costmap"]["ros__parameters"][
                "obstacle_layer"
            ]["scan"]["topic"] = scan_topic
            params["global_costmap"]["global_costmap"]["ros__parameters"][
                "observation_sources"
            ] = "scan"

            # Update groot ports in bt_navigator
            params["bt_navigator"]["ros__parameters"][
                "groot_zmq_publisher_port"
            ] = groot_pub_port
            params["bt_navigator"]["ros__parameters"][
                "groot_zmq_server_port"
            ] = groot_serv_port

            # Save the updated parameters to the robot's parameter file
            with open(robot_params_file, "w") as f:
                yaml.dump(params, f)

        # Spawn robot in Ignition Gazebo for known initial poses
        spawn_robot_known_cmd = Node(
            package="ros_ign_gazebo",
            executable="create",
            output="screen",
            arguments=[
                "-name", robot_known["name"],
                "-file", robot_sdf,
                "-x", str(robot_known["x_pose"]),
                "-y", str(robot_known["y_pose"]),
                "-z", str(robot_known.get("z_pose", "0.01")),
                "-R", "0.0",
                "-P", "0.0",
                "-Y", "0.0",
                "-namespace", robot_known["name"],
            ],
            condition=IfCondition(known_init_poses),
        )
        commands.append(spawn_robot_known_cmd)

        # Spawn robot in Ignition Gazebo for unknown initial poses
        spawn_robot_unknown_cmd = Node(
            package="ros_ign_gazebo",
            executable="create",
            output="screen",
            arguments=[
                "-name", robot_unknown["name"],
                "-file", robot_sdf,
                "-x", str(robot_unknown["x_pose"]),
                "-y", str(robot_unknown["y_pose"]),
                "-z", str(robot_unknown.get("z_pose", "0.01")),
                "-R", "0.0",
                "-P", "0.0",
                "-Y", "0.0",
                "-namespace", robot_unknown["name"],
            ],
            condition=UnlessCondition(known_init_poses),
        )
        commands.append(spawn_robot_unknown_cmd)

        # Launch navigation instance
        nav_instance_cmds = []

        # Launch rviz if specified
        if use_rviz_per_agent == "true" and use_rviz_global == "true":
            rviz_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "namespace": robot_name,
                    "use_namespace": "True",
                    "rviz_config": rviz_config_file,
                }.items(),
            )
            nav_instance_cmds.append(rviz_launch)

        # Prepare the nav_launch_arguments for the robot
        nav_launch_arguments = {
            "namespace": robot_name,
            "use_namespace": "True",
            "map": map_yaml_file,
            "use_sim_time": "True",
            "params_file": robot_params_file,
            "autostart": autostart,
            "use_rviz": "False",
            "use_simulator": "False",
            "headless": "False",
            "slam": "True",
            "slam_toolbox": slam_toolbox,
            "slam_gmapping": slam_gmapping,
            "use_robot_state_pub": use_robot_state_pub,
        }

        if known_init_poses == "true":
            nav_launch_arguments["initial_pose_x"] = str(x_pose)
            nav_launch_arguments["initial_pose_y"] = str(y_pose)
            nav_launch_arguments["initial_pose_z"] = str(z_pose)

        # Include the navigation launch file
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir_map_merge, "tb4_simulation_launch.py")
            ),
            launch_arguments=nav_launch_arguments.items(),
        )
        nav_instance_cmds.append(nav_launch)

        # Add logging if log_settings is true
        if log_settings == "true":
            nav_instance_cmds.append(
                LogInfo(
                    msg=["Launching ", robot_name],
                )
            )
            nav_instance_cmds.append(
                LogInfo(
                    msg=[robot_name, " map yaml: ", map_yaml_file],
                )
            )
            nav_instance_cmds.append(
                LogInfo(
                    msg=[robot_name, " params yaml: ", robot_params_file],
                )
            )
            nav_instance_cmds.append(
                LogInfo(
                    msg=[robot_name, " rviz config file: ", rviz_config_file],
                )
            )
            nav_instance_cmds.append(
                LogInfo(
                    msg=[
                        robot_name,
                        " using robot state pub: ",
                        use_robot_state_pub,
                    ],
                )
            )
            nav_instance_cmds.append(
                LogInfo(
                    msg=[robot_name, " autostart: ", autostart],
                )
            )

        # Group actions
        nav_instance = GroupAction(nav_instance_cmds)

        commands.append(nav_instance)

    return commands
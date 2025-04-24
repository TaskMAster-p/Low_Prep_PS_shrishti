#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.logging import get_logger
from launch import LaunchContext

def generate_launch_description():
    ld = LaunchDescription()

    # Path to the params.yaml file
    config = os.path.join(
        get_package_share_directory("explore_lite"), 
        "config", 
        "params.yaml"
    )

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation/Gazebo clock'
    )

    declare_num_robots_argument = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to launch explore nodes for'
    )

    declare_namespace_prefix_argument = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='robot',
        description='Namespace prefix for robots (e.g., "robot" for "robot1", "robot2", ...)'
    )

    # Add launch arguments to the LaunchDescription
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_num_robots_argument)
    ld.add_action(declare_namespace_prefix_argument)

    def launch_explore_nodes(context: LaunchContext, *args, **kwargs):
        try:
            num = int(context.launch_configurations['num_robots'])
            prefix = context.launch_configurations['namespace_prefix']
            # Convert string 'true'/'false' to boolean
            use_sim_time_str = context.launch_configurations['use_sim_time'].lower()
            use_sim_time_bool = use_sim_time_str == 'true'
        except (ValueError, KeyError) as e:
            get_logger().error(f"Invalid launch configuration: {e}")
            return []

        nodes = []
        for i in range(1, num + 1):
            namespace = f"{prefix}{i}"
            node_name = f"explore_node_{i}"

            # Define remappings
            remappings = [
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]

            # Create node with explicit boolean parameter
            node = Node(
                package='explore_lite',
                executable='explore',
                name=node_name,
                namespace=namespace,
                parameters=[
                    config,
                    {
                        'use_sim_time': use_sim_time_bool
                    }
                ],
                output='screen',
                remappings=remappings
            )

            nodes.append(node)
            get_logger().info(f"Launched {node_name} in namespace '{namespace}'")

        return nodes

    # Add the OpaqueFunction to handle dynamic node creation
    ld.add_action(OpaqueFunction(function=launch_explore_nodes))

    return ld
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    bringup_dir = get_package_share_directory("nav2_bringup")

    # These variables need to be defined using LaunchConfiguration
    robot_name = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    # launch_dir needs to be defined
    launch_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', 
            default_value='robot1', 
            description='Namespace to use for the robot'
        ),
        DeclareLaunchArgument(
            'use_namespace', 
            default_value='True', 
            description='Whether to use namespace'
        ),

        DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
        description="Full path to the RVIZ config file to use.",
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'launch', 'rviz_launch.py')
            ),
            launch_arguments={
                "namespace": robot_name,
                "use_namespace": "True",
                "rviz_config": rviz_config_file,
            }.items(),
        ),
        
        # Optionally log information for debugging purposes
        LogInfo(
            msg=["Launching RViz with namespace: ", LaunchConfiguration('namespace')]
        )
    ])
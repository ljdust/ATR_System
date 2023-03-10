import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('atr_visualisation')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    # Define arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "visualisation.rviz"),
        description="Full path to the RVIZ config file to use",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")

    # ATR state list subscriber. It publishes the ATR as MarkerArray
    atr_state_list_subscriber_yaml = os.path.join(
        get_package_share_directory('atr_bringup'),
        'params',
        'node_config_params.yaml'
    )
    
    # Rviz node
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )
    
    atr_state_list_subscriber = Node(
        package='atr_visualisation',
        name='atr_state_list_subscriber',
        executable='atr_state_list_subscriber',
        parameters=[atr_state_list_subscriber_yaml],
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Load nodes (actions)
    ld.add_action(rviz_cmd)
    ld.add_action(atr_state_list_subscriber)

    return ld


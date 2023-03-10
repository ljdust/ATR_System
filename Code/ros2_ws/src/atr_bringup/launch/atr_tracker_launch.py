#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from atr_bringup.config_parser import read_yaml
import argparse

def generate_launch_description():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Parse information from parent launch')
    parser.add_argument('-n', '--node_config', type=str, default='node_config_params.yaml',
                    help='Pass yaml file from parent')
    args, unknown = parser.parse_known_args()
    # Get path to package directory
    bringup_dir = get_package_share_directory('atr_bringup')
    # Get path to param file from argument
    param_file = os.path.join(bringup_dir, 'params', args.node_config)
    network_topology = read_yaml(param_file)['network_topology']['ros__parameters']
    # Load parameters for atr_tracker
    tracker_info = read_yaml(param_file)['atr_tracker']['ros__parameters']
    # Get the architecture
    system_architecture = network_topology['architecture']
    #Get debug parameter
    debug_var = tracker_info['debug']
    
    ld = LaunchDescription()
    
    # Create the launch description and populate
    ld.add_action(LaunchDescription([
                Node(
                    name = tracker_info['node_base_name'],
                    package=tracker_info['package'],
                    executable=tracker_info['executable'],
                    parameters=[{'architecture': system_architecture, 'debug': debug_var, 'max_pos_age': tracker_info['max_pos_age'],
                    'watchdog_period': tracker_info['watchdog_period'],
                    'list_publisher_period': tracker_info['list_publisher_period']}],
                    output='screen'
                    )]))
    
    # Measurement yaml file
    measurement_yaml = os.path.join(
        get_package_share_directory('atr_bringup'),
        'params',
        'node_config_params.yaml'
    )
    # Measurement node
    ld.add_action(LaunchDescription([
                Node(
                    name = "atr_state_list_listener",
                    package=tracker_info['package'],
                    executable="atr_state_list_listener",
                    parameters=[measurement_yaml],
                    output='screen'
                    )]))
    
    return ld
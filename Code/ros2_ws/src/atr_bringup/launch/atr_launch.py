#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from atr_bringup.config_parser import read_yaml
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
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
    # Load parameters for network_topology
    network_topology = read_yaml(param_file)['network_topology']['ros__parameters']
    # Load parameters for atr bot
    atr_info = read_yaml(param_file)['atr_bot']['ros__parameters']
    # create position publisher topic name
    pos_pubisher_name = atr_info['state_topic_name']
    # get the first id
    atr_id = network_topology['start_id']
    
    ld = LaunchDescription()
    timer_list = []
    
    counter = 1;
    auto_con = True;
    for group in range(network_topology['num_of_atr_groups']):
        # Create the list of spawn robots commands
        node_list = []
        
        # Create separate topic for each atr, allow for separate QoS configuration.   
        for node in range(network_topology['num_of_atr_in_groups']):
            
            if(network_topology['architecture'] ==1):
                pos_pubisher_name = atr_info['state_topic_name']+str(atr_id+node)
            if(counter <=atr_info['num_of_connected_atr']):
                auto_con = True;
            else:
                auto_con = False;
            node_list.append(
                    Node(
                        name = atr_info['node_base_name']+str(atr_id+node),
                        package=atr_info['package'],
                        executable=atr_info['executable'],
                        parameters=[{'state_topic_name': pos_pubisher_name, 'id':atr_id+node, 'auto_connect':auto_con,
                                'x': atr_info['base_position']['x'],
                                'y': atr_info['base_position']['y'],
                                'z': atr_info['base_position']['z'],
                                'state_publisher_period': atr_info['state_publisher_period']}],
                        output='screen'
                        ))
            counter = counter+1;
        
        atr_id = atr_id + network_topology['num_of_atr_in_groups']  
    
        # Create the launch description and populate
        ld_temp = LaunchDescription()
        for node in node_list:
            ld_temp.add_action(node)
        
        ld.add_action(TimerAction(
            period=network_topology['delay']*(group-1),
            actions=[
                ld_temp
                ]
        ))
        
    return ld

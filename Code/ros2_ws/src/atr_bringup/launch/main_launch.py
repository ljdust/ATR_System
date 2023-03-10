#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess

PARAM_FILENAME = "node_config_params.yaml"


# GLOBAL LAUNCH VARIABLES
visualisation = True
tracker = True
robots = True


def generate_launch_description():
    return LaunchDescription(
        [
            # VISUALISATION PACKAGE
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/visualisation_launch.py"]
                ),
                launch_arguments={
                            "node_config": PARAM_FILENAME,
                        }.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    # ATR TRACKER LAUNCH
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [ThisLaunchFileDir(), "/atr_tracker_launch.py"]
                        ),
                        launch_arguments={
                            "node_config": PARAM_FILENAME,
                        }.items(),
                    ),
                    # ATRS LAUNCH
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [ThisLaunchFileDir(), "/atr_launch.py"]
                        ),
                        launch_arguments={
                            "node_config": PARAM_FILENAME,
                        }.items(),
                    ),
                ],
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
                shell=True,
                output="screen",
            ),
        ]
    )

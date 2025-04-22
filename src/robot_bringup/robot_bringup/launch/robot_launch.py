#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    ros_setup = "/opt/ros/humble/setup.bash"
    ws_setup = "/home/skulikk/robot/install/setup.bash"

    encoder_node = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             f'source {ros_setup} && source {ws_setup} && ros2 run sensors_pkg encoder_node'],
        output='screen'
    )

    imu_node = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             f'source {ros_setup} && source {ws_setup} && ros2 run sensors_pkg imu_node'],
        output='screen'
    )

    motor_controller_node = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             f'source {ros_setup} && source {ws_setup} && ros2 run control_pkg motor_controller_node'],
        output='screen'
    )

    bluetooth_node = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             f'source {ros_setup} && source {ws_setup} && ros2 run bluetooth_pkg bluetooth_node'],
        output='screen'
    )

    return LaunchDescription([
        encoder_node,
        imu_node,
        motor_controller_node,
        bluetooth_node
    ])
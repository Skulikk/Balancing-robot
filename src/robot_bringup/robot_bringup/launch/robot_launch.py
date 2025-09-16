#!/usr/bin/env python3

# Bakalarska prace - Balancujici robot
# author: Tomas Skolek (xskole01)

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Coordinated startup of ROS2 nodes

    ros_setup = "/opt/ros/humble/setup.bash"
    ws_setup = "/home/skolek/robot/install/setup.bash"
    encoder_script = "/home/skolek/robot/run_encoder_exc.sh"

    imu_node = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             f'source {ros_setup} && source {ws_setup} && ros2 run sensors_pkg imu_node'],
        output='screen'
    )

    ultra_s_node = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             f'source {ros_setup} && source {ws_setup} && ros2 run sensors_pkg ultra_s_node'],
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

    # Encoder node runs on exclusive CPU core - startup via a script
    encoder_node = ExecuteProcess(
        cmd=['sudo', 'bash', encoder_script],
        output='screen'
    )

    return LaunchDescription([
        imu_node,
        motor_controller_node,
        ultra_s_node,
        bluetooth_node,
        encoder_node
    ])

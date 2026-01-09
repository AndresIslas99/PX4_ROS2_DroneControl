#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for drone control GUI"""

    # Launch description
    ld = LaunchDescription()

    # 1. Start MicroXRCE-DDS Agent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen'
    )
    ld.add_action(micro_xrce_agent)

    # 2. Start PX4 SITL with x500_depth model (with delay)
    px4_sitl = TimerAction(
        period=2.0,  # Wait 2 seconds for agent to start
        actions=[
            ExecuteProcess(
                cmd=['make', 'px4_sitl', 'gz_x500_depth'],
                cwd=os.path.expanduser('~/PX4-Autopilot'),
                name='px4_sitl',
                output='screen',
                shell=True
            )
        ]
    )
    ld.add_action(px4_sitl)

    # 3. Start Gazebo GUI (with delay)
    gz_gui = TimerAction(
        period=10.0,  # Wait for simulation to start
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                name='gazebo_gui',
                output='screen'
            )
        ]
    )
    ld.add_action(gz_gui)

    # 4. Start Drone Control GUI (with delay)
    drone_gui = TimerAction(
        period=15.0,  # Wait for everything else to start
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'drone_control_gui', 'drone_control_gui'],
                name='drone_control_gui',
                output='screen'
            )
        ]
    )
    ld.add_action(drone_gui)

    return ld

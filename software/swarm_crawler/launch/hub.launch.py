# this file will contain all the necessary code to operate the hub.

# this includes, but is not limited to:
# launching the hub_lift_controller.py which will use pyserial to lift and lower the arms of the hub
# figure out communication between the specific minibot it desires to communicate with, and subscribe accordingly.

import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
     package_name = 'swarm_crawler'

     # need to fill in a lot of stuff here
     




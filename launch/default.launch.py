from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import math


def generate_launch_description():
  ld = LaunchDescription()


  laser_filter = LaunchDescription([
        Node(
            package="thi_laser_filter",
            executable="laser_filter_node",
            name="laser_filter_node",
            namespace="",
            output="screen",
            parameters=[{
                "qos_scan_pub": "default", #or default
                "qos_scan_sub": "default",  #or default
                "min_range": 0.75,
            }],
            remappings=[
                #from, to
                ("scan", "scan"),
                ("scan_filtered", "scan_filtered"),
            ],
        )
    ])

  ld.add_action(laser_filter)


  return ld
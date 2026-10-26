"""robot_state_publisher + module manager.

  ros2 launch bhl_module_manager module_manager.launch.py
  ros2 launch bhl_module_manager module_manager.launch.py config:=/abs/path/B_handswap.yaml attach_mode:=dynamic \
      bhl_urdf:=/abs/path/berkeley_humanoid_lite.urdf mesh_prefix:=package://<your_bhl_description>/meshes
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bhl_module_manager import build, to_urdf
from bhl_module_manager.config import default_paths


def setup(context):
    _, _, cdir = default_paths()
    cfg = LaunchConfiguration("config").perform(context) or os.path.join(cdir, "A_full.yaml")
    mode = LaunchConfiguration("attach_mode").perform(context)
    bhl = LaunchConfiguration("bhl_urdf").perform(context) or None
    prefix = LaunchConfiguration("mesh_prefix").perform(context) or None
    asm, _ = build(cfg)
    urdf = to_urdf(asm, attach_mode=mode, bhl_urdf=bhl, mesh_prefix=prefix)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen"),
        Node(package="bhl_module_manager", executable="module_manager", name="module_manager", output="screen",
             parameters=[{"initial_config": cfg, "attach_mode": mode, "bhl_urdf": bhl or "", "mesh_prefix": prefix or "",
                          "rsp_node": "robot_state_publisher"}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("attach_mode", default_value="dynamic"),
        DeclareLaunchArgument("bhl_urdf", default_value=""),
        DeclareLaunchArgument("mesh_prefix", default_value=""),
        OpaqueFunction(function=setup),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("cuvslam_bridge")
    lc = context.launch_configurations

    config_file = lc.get("config_file") or os.path.join(pkg, "config", "cuvslam.yaml")

    overrides = {}
    if lc.get("odometry_mode"):
        overrides["odometry_mode"] = lc["odometry_mode"]
    if lc.get("use_slam"):
        overrides["use_slam"] = lc["use_slam"].lower() in ("1", "true", "yes", "on")
    if lc.get("publish_image"):
        overrides["publish_image"] = lc["publish_image"].lower() in ("1", "true", "yes", "on")
    if lc.get("run_seconds"):
        overrides["run_seconds"] = float(lc["run_seconds"])
    if lc.get("snapshot_path"):
        overrides["snapshot_path"] = lc["snapshot_path"]
    if lc.get("imu_decimate"):
        overrides["imu_decimate"] = int(lc["imu_decimate"])

    bridge = Node(
        package="cuvslam_bridge",
        executable="cuvslam_bridge_node",
        name="cuvslam_bridge",
        output="screen",
        parameters=[config_file, overrides],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg, "rviz", "cuvslam.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    return [bridge, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value="",
                              description="YAML with calibration / IMU / SLAM parameters"),
        DeclareLaunchArgument("odometry_mode", default_value="",
                              description="inertial | stereo | mono | rgbd | multisensor (overrides yaml)"),
        DeclareLaunchArgument("use_slam", default_value=""),
        DeclareLaunchArgument("imu_decimate", default_value=""),
        DeclareLaunchArgument("publish_image", default_value=""),
        DeclareLaunchArgument("run_seconds", default_value=""),
        DeclareLaunchArgument("snapshot_path", default_value=""),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])

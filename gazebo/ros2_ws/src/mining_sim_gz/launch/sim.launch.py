"""Gazebo simulation + the full mapping stack.

  ros2 launch mining_sim_gz sim.launch.py world:=cave                 # cave (full roof)
  ros2 launch mining_sim_gz sim.launch.py world:=cave_cutaway         # same cave, roof left out of the visual (like the web cut-away view)
  ros2 launch mining_sim_gz sim.launch.py world:=industrial
  ros2 launch mining_sim_gz sim.launch.py world:=cave headless:=true log_dir:=/tmp/run1     # no GUI (EGL rendering)

args: pose_source:=ground_truth|slam (what the explorer plans with), explore:=true|false, rviz:=true|false, seed:=1, size_weight:=0.12
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = {
    # world file -> (truth mask, info json, obstacle band (m above local floor), inflation ladder, ground-truth mesh for segmentation scoring)
    "industrial": ("truth_industrial", "industrial_info.json", (0.1, 1.5), [0.5, 0.4, 0.3], None),
    "cave": ("truth_cave", "cave_info.json", (0.15, 1.7), [0.5, 0.45, 0.4], "models/cave_shell/meshes/cave_shell.obj"),
    "cave_cutaway": ("truth_cave", "cave_info.json", (0.15, 1.7), [0.5, 0.45, 0.4], "models/cave_shell/meshes/cave_shell.obj"),
}
ROBOT_SCALE = 1.35


def launch_setup(context):
    pkg = get_package_share_directory("mining_sim_gz")
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    world = arg("world")
    if world not in WORLDS:
        raise RuntimeError(f"unknown world '{world}', choose one of {list(WORLDS)}")
    truth, info_name, band, inflations, gt_mesh = WORLDS[world]
    info_path = os.path.join(pkg, "config", info_name)
    info = json.load(open(info_path))
    spawn = info["spawn"]
    headless = arg("headless") == "true"
    sim_time = {"use_sim_time": True}
    common = dict(info_json=info_path, pose_source=arg("pose_source"))

    gz_cmd = ["gz", "sim", "-r", os.path.join(pkg, "worlds", f"{world}.sdf")]
    if headless:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]
    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join([os.path.join(pkg, "models"), os.path.join(pkg, "worlds"), os.environ.get("GZ_SIM_RESOURCE_PATH", "")])),
        ExecuteProcess(cmd=gz_cmd, output="screen"),
        TimerAction(period=6.0, actions=[Node(package="ros_gz_sim", executable="create", output="screen", arguments=[
            "-world", world, "-file", os.path.join(pkg, "models", "mapping_robot", "model.sdf"), "-name", "mapping_robot",
            "-x", str(spawn[0]), "-y", str(spawn[1]), "-z", str(spawn[2]), "-Y", str(info["yaw"])])]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml"), **sim_time}]),
    ]
    # static sensor frames (same mounting as the web robot, scaled by the robot scale)
    for child, xyz in (("lidar_link", (0, 0, 0.72 * ROBOT_SCALE)), ("imu_link", (0, 0, 0.3 * ROBOT_SCALE)), ("camera_link", (0.33 * ROBOT_SCALE, 0, 0.38 * ROBOT_SCALE))):
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim_time],
                            arguments=["--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]), "--frame-id", "base_link", "--child-frame-id", child]))
    actions += [
        Node(package="mining_sim_gz", executable="odom_noise_node", output="screen", parameters=[{**sim_time, "seed": int(arg("seed"))}]),
        Node(package="mining_sim_gz", executable="scan_slam_node", output="screen", parameters=[{**sim_time, "info_json": info_path, "log_dir": arg("log_dir")}]),
        Node(package="mining_sim_gz", executable="segmentation_node", output="screen", parameters=[{
            **sim_time, "info_json": info_path, "log_dir": arg("log_dir"), "pose_source": "ground_truth",
            "gt_mesh": os.path.join(pkg, gt_mesh) if gt_mesh else ""}]),
    ]
    if arg("explore") == "true":
        actions.append(Node(package="mining_sim_gz", executable="explorer_node", output="screen", parameters=[{
            **sim_time, **common, "truth_json": os.path.join(pkg, "config", truth + ".json"), "truth_bin": os.path.join(pkg, "config", truth + ".bin"),
            "obstacle_min": band[0], "obstacle_max": band[1], "inflations": inflations, "size_weight": float(arg("size_weight")),
            "sensor_height": 0.72 * ROBOT_SCALE}]))
    actions.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(pkg, "config", "rviz", "mining.rviz")], parameters=[sim_time],
                        condition=IfCondition(LaunchConfiguration("rviz"))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="cave", description="industrial | cave | cave_cutaway"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("pose_source", default_value="ground_truth", description="ground_truth | slam (pose the explorer plans with)"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("size_weight", default_value="0.12"),
        DeclareLaunchArgument("log_dir", default_value="", description="where slam_metrics.json / segmentation.json are written"),
        OpaqueFunction(function=launch_setup),
    ])

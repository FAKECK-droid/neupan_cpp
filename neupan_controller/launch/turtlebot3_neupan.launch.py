# SPDX-License-Identifier: GPL-3.0-or-later
"""Official Humble TurtleBot3 Waffle demo with the NeuPAN controller."""

import json
import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_simulation(context):
    bringup_dir = get_package_share_directory("nav2_bringup")
    with open(LaunchConfiguration("base_params_file").perform(context)) as stream:
        params = yaml.safe_load(stream)
    with open(LaunchConfiguration("controller_params_file").perform(context)) as stream:
        overlay = yaml.safe_load(stream)
    with open(LaunchConfiguration("training_config").perform(context)) as stream:
        robot = yaml.safe_load(stream)["robot"]

    # Replace the entire DWB block; keep the official goal/progress checkers,
    # planner, AMCL, behavior tree, lifecycle management and velocity smoother.
    follow_path = overlay["controller_server"]["ros__parameters"]["FollowPath"]
    follow_path["dune_checkpoint"] = str(
        Path(LaunchConfiguration("dune_checkpoint").perform(context)).expanduser()
    )
    follow_path["max_speed"] = robot["max_speed"]
    follow_path["max_acce"] = robot["max_acce"]
    controller_params = params["controller_server"]["ros__parameters"]
    controller_params["FollowPath"] = follow_path
    # Latch XY arrival while the controller finishes the goal yaw in place.
    for checker in controller_params["goal_checker_plugins"]:
        if controller_params[checker]["plugin"] == "nav2_controller::SimpleGoalChecker":
            controller_params[checker]["stateful"] = True

    # Both costmaps use the same polygon and vertex order as DUNE training.
    # Their original layers, rates and padding remain unchanged.
    footprint = json.dumps(robot["vertices"])
    for costmap in ("local_costmap", "global_costmap"):
        params[costmap][costmap]["ros__parameters"]["footprint"] = footprint

    with tempfile.NamedTemporaryFile(
        mode="w", prefix="neupan_tb3_", suffix=".yaml", delete=False
    ) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        params_file = stream.name

    def cleanup_params(_context):
        os.unlink(params_file)
        return []

    arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "map", "world", "headless", "use_rviz", "use_sim_time",
            "use_composition", "autostart", "x_pose", "y_pose", "yaw",
        )
    }
    arguments["params_file"] = params_file
    return [
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_params)])
        ),
        Node(
            package="neupan_controller",
            executable="scan_to_cloud",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "input_topic": "scan",
                "output_topic": follow_path["points_topic"],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "tb3_simulation_launch.py")
            ),
            launch_arguments=arguments.items(),
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory("neupan_controller")
    bringup_dir = get_package_share_directory("nav2_bringup")
    turtlebot_dir = get_package_share_directory("turtlebot3_gazebo")
    model_path = Path.home() / ".ros/neupan/turtlebot3_waffle/turtlebot3_waffle.bin"
    defaults = {
        "base_params_file": (
            os.path.join(bringup_dir, "params", "nav2_params.yaml"),
            "Official full Nav2 parameter file used as the base",
        ),
        "controller_params_file": (
            os.path.join(package_dir, "config", "turtlebot3_waffle_controller.yaml"),
            "NeuPAN FollowPath parameter overlay",
        ),
        "training_config": (
            os.path.join(package_dir, "config", "turtlebot3_waffle_train.yaml"),
            "DUNE training configuration containing the shared robot polygon",
        ),
        "dune_checkpoint": (str(model_path), "Trained Waffle NPTF .bin file"),
        "map": (
            os.path.join(bringup_dir, "maps", "turtlebot3_world.yaml"),
            "Occupancy map matching the Gazebo world",
        ),
        "world": (
            os.path.join(bringup_dir, "worlds", "world_only.model"),
            "Official Gazebo Classic world",
        ),
        "headless": ("False", "Disable the Gazebo GUI"),
        "use_rviz": ("True", "Start RViz"),
        "use_sim_time": ("True", "Use the simulator clock"),
        # Avoid stalling all navigation node loads on a lost load_node response.
        "use_composition": ("False", "Use official Nav2 composed bringup"),
        "autostart": ("True", "Activate Nav2 lifecycle nodes automatically"),
        "x_pose": ("-2.0", "Initial robot x position"),
        "y_pose": ("-0.5", "Initial robot y position"),
        "yaw": ("0.0", "Initial robot yaw"),
    }
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value, description=description)
            for name, (value, description) in defaults.items()
        ],
        # Defining GAZEBO_MODEL_PATH replaces Gazebo's compiled default.
        # Keep sun/ground_plane as well as the TurtleBot meshes available locally.
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            [
                "/usr/share/gazebo-11/models",
                os.pathsep,
                os.path.join(turtlebot_dir, "models"),
                os.pathsep,
                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            ],
        ),
        # The complete demo is installed locally. Avoid the legacy HTTP database
        # scan, which otherwise blocks startup when an online request hangs.
        SetEnvironmentVariable(
            "GAZEBO_MODEL_DATABASE_URI",
            (Path(package_dir) / "config/gazebo_offline").as_uri(),
        ),
        # Official spawn_entity.py uses /usr/bin/env python3 and needs Ubuntu's
        # ROS/lxml packages, not the isolated CUDA training environment.
        SetEnvironmentVariable(
            "PATH", ["/usr/bin", os.pathsep, EnvironmentVariable("PATH")]
        ),
        OpaqueFunction(function=launch_simulation),
    ])

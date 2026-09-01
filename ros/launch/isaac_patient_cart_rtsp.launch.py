"""Start Isaac Sim and the Quest/OpenXR dVRK patient-cart system."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_SCENE = "ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml"


def _absolute_or_package_path(value, config_directory, default_name):
    value = value.strip() or default_name
    value = os.path.expanduser(value)
    return value if os.path.isabs(value) else os.path.join(config_directory, value)


def _launch_setup(context, *args, **kwargs):
    share_directory = get_package_share_directory("saw_openxr")
    isaac_share_directory = get_package_share_directory("dvrk_isaac_sim")
    config_directory = os.path.join(share_directory, "config")
    system_config = _absolute_or_package_path(
        LaunchConfiguration("system_config").perform(context),
        config_directory,
        "system-MTML-MTMR-OpenXR-patient-cart-ROS.json",
    )
    if not os.path.isfile(system_config):
        raise RuntimeError("dVRK system configuration does not exist: " + system_config)

    startup_delay = float(LaunchConfiguration("dvrk_system_delay").perform(context))
    if startup_delay < 0.0:
        raise RuntimeError('Launch argument "dvrk_system_delay" must not be negative.')

    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(isaac_share_directory, "launch", "simulator.launch.py")),
        launch_arguments={
            "config": LaunchConfiguration("isaac_config"),
            "isaac_sim_dir": LaunchConfiguration("isaac_sim_dir"),
            "scene": LaunchConfiguration("isaac_scene"),
            "headless": LaunchConfiguration("isaac_headless"),
        }.items(),
    )

    # The system JSON's configure-parameter is relative to this directory.
    # The installed default selects sawOpenXR-isaac-rtsp.json.
    dvrk_system = Node(
        package="dvrk_robot",
        executable="dvrk_system",
        name="dvrk_system",
        output="screen",
        cwd=share_directory,
        arguments=["--json-config", system_config],
    )
    return [isaac_launch, TimerAction(period=startup_delay, actions=[dvrk_system])]


def generate_launch_description():
    isaac_share_directory = get_package_share_directory("dvrk_isaac_sim")
    return LaunchDescription([
        DeclareLaunchArgument(
            "system_config",
            default_value="system-MTML-MTMR-OpenXR-patient-cart-ROS.json",
            description="Installed config filename or an absolute dvrk_system JSON path.",
        ),
        DeclareLaunchArgument(
            "isaac_config",
            default_value=os.path.join(isaac_share_directory, "share", "isaac_sim.yaml"),
            description="dvrk_isaac_sim simulator configuration YAML.",
        ),
        DeclareLaunchArgument(
            "isaac_scene",
            default_value=DEFAULT_SCENE,
            description="Isaac scene filename or an absolute scene YAML path.",
        ),
        DeclareLaunchArgument(
            "isaac_sim_dir",
            default_value="",
            description="Optional Isaac Sim directory override.",
        ),
        DeclareLaunchArgument(
            "isaac_headless",
            default_value="true",
            description="Isaac Sim headless override; true runs without the desktop window.",
        ),
        DeclareLaunchArgument(
            "dvrk_system_delay",
            default_value="15.0",
            description="Seconds to wait for Isaac Sim before starting dvrk_system.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])

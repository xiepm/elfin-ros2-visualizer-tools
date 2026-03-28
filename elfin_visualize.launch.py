#!/usr/bin/env python3
import tempfile
from pathlib import Path
from string import Template

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


WORKSPACE_ROOT = Path("/home/xpm/workspace/ros2_ws")
RVIZ_CONFIG = WORKSPACE_ROOT / "elfin_visualize.rviz"
MODEL_XACRO_MAP = {
    "elfin3": WORKSPACE_ROOT / "src/elfin_robot_ros2/elfin3/elfin3_ros2_gazebo/urdf/elfin3.urdf.xacro",
    "elfin5": WORKSPACE_ROOT / "src/elfin_robot_ros2/elfin5/elfin5_ros2_gazebo/urdf/elfin5.urdf.xacro",
    "elfin5_l": WORKSPACE_ROOT / "src/elfin_robot_ros2/elfin5_l/elfin5_l_ros2_gazebo/urdf/elfin5_l.urdf.xacro",
    "elfin10": WORKSPACE_ROOT / "src/elfin_robot_ros2/elfin10/elfin10_ros2_gazebo/urdf/elfin10.urdf.xacro",
    "elfin10_l": WORKSPACE_ROOT / "src/elfin_robot_ros2/elfin10_l/elfin10_l_ros2_gazebo/urdf/elfin10_l.urdf.xacro",
    "elfin15": WORKSPACE_ROOT / "src/elfin_robot_ros2/elfin15/elfin15_ros2_gazebo/urdf/elfin15.urdf.xacro",
}


def generate_visualization_assets(model):
    xacro_file = MODEL_XACRO_MAP[model]
    robot_description_xml = xacro.process_file(
        str(xacro_file),
        mappings={
            "use_real_hardware": "false",
            "use_fake_hardware": "true",
        },
    ).toxml()

    temp_dir = Path(tempfile.gettempdir())
    urdf_file = temp_dir / f"elfin_visualize_{model}.urdf"
    rviz_file = temp_dir / f"elfin_visualize_{model}.rviz"

    urdf_file.write_text(robot_description_xml, encoding="utf-8")

    rviz_template = Template(RVIZ_CONFIG.read_text(encoding="utf-8"))
    rviz_file.write_text(
        rviz_template.safe_substitute(urdf_file=str(urdf_file)),
        encoding="utf-8",
    )

    return robot_description_xml, urdf_file, rviz_file


def build_nodes(context):
    model = LaunchConfiguration("model").perform(context)
    robot_description_xml, urdf_file, rviz_file = generate_visualization_assets(model)
    robot_description = {"robot_description": robot_description_xml}

    return [
        LogInfo(msg=f"[elfin_visualize] model={model} urdf={urdf_file} rviz={rviz_file}"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        TimerAction(
            period=0.5,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "python3",
                        str(WORKSPACE_ROOT / "control_elfin.py"),
                        "--model",
                        model,
                        "--mode",
                        "joint_state_only",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", str(rviz_file)],
                    output="screen",
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs"),
            DeclareLaunchArgument(
                "model",
                default_value="elfin5",
                description="Elfin model: elfin3, elfin5, elfin5_l, elfin10, elfin10_l, elfin15",
            ),
            OpaqueFunction(function=build_nodes),
        ]
    )

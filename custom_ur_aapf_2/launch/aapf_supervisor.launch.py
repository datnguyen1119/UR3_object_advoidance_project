"""
Launch the AAPF Supervisor v2 + ObstacleManager together.

Both read parameters from `config/aapf_params.yaml`. Run alongside the existing
moveit_servo launch:

    Terminal 1:
      ros2 launch custom_ur_simulation spawn_moveit_camera_gripper.launch.py \\
          with_gripper:=true

    Terminal 2:
      ros2 launch custom_ur_servo spawn_moveit_servo_2.launch.py \\
          with_gripper:=true
      ros2 service call /custom_moveit_servo/start_servo std_srvs/srv/Trigger {}

    Terminal 3 (this launch):
      ros2 launch custom_ur_aapf_2 aapf_supervisor.launch.py
      ros2 service call /aapf_supervisor/start std_srvs/srv/Trigger {}

    Terminal 4 — plan a path in RViz, spawn an obstacle on it, watch the FSM:
      ros2 service call /obstacle_manager/spawn std_srvs/srv/Trigger {}
      ros2 topic echo /aapf_supervisor/mode_text
      ros2 topic echo /aapf_supervisor/error_text

NOTE: this is a separate package from custom_ur_aapf and reuses the SAME node
names (aapf_supervisor / obstacle_manager) — launch only ONE of the two
packages at a time to avoid node-name clashes.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name: str, relative_path: str):
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory("custom_ur_aapf_2")
    default_params = os.path.join(pkg_share, "config", "aapf_params.yaml")

    # Kinematics solver (KDL) for RobotState::setFromIK in the re-IK resume path.
    # The PSM gets robot_description / _semantic from topics, but
    # robot_description_kinematics is a PARAMETER only — it must be loaded into
    # THIS node's namespace or setFromIK has no solver. Source: ur3_moveit_config.
    kinematics_yaml = load_yaml("ur3_moveit_config", "config/kinematics.yaml")
    kinematics_params = (
        {"robot_description_kinematics": kinematics_yaml} if kinematics_yaml else {}
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to YAML with both aapf_supervisor and obstacle_manager params.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Match Gazebo/MoveIt clock source.",
    )

    common_params = [
        LaunchConfiguration("params_file"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    supervisor = Node(
        package="custom_ur_aapf_2",
        executable="aapf_supervisor_node",
        name="aapf_supervisor",
        output="screen",
        parameters=common_params + [kinematics_params],
    )

    obstacle_manager = Node(
        package="custom_ur_aapf_2",
        executable="obstacle_manager_node",
        name="obstacle_manager",
        output="screen",
        parameters=common_params,
    )

    # Peer obstacle source: a draggable RViz InteractiveMarker. Uses its own
    # collision-object id, so it coexists with obstacle_manager's test_sphere.
    interactive_obstacle = Node(
        package="custom_ur_aapf_2",
        executable="interactive_obstacle_node",
        name="interactive_obstacle",
        output="screen",
        parameters=common_params,
    )

    # Joint velocity estimator: q̇ from joint-position finite-difference, published
    # on /arm_joint_velocities for the supervisor's singularity-coast hold.
    joint_velocity_estimator = Node(
        package="custom_ur_aapf_2",
        executable="joint_velocity_estimator_node",
        name="joint_velocity_estimator",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        supervisor,
        obstacle_manager,
        interactive_obstacle,
        joint_velocity_estimator,
    ])

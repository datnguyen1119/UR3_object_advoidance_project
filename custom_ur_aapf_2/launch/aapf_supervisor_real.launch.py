#  Bản REAL của aapf_supervisor.launch.py — Stage 2 (né vật cản trên robot thật).
#
#  Khác bản sim ở 2 điểm:
#    1. Đọc config RIÊNG: config/aapf_params_real.yaml (scaled JTC + controller switch
#       đã bake sẵn trong yaml, không còn override inline). Bản sim dùng aapf_params.yaml.
#    2. use_sim_time -> false
#
#  Chạy SAU bringup + servo_real:
#    ros2 launch custom_ur_aapf_2 aapf_supervisor_real.launch.py
#    ros2 service call /aapf_supervisor/start std_srvs/srv/Trigger {}

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
    # Config RIÊNG cho robot thật (scaled JTC + controller switch đã bake sẵn).
    default_params = os.path.join(pkg_share, "config", "aapf_params_real.yaml")

    # Kinematics solver (KDL) for RobotState::setFromIK in the re-IK resume path.
    # The real Servo launch deliberately omits kinematics for SERVO (it uses
    # inverse-Jacobian), but the SUPERVISOR node still needs it for re-IK — they are
    # different nodes. robot_description_kinematics is a PARAM only, so load it here.
    kinematics_yaml = load_yaml("ur3_moveit_config", "config/kinematics.yaml")
    kinematics_params = (
        {"robot_description_kinematics": kinematics_yaml} if kinematics_yaml else {}
    )

    params_file_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")  # REAL: false

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

    interactive_obstacle = Node(
        package="custom_ur_aapf_2",
        executable="interactive_obstacle_node",
        name="interactive_obstacle",
        output="screen",
        parameters=common_params,
    )

    # q̇ estimator (joint-position finite-difference) → /arm_joint_velocities.
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

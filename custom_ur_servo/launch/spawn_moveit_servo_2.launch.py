from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import yaml


def load_yaml(package_name: str, relative_path: str):
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    sim_share   = get_package_share_directory("custom_ur_simulation")
    desc_share  = get_package_share_directory("custom_ur_description")
    servo_share = get_package_share_directory("custom_ur_servo")

    # --- args ---
    with_gripper = DeclareLaunchArgument(
        "with_gripper", default_value="true",
        description="Phải khớp với arg đã chạy trong spawn_moveit_camera_gripper.launch.py."
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    ld.add_action(with_gripper)
    ld.add_action(use_sim_time_arg)

    # Resolve with_gripper sớm để dùng cho mappings (xacro arg). Vì MoveItConfigsBuilder
    # phải process xacro lúc build description, không thể substitute LaunchConfiguration sau.
    _wg = "true"
    for _a in sys.argv:
        if _a.startswith("with_gripper:="):
            _wg = _a.split(":=", 1)[1].strip().lower()
            break

    # --- MoveIt config (file_path giữ NGUYÊN giống spawn_moveit_camera_gripper) ---
    joint_controllers_file = os.path.join(sim_share, "config", "ur3_controllers_gripper.yaml")

    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur3_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur3",
                "sim_gazebo": "true",
                "sim_ignition": "false",
                "use_fake_hardware": "false",
                "with_gripper": _wg,
                "simulation_controllers": joint_controllers_file,
                "initial_positions_file": os.path.join(desc_share, "config", "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --- Servo params ---
    servo_yaml = load_yaml("custom_ur_servo", "config/moveit_servo.yaml")
    if servo_yaml is None:
        raise RuntimeError("Failed to load config/moveit_servo.yaml")

    # moveit_servo declares params dưới namespace "moveit_servo" — phải wrap,
    # nếu không Servo dùng default (move_group_name="panda_arm") và crash.
    servo_params = {"moveit_servo": servo_yaml}

    # --- Servo node ---
    servo_node = Node(
        package="custom_ur_servo",
        executable="custom_moveit_servo",
        name="custom_moveit_servo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    ld.add_action(servo_node)

    return ld

#  Bản REAL của spawn_moveit_servo_2.launch.py — Stage 2 (teleop trên robot thật).
#
#  Khác bản sim ở 3 điểm (CHECKLIST #2 dây Servo + #3 clock):
#    1. command_out_topic  -> /scaled_joint_trajectory_controller/joint_trajectory
#    2. use_gazebo         -> false  (bỏ delay dành cho Gazebo)
#    3. use_sim_time       -> false ; sim_gazebo:=false trong robot_description
#
#  Chạy SAU khi bringup_real_camera_gripper.launch.py đã lên và robot đã Play:
#    ros2 launch custom_ur_servo spawn_moveit_servo_real.launch.py
#    ros2 service call /custom_moveit_servo/start_servo std_srvs/srv/Trigger {}

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import yaml


def load_yaml(package_name, relative_path):
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    ld = LaunchDescription()

    sim_share  = get_package_share_directory("custom_ur_simulation")
    desc_share = get_package_share_directory("custom_ur_description")

    ld.add_action(DeclareLaunchArgument("with_gripper", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))  # REAL: false

    _wg = "true"
    for _a in sys.argv:
        if _a.startswith("with_gripper:="):
            _wg = _a.split(":=", 1)[1].strip().lower()
            break

    joint_controllers_file = os.path.join(sim_share, "config", "ur3_controllers_gripper.yaml")

    # robot_description cho HARDWARE THẬT (sim_gazebo=false). Servo chỉ dùng phần kinematics,
    # nhưng để nhất quán với bringup ta build cùng cờ.
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur3_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur3",
                "sim_gazebo": "false",
                "sim_ignition": "false",
                "use_fake_hardware": "false",
                "with_gripper": _wg,
                "simulation_controllers": joint_controllers_file,
                "initial_positions_file": os.path.join(desc_share, "config", "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers_real.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Nạp moveit_servo.yaml rồi PATCH cho robot thật (không cần file yaml riêng).
    servo_yaml = load_yaml("custom_ur_servo", "config/moveit_servo.yaml")
    if servo_yaml is None:
        raise RuntimeError("Failed to load config/moveit_servo.yaml")
    # --- HƯỚNG 2: stream THẲNG vị trí vào forward_position_controller ---
    # scaled_jtc không hợp streaming (nội suy/ngoại suy mỗi chu kỳ -> rung -> spike -> protective stop).
    # JointGroupPositionController áp setpoint vị trí NGAY xuống servoj -> mượt, không spike.
    servo_yaml["command_out_type"]  = "std_msgs/Float64MultiArray"
    servo_yaml["command_out_topic"] = "/forward_position_controller/commands"
    servo_yaml["publish_joint_positions"]  = True
    servo_yaml["publish_joint_velocities"] = False  # Float64MultiArray = vị trí; KHÔNG gửi velocity (tránh overshoot)
    servo_yaml["use_gazebo"] = False
    servo_yaml["publish_period"] = 0.03             # 100 Hz, khớp update_rate controller
    servo_yaml["low_pass_filter_coeff"] = 2.0       # lọc vừa phải (tăng nếu còn rung, giảm nếu trễ)
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="custom_ur_servo",
        executable="custom_moveit_servo",
        name="custom_moveit_servo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # KINEMATICS CỐ Ý KHÔNG NẠP: nếu có IK solver (KDL), Servo dùng đường
            # searchPositionIK (lặp, timeout 5ms, nghiệm xấp xỉ) -> giải thiếu mỗi
            # chu kỳ -> EE chậm ~10x dù twist lệnh đúng. Bỏ kinematics => nhóm không
            # có solver => Servo dùng INVERSE-JACOBIAN (delta_theta = J⁺·delta_x),
            # chính xác cho vận tốc lệnh và nhanh hơn (cách MoveIt khuyến nghị để servo).
            # moveit_config.robot_description_kinematics,
            servo_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    ld.add_action(servo_node)

    return ld

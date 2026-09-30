#  ┌──────────────────────────────────────────────────────────────────────────┐
#  │  BRINGUP ROBOT THẬT — Cách B (tự dựng tầng đáy, không qua ur_control.launch)│
#  └──────────────────────────────────────────────────────────────────────────┘
#
#  Đây là bản song sinh real-hardware của spawn_moveit_camera_gripper.launch.py.
#  Khác biệt DUY NHẤT là TẦNG ĐÁY: thay Gazebo + spawn_entity bằng
#  ur_ros2_control_node (controller_manager thật) nói chuyện với:
#     - TAY MÁY  qua TCP   (robot_ip)
#     - GRIPPER  qua serial (com_port, RobotiqGripperHardwareInterface)
#  Ba tầng trên (move_group / rviz, và — chạy riêng — servo + aapf) giữ nguyên.
#
#  ⚠️  CHƯA chạy thử trên phần cứng. Đọc kỹ thủ tục bring-up an toàn ở cuối file
#      trước khi cấp nguồn cho robot.
#
#  Chạy:
#    ros2 launch custom_ur_simulation bringup_real_camera_gripper.launch.py \
#        robot_ip:=<IP_ROBOT> com_port:=/dev/ttyUSB0 with_gripper:=true

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys as _sys


# MoveItConfigsBuilder process xacro NGAY lúc build description, nên các giá trị
# ảnh hưởng tới nội dung URDF (robot_ip, com_port, gripper...) phải là STRING có
# sẵn — không thể là LaunchConfiguration. Đọc trực tiếp từ argv (cùng mẹo file gốc).
def _argv(key, default):
    for a in _sys.argv:
        if a.startswith(key + ":="):
            return a.split(":=", 1)[1].strip()
    return default


def generate_launch_description():
    ld = LaunchDescription()

    sim_share  = get_package_share_directory("custom_ur_simulation")
    desc_share = get_package_share_directory("custom_ur_description")

    # ---- args (CHECKLIST #1: hardware plugin + IP) ----
    robot_ip    = _argv("robot_ip",    "192.168.1.1")          # BẮT BUỘC truyền IP thật
    com_port    = _argv("com_port",    "/dev/ttyUSB0")     # cổng serial gripper
    headless    = _argv("headless_mode", "false")          # false = phải bấm Play "External Control" trên pendant
    _wg         = _argv("with_gripper", "true").lower()  # mặc định true để khớp SRDF (group robotiq_gripper luôn có)

    ld.add_action(DeclareLaunchArgument("robot_ip",     default_value=robot_ip))
    ld.add_action(DeclareLaunchArgument("com_port",     default_value=com_port))
    ld.add_action(DeclareLaunchArgument("headless_mode",default_value=headless))
    ld.add_action(DeclareLaunchArgument("with_gripper", default_value=_wg,
        description="true = thêm LINK gripper (khớp SRDF, tránh lỗi 'not known to URDF'). "
                    "Hiện chỉ là visual/collision, CHƯA có hardware interface."))
    # Tách riêng: chỉ spawn gripper_position_controller khi đã wire gripper hardware (Stage 2).
    ld.add_action(DeclareLaunchArgument("spawn_gripper_controller", default_value="false"))
    ld.add_action(DeclareLaunchArgument("with_rviz",    default_value="true"))
    ld.add_action(DeclareLaunchArgument("with_octomap", default_value="false"))

    # ---- URScript + RTDE recipe (driver cần để upload chương trình điều khiển) ----
    script_filename = os.path.join(
        get_package_share_directory("ur_client_library"), "resources", "external_control.urscript")
    input_recipe = os.path.join(
        get_package_share_directory("ur_robot_driver"), "resources", "rtde_input_recipe.txt")
    output_recipe = os.path.join(
        get_package_share_directory("ur_robot_driver"), "resources", "rtde_output_recipe.txt")

    controllers_file = os.path.join(sim_share, "config", "ur3_controllers_gripper.yaml")

    # ---- robot_description cho HARDWARE THẬT (CHECKLIST #1) ----
    # sim_gazebo=false + use_fake_hardware=false => xacro chọn nhánh
    # URPositionHardwareInterface (tay) + RobotiqGripperHardwareInterface (gripper).
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
                "robot_ip": robot_ip,
                "com_port": com_port,
                "headless_mode": headless,
                "script_filename": script_filename,
                "input_recipe_filename": input_recipe,
                "output_recipe_filename": output_recipe,
                "simulation_controllers": controllers_file,
                "initial_positions_file": os.path.join(desc_share, "config", "initial_positions.yaml"),
                # kinematics_params: ĐỂ MẶC ĐỊNH (chưa có calibration của con robot này).
                #   Khi có file calibration, thêm:
                #   "kinematics_params": "<đường dẫn>/my_robot_calibration.yaml",
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        # CHECKLIST #2: move_group thực thi qua scaled_joint_trajectory_controller (bản _real)
        .trajectory_execution(file_path="config/moveit_controllers_real.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # CHECKLIST #3: use_sim_time = False ở MỌI node (không còn /clock của Gazebo).
    use_sim_time = {"use_sim_time": False}

    # ---- robot_state_publisher ----
    ld.add_action(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time],
    ))

    # ---- TẦNG ĐÁY: controller_manager UR-specific (thay gazebo_ros2_control) ----
    # ur_ros2_control_node nạp robot_description (có 2 khối <ros2_control>) + controllers.yaml.
    ur_control_node = Node(
        package="ur_robot_driver", executable="ur_ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description, controllers_file, use_sim_time],
    )
    ld.add_action(ur_control_node)

    # ---- AN TOÀN: dừng motion controller khi chương trình robot ngừng/được nhả ----
    ld.add_action(Node(
        package="ur_robot_driver", executable="controller_stopper_node",
        name="controller_stopper", output="screen",
        parameters=[
            {"headless_mode": (headless.lower() == "true")},
            {"joint_controller_active": True},
            {"consistent_controllers": [
                "joint_state_broadcaster",
                "io_and_status_controller",
                "speed_scaling_state_broadcaster",
            ]},
            use_sim_time,
        ],
    ))

    # ---- Dashboard client (start/stop/power/trạng thái qua cổng 29999) ----
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ur_robot_driver"), "launch", "ur_dashboard_client.launch.py"])),
        launch_arguments={"robot_ip": robot_ip}.items(),
    ))

    # ---- urscript_interface (gửi URScript thô nếu cần) ----
    ld.add_action(Node(
        package="ur_robot_driver", executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}, use_sim_time],
        output="screen",
    ))

    # ---- CONTROLLER SPAWNERS ----
    def spawner(name, active=True, cond=None, param_file=None):
        args = [name, "--controller-manager", "/controller_manager"]
        if not active:
            args.append("--inactive")
        if param_file:
            args += ["--param-file", param_file]
        return Node(package="controller_manager", executable="spawner",
                    arguments=args, output="screen",
                    condition=IfCondition(cond) if cond else None)

    # extra_joints: cho broadcaster tự publish state khớp gripper (không có HW interface)
    # -> tắt warning "Missing robotiq_85_left_knuckle_joint" của MoveIt.
    jsb_extra = os.path.join(sim_share, "config", "jsb_extra_joints_real.yaml")

    # Broadcasters + IO trước (controller_stopper & scaled JTC phụ thuộc chúng).
    broadcasters = [
        spawner("joint_state_broadcaster", param_file=jsb_extra),
        spawner("io_and_status_controller"),
        spawner("speed_scaling_state_broadcaster"),
        # force_torque_sensor_broadcaster ĐÃ BỎ: robot là UR3 CB3 (không có FT sensor),
        # và type trong yaml ('ur_controllers/ForceTorqueStateBroadcaster') cũng sai/không tồn tại
        # (đúng phải là 'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster').
    ]
    # CHECKLIST #2: motion controller = scaled_joint_trajectory_controller (tôn trọng speed slider).
    # LƯU Ý: gripper_position_controller gác bằng arg RIÊNG `spawn_gripper_controller`
    # (KHÔNG phải with_gripper). Vì với with_gripper:=true gripper hiện chỉ là link
    # visual/collision (include_ros2_control="false" -> không có hardware interface),
    # spawn gripper controller lúc này sẽ FAIL. Để dành Stage 2 khi wire gripper hardware.
    motion = [
        spawner("scaled_joint_trajectory_controller"),
        # forward_position_controller: NẠP SẴN nhưng INACTIVE. Dùng cho Servo (teleop/AAPF):
        # khi servo -> switch scaled_jtc OFF / forward_position ON (xem hướng dẫn trong launch servo_real).
        # Không thể cùng active với scaled_jtc (tranh command interface 6 khớp).
        spawner("forward_position_controller", active=False),
        spawner("gripper_position_controller", cond=LaunchConfiguration("spawn_gripper_controller")),
    ]
    # Spawn broadcasters ngay khi CM lên (spawner tự chờ controller_manager);
    # motion controller hoãn 3s để chắc io_and_status đã active.
    for s in broadcasters:
        ld.add_action(s)
    ld.add_action(TimerAction(period=3.0, actions=motion))

    # ---- TẦNG TRÊN: move_group + rviz (giống file Gazebo, chỉ khác use_sim_time) ----
    mg_params = moveit_config.to_dict()
    mg_params.update(use_sim_time)
    # FIX "robot không tới đích": khi speed slider trên pendant < 100%,
    # scaled_joint_trajectory_controller chạy CHẬM hơn thời lượng MoveIt dự tính
    # -> bộ giám sát thời lượng của MoveIt tưởng "treo" và HỦY quỹ đạo giữa chừng
    # (status TIMED_OUT). Với hardware có speed-scaling, thời lượng thực không đoán
    # trước được -> tắt giám sát thời lượng (hoặc nới rất rộng).
    mg_params.update({
        "trajectory_execution.execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    })

    ld.add_action(Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[mg_params], arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(LaunchConfiguration("with_octomap")),
    ))
    mg_no_sensors = dict(mg_params); mg_no_sensors.pop("sensors", None)
    ld.add_action(Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[mg_no_sensors], arguments=["--ros-args", "--log-level", "info"],
        condition=UnlessCondition(LaunchConfiguration("with_octomap")),
    ))

    rviz_cfg = os.path.join(sim_share, "rviz", "interactive_obstacle.rviz")
    ld.add_action(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz")),
    ))

    return ld

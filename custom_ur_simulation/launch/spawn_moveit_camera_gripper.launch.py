from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    sim_share       = get_package_share_directory("custom_ur_simulation")
    desc_share      = get_package_share_directory("custom_ur_description")
    robotiq_share   = get_package_share_directory("robotiq_description")
    ur_share        = get_package_share_directory("ur_description")
    gazebo_ros_dir  = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(sim_share, 'worlds', 'ur3_workspace.world')

    # --- ENV Gazebo ---
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(["/usr/share/gazebo-11", sim_share, desc_share, robotiq_share, ur_share])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join([
            os.path.join(desc_share, "models"),
            os.path.join(robotiq_share, "models"),
            os.path.expanduser("~/.gazebo/models")
        ])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join([
            "/opt/ros/humble/lib",
            os.path.normpath(os.path.join(sim_share, "..", "..", "lib")),
        ])
    ))

    # --- args ---
    with_rviz     = DeclareLaunchArgument("with_rviz", default_value="true")
    with_octomap  = DeclareLaunchArgument("with_octomap", default_value="false",
                                          description="True = move_group dùng sensor 3D dựng octomap "
                                                      "(các voxel hình hộp trong planning scene). "
                                                      "False = tắt octomap, scene chỉ còn vật cản bạn tự thêm (vd sphere).")
    with_gripper  = DeclareLaunchArgument("with_gripper", default_value="false",
                                          description="True = spawn Robotiq gripper trong URDF + spawn gripper_position_controller. "
                                                      "False = chỉ có arm UR3, giảm collision link để APF dễ né wall.")
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    ld.add_action(with_rviz); ld.add_action(with_octomap); ld.add_action(with_gripper)
    ld.add_action(x_arg); ld.add_action(y_arg); ld.add_action(z_arg)

    # Resolve with_gripper sớm để dùng cho mappings (xacro arg). Vì MoveItConfigsBuilder
    # phải process xacro lúc build description, không thể substitute LaunchConfiguration sau.
    # → đọc trực tiếp từ sys.argv hoặc set qua perform_substitutions.
    # Cách đơn giản nhất: dùng env-style — kiểm tra `with_gripper:=xxx` trong sys.argv.
    import sys as _sys
    _wg = "true"
    for _a in _sys.argv:
        if _a.startswith("with_gripper:="):
            _wg = _a.split(":=", 1)[1].strip().lower()
            break

    # --- MoveIt config ---
    joint_controllers_file = os.path.join(sim_share, "config", "ur3_controllers_gripper.yaml")
    robot_xacro_file = os.path.join(
        desc_share,
        "urdf",
        "ur3_gripper.urdf.xacro",
    )

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
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # --- Gazebo ---
    # paused=false: cần /clock tick để spawner (use_sim_time) không timeout.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "gui": "true",
            "paused": "false",
            "verbose": "true",
            # "world": world_file
        }.items()
    )
    ld.add_action(gazebo)

    # --- RSP ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn ---
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity","cobot",
            "-topic","robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    ld.add_action(TimerAction(period=5.0, actions=[spawn]))

    jsb  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"], output="screen")
    arm  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_trajectory_controller","--controller-manager","/controller_manager"], output="screen")
    grip = Node(package="controller_manager", executable="spawner",
                arguments=["gripper_position_controller","--controller-manager","/controller_manager"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("with_gripper")))

    # jsb lên trước jtc/grip: MoveIt cần /joint_states để có start state.
    after_spawn_actions = [
        TimerAction(period=3.0, actions=[jsb]),
        TimerAction(period=5.0, actions=[arm, grip]),
    ]
    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=spawn, on_start=after_spawn_actions)
    ))

    # rviz_cfg = os.path.join(get_package_share_directory("ur3_moveit_config"),
    #                         "config", "moveit.rviz")
    rviz_cfg = os.path.join(get_package_share_directory("custom_ur_simulation"),
                        "rviz", "interactive_obstacle.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz"))
    )
    ld.add_action(rviz)

    mg_params = moveit_config.to_dict()
    mg_params.update({"use_sim_time": True})

    move_group_with_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params],
        arguments=["--ros-args","--log-level","info"],
        condition=IfCondition(LaunchConfiguration("with_octomap")),   # << NEW
    )

    mg_params_no_sensors = dict(mg_params)
    mg_params_no_sensors.pop("sensors", None)

    move_group_no_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params_no_sensors],
        arguments=["--ros-args","--log-level","info"],
        condition=UnlessCondition(LaunchConfiguration("with_octomap")),
    )

    

    ld.add_action(move_group_with_octomap)
    ld.add_action(move_group_no_octomap)

    return ld

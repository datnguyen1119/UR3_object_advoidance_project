from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Bật/tắt cầu nối human-arm -> planning scene (né AAPF). Mặc định bật.
    obstacle_bridge_arg = DeclareLaunchArgument(
        'obstacle_bridge', default_value='true',
        description='Chạy human_obstacle_bridge để đẩy cẳng tay vào /planning_scene cho AAPF né.'
    )

    # Tiền tố topic của camera. rs_launch.py MẶC ĐỊNH đẩy vào /camera/camera/... (do
    # camera_namespace='camera' + camera_name='camera'). Node subscribe tương đối
    # nên chỉ cần đổi prefix này. Nếu chạy realsense với camera_namespace:=/ thì
    # đặt camera_prefix:=/camera.
    camera_prefix_arg = DeclareLaunchArgument(
        'camera_prefix', default_value='/camera/camera',
        description='Tiền tố topic camera (mặc định /camera/camera của rs_launch.py).'
    )
    cam = LaunchConfiguration('camera_prefix')

    # Hệ số phóng to cửa sổ hiển thị (USB2 ảnh nhỏ -> để 2.0~3.0 cho dễ nhìn).
    display_scale_arg = DeclareLaunchArgument(
        'display_scale', default_value='2.0',
        description='Phóng to cửa sổ video (1.0 = gốc, 2.0 = gấp đôi).'
    )

    real_arm_node = Node(
        package='custom_ur_human_tracking',
        executable='real_arm_node',
        name='real_arm_node',
        output='screen',
        parameters=[{'display_scale': LaunchConfiguration('display_scale')}],
        remappings=[
            ('/camera/color/image_raw',                  [cam, '/color/image_raw']),
            ('/camera/color/camera_info',                [cam, '/color/camera_info']),
            ('/camera/aligned_depth_to_color/image_raw', [cam, '/aligned_depth_to_color/image_raw']),
        ],
    )

    obstacle_bridge = Node(
        package='custom_ur_human_tracking',
        executable='human_obstacle_bridge',
        name='human_obstacle_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('obstacle_bridge')),
        parameters=[{
            'num_spheres': 5,
            'radius': 0.05,         # bán kính mỗi sphere cẳng tay [m]
            'hand_radius': 0.07,     # bán kính cầu bàn tay [m]
            'hand_offset': 0.05,     # tâm bàn tay cách cổ tay [m] theo hướng cẳng tay
            'stale_timeout': 0.5,
        }],
    )

    return LaunchDescription([
        obstacle_bridge_arg,
        camera_prefix_arg,
        display_scale_arg,
        real_arm_node,
        obstacle_bridge,
    ])

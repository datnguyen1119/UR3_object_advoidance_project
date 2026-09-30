ros2 topic echo /aapf_supervisor/obstacle_distance &

ros2 param set /obstacle_manager obstacle.x 0.5
ros2 param set /obstacle_manager obstacle.z 0.3

ros2 service call /obstacle_manager/spawn std_srvs/srv/Trigger {}
ros2 service call /obstacle_manager/remove std_srvs/srv/Trigger {}

# distance = X

ros2 param set /obstacle_manager obstacle.x 5.0
ros2 service call /obstacle_manager/spawn std_srvs/srv/Trigger {}
# distance phải tăng nhiều (vì sphere xa hẳn)

# aapf
ros2 launch custom_ur_aapf_2 aapf_supervisor.launch.py
ros2 service call /aapf_supervisor/start std_srvs/srv/Trigger {}

# launch moveit và rviz
ros2 launch custom_ur_simulation spawn_moveit_camera_gripper.launch.py with_gripper:=true

# launch moveit servo
ros2 launch custom_ur_servo spawn_moveit_servo_2.launch.py with_gripper:=true

# check moveit servo nào được sử dụng
ros2 pkg prefix moveit_servo

- phải nằm trong file install của ws mới là ổn, còn của hê thống thì không được


ros2 control list_hardware_components      # 'ur' -> state: active
ros2 control list_controllers              # scaled_joint_trajectory_controller -> active
ros2 topic echo /joint_states --once       # 6 góc khớp pose thật trên pendant

# ============================================================
# Chạy trên ROBOT THẬT  (mỗi T = một terminal; source install/setup.bash ở MỌI terminal)
# ============================================================
source install/setup.bash

# --- T1 — bringup tầng đáy + move_group ---
ros2 launch custom_ur_simulation bringup_real_camera_gripper.launch.py robot_ip:=192.168.1.1
#   → rồi PLAY External Control trên pendant, speed slider ~10-20%
#   kiểm tra trước khi đi tiếp:
ros2 control list_controllers          # scaled_joint_trajectory_controller + forward_position_controller phải LOAD
ros2 control list_hardware_components   # 'ur' -> active

# --- T2 — MoveIt Servo (bản real: forward_position_controller + use_gazebo=false) ---
ros2 launch custom_ur_servo spawn_moveit_servo_real.launch.py
ros2 service call /custom_moveit_servo/start_servo std_srvs/srv/Trigger {}

# --- T3 — AAPF supervisor (real) + obstacle/estimator (tự khởi động trong launch) ---
#   LƯU Ý: dùng đúng launch real → tự nạp aapf_params_real.yaml
#   (enable_controller_switch=true, scaled<->forward, enable_singularity_coast=true)
ros2 launch custom_ur_aapf_2 aapf_supervisor_real.launch.py
ros2 service call /aapf_supervisor/start std_srvs/srv/Trigger {}

# --- T4 — quan sát + thả vật cản lên đường đã plan ---
ros2 topic echo /aapf_supervisor/mode_text          # FOLLOW / AAPF_AVOID / RETURN / STOP
ros2 service call /obstacle_manager/spawn std_srvs/srv/Trigger {}

# ---- Chẩn đoán singularity-coast (trục 2) ----
ros2 param get /aapf_supervisor enable_singularity_coast    # phải true
ros2 param get /aapf_supervisor singularity.kappa_enter     # ngưỡng đang chạy (YAML, KHÔNG phải .hpp)
ros2 topic echo /aapf_supervisor/condition_number           # κ liên tục (mọi mode)
ros2 topic echo /aapf_supervisor/near_singularity           # true khi κ > kappa_enter
ros2 topic echo /arm_joint_velocities                       # q̇ từ JointVelocityEstimator
ros2 topic echo /delta_twist_cmds                           # kênh Cartesian (κ thấp)
ros2 topic echo /delta_joint_cmds                           # kênh JOINT_HOLD (κ cao, coast)
ros2 topic echo /custom_moveit_servo/status                 # 0 ok / 1 vào sing / 6 ra sing / 2 halt

# ---- Khôi phục sau STOP ----
ros2 service call /aapf_supervisor/reset std_srvs/srv/Trigger {}

ros2 topic echo /aapf_supervisor/ee_speed

# ============================================================
# NHẬN DIỆN TAY NGƯỜI BẰNG CAMERA  → né AAPF
#   Node: custom_ur_human_tracking (MediaPipe PoseLandmarker + AprilTag)
#   Bridge: human_obstacle_bridge  (cẳng tay → 5 sphere trên /planning_scene)
#   Stack chạy numpy 2 (mediapipe 0.10.35 Tasks API, KHÔNG dùng cv_bridge)
# ============================================================

# --- T0 — DRIVER CAMERA REALSENSE (bắt buộc, chạy TRƯỚC human_tracking) ---
#   Cài 1 lần nếu chưa có:  sudo apt install ros-humble-realsense2-camera
#   Nếu node chết vì libdiagnostic_updater.so: sudo apt install --only-upgrade ros-humble-diagnostic-updater
#   PHẢI bật align_depth để có /camera/aligned_depth_to_color/image_raw

#   rs_launch.py mặc định publish vào /camera/camera/...  -> human_tracking.launch.py
#   ĐÃ tự remap về đúng prefix đó (arg camera_prefix, mặc định /camera/camera).
#   KHÔNG cần camera_namespace:='' (ros2 launch không nhận giá trị rỗng -> lỗi malformed).

# (a) USB 3.0 (cáp SuperSpeed + cổng USB-C xanh) — chất lượng đầy đủ:
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    pointcloud.enable:=false \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30

# (b) USB 2.0 (cáp thường / camera báo "Frames didn't arrive") — hạ profile + tắt luồng thừa.
#     Kiểm tra USB2 hay USB3:  rs-enumerate-devices | grep -i "Usb Type"   (2.1 = USB2)
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true pointcloud.enable:=false \
    enable_infra1:=false enable_infra2:=false \
    enable_gyro:=false enable_accel:=false \
    rgb_camera.color_profile:=424x240x6 \
    depth_module.depth_profile:=480x270x6
#   424x240x6 = cực nhẹ cho USB2. Chạy êm rồi có thể nâng lên x15.
#   USB2 = ảnh nhỏ → khi calib T-Pose phải ĐỨNG GẦN camera hơn.
#   Nếu bạn chạy realsense với 1 namespace (camera_namespace:=/camera) thì:
#     ros2 launch custom_ur_human_tracking human_tracking.launch.py camera_prefix:=/camera
#   Kiểm tra ĐỦ 3 topic node cần (nếu thiếu → node không xử lý):
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/camera_info --once      # fx, fy, cx, cy
#   Nếu driver đặt topic khác tên → remap khi chạy human_tracking, ví dụ:
#     ros2 run custom_ur_human_tracking real_arm_node --ros-args \
#         -r /camera/color/image_raw:=/camera/camera/color/image_raw

# --- T-cam — NHẬN DIỆN + BRIDGE (real_arm_node + human_obstacle_bridge) ---
    ros2 launch custom_ur_human_tracking human_tracking.launch.py
#   Chỉ chạy nhận diện, TẮT bridge (không đẩy obstacle vào scene):
ros2 launch custom_ur_human_tracking human_tracking.launch.py obstacle_bridge:=false
#   Chạy lẻ từng node:
ros2 run custom_ur_human_tracking real_arm_node
ros2 run custom_ur_human_tracking human_obstacle_bridge

# --- HIỆU CHUẨN (CALIB) ---
#   1. Đặt mã AprilTag (36h11, cạnh 9cm) trong khung hình — trục sẽ hiện trên cửa sổ.
#   2. Đứng dang 2 tay ngang vai (T-POSE), giữ yên ~50 frame tới khi báo "HIỆU CHUẨN XONG".
#   3. Sau đó cửa sổ chuyển "TRACKING" → cẳng tay được publish.
#      (Giữ lại T-Pose 2s sau 5s cooldown để calib lại nếu cần.)

# --- KIỂM TRA NHẬN DIỆN CHẠY ---
ros2 topic hz  /human_skeleton                         # ~15Hz khi đang tracking (elbow, wrist @ frame world)
ros2 topic echo /human_skeleton --once                 # 2 pose: [elbow, wrist]
ros2 run rqt_image_view rqt_image_view /camera/color/image_annotated   # xem ảnh HUD + skeleton
ros2 topic echo /camera_debug_data --once              # [tvec, rpy, visibility x3, fps]

# --- KIỂM TRA BRIDGE ĐẨY OBSTACLE VÀO SCENE ---
ros2 topic echo /planning_scene --once | grep -A2 human_forearm    # 5 sphere REMOVE+ADD
ros2 topic echo /monitored_planning_scene --once | grep human_forearm   # move_group đã merge

# --- TINH CHỈNH BRIDGE (số sphere / bán kính an toàn / timeout) ---
ros2 param set /human_obstacle_bridge num_spheres 5       # số sphere dọc cẳng tay
ros2 param set /human_obstacle_bridge radius 0.06         # bán kính đệm an toàn [m] (mặc định 0.035)
ros2 param set /human_obstacle_bridge stale_timeout 0.5   # mất tracking quá lâu → gỡ obstacle [s]
#   LƯU Ý: đổi param lúc chạy chỉ áp dụng cho lần publish kế tiếp; radius nhỏ = biên an toàn mỏng.

# --- THỨ TỰ CHẠY ĐẦY ĐỦ (né tay người thật) ---
#   T1: bringup real (UR + move_group + rviz)
#   T2: moveit servo real  + start_servo
#   T3: aapf_supervisor_real  + /aapf_supervisor/start
#   T0: realsense driver (align_depth)
#   T-cam: human_tracking.launch.py  → calib T-Pose → robot né cẳng tay
ros2 topic echo /aapf_supervisor/mode_text             # phải nhảy FOLLOW → AAPF_AVOID khi tay lại gần

# ============================================================
# LOGGER CSV  (custom_ur_logger) — gom camera + AAPF vào 1 file
#   34 cột: Step,Time,State,Cam_State,Loop_Delta,Min_Dist,Robot_XYZ(EE),
#           J1-6_pos(góc rad),J1-6_vel,AprilTag_XYZ,Cam_FPS,
#           CamSh_XYZ,CamEl_XYZ,CamWr_XYZ (điểm cam world)
#   Nguồn: /aapf_supervisor/log_debug, /real_arm_node/track_state,
#          /real_arm_node/joints_world, /joint_states, /arm_joint_velocities, /camera_debug_data
#   -> PHẢI rebuild custom_ur_aapf_2 + custom_ur_human_tracking rồi source lại.
# ============================================================
colcon build --packages-select custom_ur_aapf_2 custom_ur_human_tracking custom_ur_logger
source install/setup.bash

# --- chạy node logger (sau khi AAPF + camera đã chạy) ---
ros2 launch custom_ur_logger logger.launch.py                 # mặc định ~/aapf_logs, 50Hz
#   hoặc chỉnh:
ros2 launch custom_ur_logger logger.launch.py output_dir:=/home/dat-ubuntu2204/logs rate_hz:=50.0
#   hoặc chạy lẻ:
ros2 run custom_ur_logger human_aapf_logger --ros-args -p output_dir:=/tmp/logs -p rate_hz:=50.0

# --- BẬT / TẮT ghi (file lưu streaming, mỗi lần start = 1 file mới) ---
ros2 service call /human_aapf_logger/start std_srvs/srv/Trigger {}   # -> ~/aapf_logs/human_aapf_<time>.csv
ros2 service call /human_aapf_logger/stop  std_srvs/srv/Trigger {}

# --- kiểm tra nguồn dữ liệu có đủ không (ô trống = nguồn đó chưa publish) ---
ros2 topic echo /aapf_supervisor/log_debug --once   # [v_att3,v_rep3,v_cmd3,ee3] — cần supervisor chạy
ros2 topic echo /real_arm_node/track_state --once   # Cam_State — cần real_arm_node chạy
ros2 topic hz   /arm_joint_velocities               # J1-6 — cần joint_velocity_estimator (trong launch AAPF)

---

# 0. Rebuild (vì đã sửa C++ AAPF + camera) rồi source lại
colcon build --packages-select custom_ur_aapf_2 custom_ur_human_tracking custom_ur_logger
source install/setup.bash

# 1. Chạy node logger (sau khi AAPF + camera đã chạy)
ros2 launch custom_ur_logger logger.launch.py            # ~/aapf_logs, 50Hz

# 2. Bật ghi → tạo file mới human_aapf_<timestamp>.csv
ros2 service call /human_aapf_logger/start std_srvs/srv/Trigger {}

# 3. Tắt ghi
ros2 service call /human_aapf_logger/stop std_srvs/srv/Trigger {}

---
ros2 launch custom_ur_simulation bringup_real_camera_gripper.launch.py robot_ip:=192.168.1.1
ros2 launch custom_ur_servo spawn_moveit_servo_real.launch.py
ros2 launch custom_ur_aapf_2 aapf_supervisor_real.launch.py
ros2 service call /aapf_supervisor/start std_srvs/srv/Trigger {}


ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    pointcloud.enable:=false \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30


ros2 launch custom_ur_human_tracking human_tracking.launch.py

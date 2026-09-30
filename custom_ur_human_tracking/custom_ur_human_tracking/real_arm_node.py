#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import cv2
import numpy as np
import mediapipe as mp
# mediapipe>=0.10.30 (bản numpy-2-native) đã BỎ API mp.solutions.pose. Dùng
# Tasks API (PoseLandmarker) thay thế — chỉ số landmark giữ nguyên (33 điểm).
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    PoseLandmarker, PoseLandmarkerOptions, RunningMode,
)
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
import os

# ==============================================================================
# THÔNG SỐ HIỆU CHUẨN APRILTAG (ĐÓNG VAI TRÒ LÀ GỐC TỌA ĐỘ TRUNG GIAN)
# ==============================================================================
MARKER_SIZE = 0.09  # Kích thước mã 9cm

# --- BỘ LỌC CHỐNG NHIỄU THỜI GIAN VÀ KHÔNG GIAN ---
ALPHA_EMA = 0.4
DEAD_ZONE = 0.005  # Vùng chết vi mô 5mm (0.005m) chống rung RViz

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__)) if '__file__' in globals() else os.getcwd()


def quaternion_about_axis(angle, axis):
    """ Thay thế tf.transformations.quaternion_about_axis (ROS1) — trả về [x, y, z, w]. """
    axis = np.array(axis[:3], dtype=np.float64)
    n = np.linalg.norm(axis)
    q = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    if n > 1e-8:
        q[:3] = axis / n * np.sin(angle / 2.0)
    q[3] = np.cos(angle / 2.0)
    return q


class RealArmNode(Node):
    def __init__(self):
        super().__init__('real_arm_node')

        self.latest_depth_cv = None

        # --- CHIẾN THUẬT RATE LIMITING ---
        self.target_fps = 15.0
        self.process_interval = 1.0 / self.target_fps
        self.last_process_time = self.get_time()

        self.last_fps_time = self.get_time()
        self.frame_count = 0
        self.current_fps = 0.0

        # --- MÁY TRẠNG THÁI & AUTO-CALIBRATION (50 KHUNG HÌNH) ---
        self.state = "WAITING_FOR_T_POSE"
        self.t_pose_start_time = 0.0

        # BIẾN MỚI BỔ SUNG ĐỂ SỬA LỖI UX
        self.t_pose_lost_time = 0.0     # Thời gian ân xá khi lỡ nháy frame
        self.last_calib_time = 0.0      # Thời gian hồi chiêu (Cooldown) chống lặp

        # Biến đếm và lưu trữ kỷ lục 50 frame
        self.calib_frames = 0
        self.calib_rvecs = []
        self.calib_tvecs = []
        self.calib_biceps = []
        self.calib_forearms = []

        self.calibrated_bicep_len = 0.0
        self.calibrated_forearm_len = 0.0

        # --- TOÁN HỌC MA TRẬN 3D ---
        self.current_rvec = None
        self.current_tvec = None
        self.locked_T_m_c = None  # Ma trận Camera -> Marker đã chốt

        # MA TRẬN TĨNH: Marker -> World Gazebo (Dựa trên setup thực tế)
        # Tọa độ: X=0.75, Y=0.1, Z=0.56
        # Trục Z mã (nhìn ra) -> Trục Y Gazebo
        # Trục Y mã (hướng xuống) -> Trục -Z Gazebo
        # Trục X mã (sang phải) -> Trục X Gazebo
        self.T_w_m = np.eye(4, dtype=np.float64)
        self.T_w_m[:3, :3] = np.array([
            [  0,  0,  1],
            [ -1,  0,  0],
            [  0, -1,  0]
        ])
        self.T_w_m[:3, 3] = [0.0, 0.0, 0.5]

        self.camera_matrix = np.eye(3, dtype=np.float64)
        self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

        # ----------------------------------------------------------------------
        # KHỞI TẠO BỘ QUÉT MÃ APRILTAG
        # ----------------------------------------------------------------------
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.cv2_new_api = True
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.cv2_new_api = False

        half_s = MARKER_SIZE / 2.0
        self.marker_obj_points = np.array([
            [-half_s,  half_s, 0],
            [ half_s,  half_s, 0],
            [ half_s, -half_s, 0],
            [-half_s, -half_s, 0]
        ], dtype=np.float32)

        # --- BỘ LỌC VÀ CACHE HIỂN THỊ ---
        self.last_target_points = None  # Dùng cho Dead-zone
        self.smoothed_joints_world = {11: None, 13: None, 15: None}

        self.display_cache = {
            'pixels': None,
            'cam_coords': None,
            'marker_coords': None,
            't_pose_debug': None
        }

        # --- MEDIAPIPE TASKS: PoseLandmarker (IMAGE mode) ---
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value or self._resolve_model_path()
        pose_opts = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = PoseLandmarker.create_from_options(pose_opts)
        self.get_logger().info(f"[RealArm] PoseLandmarker model: {model_path}")

        self.fx, self.fy, self.cx, self.cy = 385.0, 385.0, 320.0, 240.0
        self.camera_info_received = False

        self.skeleton_pub = self.create_publisher(PoseArray, '/human_skeleton', 1)
        # 3 điểm world (vai, khuỷu, cổ tay) cho logger — KHÁC /human_skeleton (chỉ elbow+wrist).
        self.joints_world_pub = self.create_publisher(PoseArray, '/real_arm_node/joints_world', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/ghost_arm_visual', 1)
        self.debug_img_pub = self.create_publisher(Image, '/camera/color/image_annotated', 1)
        self.cam_data_pub = self.create_publisher(Float64MultiArray, '/camera_debug_data', 1)
        # FSM state (WAITING/HOLDING/TRACKING) cho logger (Cam_State). Timer 10Hz
        # publish đều kể cả khi không có frame camera.
        self.state_pub = self.create_publisher(String, '/real_arm_node/track_state', 1)
        self.create_timer(0.1, lambda: self.state_pub.publish(String(data=self.state)))

        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_cb, 1)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 1)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 1)

        # --- CỬA SỔ HIỂN THỊ (phóng to cho dễ nhìn, nhất là USB2 ảnh nhỏ) ---
        self.declare_parameter('display_scale', 2.0)   # hệ số phóng to cửa sổ (1.0 = gốc)
        self.display_scale = float(self.get_parameter('display_scale').value)
        self.window_name = "HRI - World & AprilTag System"
        # WINDOW_NORMAL: kéo giãn cửa sổ tuỳ ý bằng chuột (ngoài display_scale).
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f"[RealArm] NODE ĐÃ KHỞI ĐỘNG! Hệ thống chờ T-Pose... (display_scale={self.display_scale})")

    def get_time(self):
        """ Thay thế rospy.get_time() — trả về thời gian hệ thống (giây, float). """
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------------------
    # THAY CV_BRIDGE: chuyển đổi Image <-> numpy thủ công.
    # cv_bridge của ROS Humble biên dịch theo numpy 1.x -> segfault trên numpy 2.
    # Node này chạy numpy 2 (cùng stack với mediapipe 0.10.35), nên tự convert.
    # -------------------------------------------------------------------------
    @staticmethod
    def imgmsg_to_bgr(msg):
        """sensor_msgs/Image (rgb8|bgr8) -> HxWx3 uint8 BGR."""
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)[:, :w * 3]
        arr = arr.reshape(h, w, 3)
        if msg.encoding == 'rgb8':
            arr = arr[:, :, ::-1]  # RGB -> BGR
        return np.ascontiguousarray(arr)

    @staticmethod
    def depthmsg_to_cv(msg):
        """sensor_msgs/Image depth (16UC1|mono16|32FC1) -> HxW."""
        dt = np.float32 if msg.encoding == '32FC1' else np.uint16
        itemsize = np.dtype(dt).itemsize
        arr = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.step // itemsize)
        return np.ascontiguousarray(arr[:, :msg.width])

    def bgr_to_imgmsg(self, img):
        """HxWx3 uint8 BGR -> sensor_msgs/Image (bgr8)."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(img.shape[1] * 3)
        msg.data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
        return msg

    def _resolve_model_path(self):
        """Tìm file model PoseLandmarker (.task) trong share hoặc cây nguồn."""
        fname = 'pose_landmarker_full.task'
        candidates = []
        try:
            candidates.append(os.path.join(
                get_package_share_directory('custom_ur_human_tracking'), 'models', fname))
        except Exception:
            pass
        candidates.append(os.path.join(SCRIPT_DIR, '..', 'models', fname))  # cây nguồn (dev)
        candidates.append(os.path.join(SCRIPT_DIR, 'models', fname))
        for c in candidates:
            if os.path.exists(c):
                return os.path.abspath(c)
        raise FileNotFoundError(
            f"Không tìm thấy model '{fname}'. Đã thử: {candidates}. Tải bằng: "
            f"curl -o {candidates[-1]} https://storage.googleapis.com/mediapipe-models/"
            f"pose_landmarker/pose_landmarker_full/float16/latest/{fname}")

    def cleanup(self):
        try:
            self.landmarker.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        self.get_logger().info("[RealArm] Đã đóng cửa sổ Camera.")

    def camera_info_cb(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

            self.camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ], dtype=np.float64)

            if len(msg.d) > 0:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64)

            self.camera_info_received = True

    def depth_callback(self, msg):
        try:
            self.latest_depth_cv = self.depthmsg_to_cv(msg)
        except Exception:
            pass

    def get_filtered_depth(self, x, y, window_size=11):
        if self.latest_depth_cv is None:
            return 0.0
        h, w = self.latest_depth_cv.shape
        half_w = window_size // 2

        x_min, x_max = max(0, x - half_w), min(w - 1, x + half_w)
        y_min, y_max = max(0, y - half_w), min(h - 1, y + half_w)
        roi = self.latest_depth_cv[y_min:y_max+1, x_min:x_max+1]

        valid_depths = roi[roi > 0]
        if len(valid_depths) == 0:
            return 0.0

        min_depth = np.min(valid_depths)
        foreground = valid_depths[valid_depths <= min_depth + 150]

        return np.median(foreground) / 1000.0

    def deproject_pixel_to_point(self, u, v, z_meters):
        if z_meters <= 0.001:
            return None
        x = (u - self.cx) * z_meters / self.fx
        y = (v - self.cy) * z_meters / self.fy
        return np.array([x, y, z_meters])

    # -------------------------------------------------------------------------
    # CHUỖI ĐỘNG HỌC MỚI (KINEMATIC CHAIN): Camera -> Marker -> Gazebo World
    # -------------------------------------------------------------------------
    def transform_camera_to_world(self, cam_point, T_m_c):
        """ Chuyển điểm từ Camera sang Gazebo thông qua ma trận tĩnh của AprilTag """
        pt_4d_cam = np.array([cam_point[0], cam_point[1], cam_point[2], 1.0])
        pt_marker = np.dot(T_m_c, pt_4d_cam)
        pt_world = np.dot(self.T_w_m, pt_marker)
        return pt_world[:3]

    def calculate_marker_transform(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T_c_m = np.eye(4, dtype=np.float64)
        T_c_m[:3, :3] = R
        T_c_m[:3, 3] = tvec.flatten()
        T_m_c = np.linalg.inv(T_c_m)
        return T_m_c

    def transform_point_matrix(self, pt_3d_camera, T_matrix):
        pt_4d = np.array([pt_3d_camera[0], pt_3d_camera[1], pt_3d_camera[2], 1.0])
        pt_transformed = np.dot(T_matrix, pt_4d)
        return pt_transformed[:3]

    def check_t_pose(self, lms):
        l_sh, l_el, l_wr = lms[11], lms[13], lms[15]
        r_sh, r_el, r_wr = lms[12], lms[14], lms[16]

        if any(pt.visibility < 0.5 for pt in [l_sh, l_el, l_wr, r_sh, r_el, r_wr]):
            return False, {"ratio": 0, "straight_l": 0, "straight_r": 0, "valid_x": False}

        def dist2d(p1, p2):
            return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

        sh_width = dist2d(l_sh, r_sh)
        wr_width = dist2d(l_wr, r_wr)
        ratio = wr_width / sh_width if sh_width > 0 else 0

        def straightness(sh, el, wr):
            total = dist2d(sh, el) + dist2d(el, wr)
            return dist2d(sh, wr) / total if total > 0 else 0

        straight_l = straightness(l_sh, l_el, l_wr)
        straight_r = straightness(r_sh, r_el, r_wr)

        valid_x = (r_wr.x < r_el.x < r_sh.x) and (l_sh.x < l_el.x < l_wr.x)

        is_valid = (ratio > 2.0) and (straight_l > 0.85) and (straight_r > 0.85) and valid_x
        return is_valid, {"ratio": ratio, "straight_l": straight_l, "straight_r": straight_r, "valid_x": valid_x}

    def normalize_vector(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm

    # =========================================================================
    # VÒNG LẶP CHÍNH
    # =========================================================================
    def color_callback(self, msg):
        current_time = self.get_time()

        try:
            self.frame_count += 1
            if current_time - self.last_fps_time >= 1.0:
                self.current_fps = self.frame_count / (current_time - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = current_time

            cv_color = self.imgmsg_to_bgr(msg)
            h, w, _ = cv_color.shape

            # --- QUÉT MÃ APRILTAG ---
            marker_detected_now = False
            if self.state != "CALIBRATED_AND_TRACKING" or self.t_pose_start_time != 0:
                gray_img = cv2.cvtColor(cv_color, cv2.COLOR_BGR2GRAY)

                if self.cv2_new_api:
                    corners, ids, rejected = self.aruco_detector.detectMarkers(gray_img)
                else:
                    corners, ids, rejected = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_params)

                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(cv_color, corners, ids)
                    success, rvec, tvec = cv2.solvePnP(self.marker_obj_points, corners[0][0], self.camera_matrix, self.dist_coeffs)

                    if success:
                        try:
                            cv2.drawFrameAxes(cv_color, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        except AttributeError:
                            cv2.aruco.drawAxis(cv_color, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

                        self.current_rvec = rvec
                        self.current_tvec = tvec
                        marker_detected_now = True

            status_text = "WAITING / NO ARM"
            hud_color = (0, 0, 255)

            # --- XỬ LÝ MEDIA PIPE ---
            if current_time - self.last_process_time >= self.process_interval:
                self.last_process_time = current_time
                img_rgb = cv2.cvtColor(cv_color, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                                    data=np.ascontiguousarray(img_rgb))
                results = self.landmarker.detect(mp_image)
                is_tracking_success = False
                t_pose_debug = None

                # Tasks API: pose_landmarks là list (1 phần tử/người). Lấy người đầu.
                if results.pose_landmarks:
                    lms = results.pose_landmarks[0]

                    p11 = (int(lms[11].x * w), int(lms[11].y * h))
                    p13 = (int(lms[13].x * w), int(lms[13].y * h))
                    p15 = (int(lms[15].x * w), int(lms[15].y * h))
                    pixel_points = [p11, p13, p15]

                    raw_3d_cam_points = []
                    all_points_valid = True

                    for (u, v) in pixel_points:
                        u, v = np.clip(u, 0, w - 1), np.clip(v, 0, h - 1)
                        z_meters = self.get_filtered_depth(u, v, window_size=11)
                        cam_pt = self.deproject_pixel_to_point(u, v, z_meters)

                        if cam_pt is not None:
                            raw_3d_cam_points.append(cam_pt)
                        else:
                            all_points_valid = False
                            break

                    if all_points_valid and len(raw_3d_cam_points) == 3:
                        sh_cam, el_cam, wr_cam = raw_3d_cam_points
                        t_pose_detected, t_pose_debug = self.check_t_pose(lms)

                        # BƯỚC 1: CHỜ T-POSE
                        if self.state == "WAITING_FOR_T_POSE":
                            status_text = "DANG 2 TAY NGANG VAI (T-POSE) DE CALIB!"
                            hud_color = (0, 255, 255)

                            if t_pose_detected:
                                self.state = "HOLDING_T_POSE"
                                self.calib_frames = 0
                                self.t_pose_lost_time = 0.0  # Reset thời gian ân xá
                                self.calib_rvecs.clear()
                                self.calib_tvecs.clear()
                                self.calib_biceps.clear()
                                self.calib_forearms.clear()

                        # BƯỚC 2: CALIB 50 KHUNG HÌNH (CÓ THỜI GIAN ÂN XÁ 1.5s)
                        elif self.state == "HOLDING_T_POSE":
                            if t_pose_detected and marker_detected_now:
                                # Điều kiện hoàn hảo -> Tiếp tục thu thập
                                self.t_pose_lost_time = 0.0  # Reset bộ đếm ân xá
                                self.calib_frames += 1
                                self.calib_rvecs.append(self.current_rvec.copy())
                                self.calib_tvecs.append(self.current_tvec.copy())
                                self.calib_biceps.append(np.linalg.norm(el_cam - sh_cam))
                                self.calib_forearms.append(np.linalg.norm(wr_cam - el_cam))

                                status_text = f"DANG THU THAP... {self.calib_frames}/50"
                                hud_color = (0, 255, 255)

                                if self.calib_frames >= 50:
                                    # CHỐT TRUNG BÌNH CỘNG
                                    avg_rvec = np.mean(self.calib_rvecs, axis=0)
                                    avg_tvec = np.mean(self.calib_tvecs, axis=0)
                                    self.locked_T_m_c = self.calculate_marker_transform(avg_rvec, avg_tvec)

                                    self.calibrated_bicep_len = np.clip(np.mean(self.calib_biceps), 0.15, 0.40)
                                    self.calibrated_forearm_len = np.clip(np.mean(self.calib_forearms), 0.15, 0.40)

                                    self.state = "CALIBRATED_AND_TRACKING"
                                    self.last_calib_time = current_time  # Bắt đầu đếm thời gian Cooldown
                                    self.get_logger().info(f"HIỆU CHUẨN XONG BẰNG 50 FRAME TRUNG BÌNH!")
                            else:
                                # Nháy frame / Rớt tay -> Kích hoạt thời gian ân xá
                                if self.t_pose_lost_time == 0.0:
                                    self.t_pose_lost_time = current_time

                                if current_time - self.t_pose_lost_time > 1.5:
                                    # Mất dáng tay quá 1.5s -> Hủy bỏ, bắt đầu lại từ đầu
                                    self.state = "WAITING_FOR_T_POSE"
                                    status_text = "[!] Mat dang T-Pose hoac AprilTag. Vui long lam lai!"
                                    hud_color = (0, 0, 255)
                                else:
                                    # Đang trong 1.5s ân xá -> Tạm dừng đếm, giữ nguyên thành quả
                                    status_text = f"GIU YEN... (Tam dung {self.calib_frames}/50)"
                                    hud_color = (0, 165, 255)  # Màu cam

                        # BƯỚC 3: TRACKING & ĐIỀU KHIỂN RVIZ
                        if self.state == "CALIBRATED_AND_TRACKING":
                            status_text = f"TRACKING (World & AprilTag)"
                            hud_color = (0, 255, 0)

                            # Kiểm tra Cooldown: Phải qua 5s kể từ lần Calib trước mới cho phép Calib lại
                            if t_pose_detected and (current_time - self.last_calib_time > 5.0):
                                if self.t_pose_start_time == 0:
                                    self.t_pose_start_time = current_time
                                elif current_time - self.t_pose_start_time >= 2.0 and marker_detected_now:
                                    # Giữ T-Pose 2s sau Cooldown -> Reset Calib
                                    self.state = "HOLDING_T_POSE"
                                    self.calib_frames = 0
                                    self.t_pose_lost_time = 0.0
                                    self.calib_rvecs.clear()
                                    self.calib_tvecs.clear()
                                    self.calib_biceps.clear()
                                    self.calib_forearms.clear()
                            else:
                                self.t_pose_start_time = 0

                            # ---> XỬ LÝ LUỒNG ĐIỀU KHIỂN RVIZ VỚI MA TRẬN 3D MỚI <---
                            final_sh_cam = sh_cam
                            v_bicep_cam = self.normalize_vector(el_cam - final_sh_cam)
                            final_el_cam = final_sh_cam + v_bicep_cam * self.calibrated_bicep_len
                            v_forearm_cam = self.normalize_vector(wr_cam - final_el_cam)
                            final_wr_cam = final_el_cam + v_forearm_cam * self.calibrated_forearm_len

                            # Chuyển qua Gazebo World bằng chuỗi Ma trận T_{W->M} x T_{M->C}
                            sh_world = self.transform_camera_to_world(final_sh_cam, self.locked_T_m_c)
                            el_world = self.transform_camera_to_world(final_el_cam, self.locked_T_m_c)
                            wr_world = self.transform_camera_to_world(final_wr_cam, self.locked_T_m_c)
                            target_world_points = [sh_world, el_world, wr_world]

                            # === BỘ LỌC VÙNG CHẾT VI MÔ (MICRO DEAD-ZONE) ===
                            if self.last_target_points is None:
                                self.last_target_points = target_world_points
                            else:
                                for i in range(3):
                                    # Nếu khoảng cách nhúc nhích < 5mm, BỎ QUA để chống giật RViz
                                    if np.linalg.norm(target_world_points[i] - self.last_target_points[i]) < DEAD_ZONE:
                                        target_world_points[i] = self.last_target_points[i]
                                    else:
                                        self.last_target_points[i] = target_world_points[i]

                            # Áp dụng EMA Filter sau khi đã qua Vùng Chết
                            smoothed_world_points = []
                            for i, idx in enumerate([11, 13, 15]):
                                if self.smoothed_joints_world[idx] is None:
                                    self.smoothed_joints_world[idx] = target_world_points[i]
                                else:
                                    self.smoothed_joints_world[idx] = ALPHA_EMA * target_world_points[i] + (1.0 - ALPHA_EMA) * self.smoothed_joints_world[idx]
                                smoothed_world_points.append(self.smoothed_joints_world[idx])

                            rviz_shoulder, rviz_elbow, rviz_wrist = smoothed_world_points

                            # Gửi lên RViz (ĐÃ RÚT GỌN CHỈ CÒN CẲNG TAY)
                            self.publish_skeleton(rviz_elbow, rviz_wrist)
                            self.publish_visual_markers(rviz_elbow, rviz_wrist)
                            # 3 điểm world (vai/khuỷu/cổ tay) cho logger
                            self.publish_joints_world(rviz_shoulder, rviz_elbow, rviz_wrist)

                            # ---> XỬ LÝ LUỒNG GIAO DIỆN (DASHBOARD) <---
                            marker_coords = [
                                self.transform_point_matrix(final_sh_cam, self.locked_T_m_c),
                                self.transform_point_matrix(final_el_cam, self.locked_T_m_c),
                                self.transform_point_matrix(final_wr_cam, self.locked_T_m_c)
                            ]

                            # PUBLISH DỮ LIỆU APRILTAG VÀ MEDIAPIPE CHO LOG
                            if self.current_tvec is not None and self.current_rvec is not None:
                                msg_data = Float64MultiArray()
                                roll, pitch, yaw = 0.0, 0.0, 0.0
                                try:
                                    rmat, _ = cv2.Rodrigues(self.current_rvec)
                                    proj_matrix = np.hstack((rmat, self.current_tvec))
                                    euler_angles = cv2.decomposeProjectionMatrix(proj_matrix)[6]
                                    pitch, yaw, roll = euler_angles.flatten()
                                except:
                                    pass

                                msg_data.data = [
                                    self.current_tvec[0][0], self.current_tvec[1][0], self.current_tvec[2][0],
                                    roll, pitch, yaw,
                                    lms[11].visibility, lms[13].visibility, lms[15].visibility,
                                    self.current_fps
                                ]
                                self.cam_data_pub.publish(msg_data)

                            is_tracking_success = True
                            self.display_cache = {
                                'pixels': pixel_points,
                                'cam_coords': [final_sh_cam, final_el_cam, final_wr_cam],
                                'marker_coords': marker_coords,
                                't_pose_debug': t_pose_debug
                            }

                if not is_tracking_success:
                    self.smoothed_joints_world = {11: None, 13: None, 15: None}
                    self.last_target_points = None
                    self.display_cache = {
                        'pixels': None, 'cam_coords': None, 'marker_coords': None,
                        't_pose_debug': t_pose_debug if 't_pose_debug' in locals() else None
                    }

            # -------------------------------------------------------------
            # VẼ ĐỒ HỌA TRÊN ẢNH CAMERA (GÓC NHÌN CAMERA)
            # -------------------------------------------------------------
            if self.display_cache['pixels'] is not None and self.state == "CALIBRATED_AND_TRACKING":
                p11, p13, p15 = self.display_cache['pixels']
                cv2.line(cv_color, p11, p13, (0, 255, 0), 4)
                cv2.line(cv_color, p13, p15, (0, 255, 0), 4)
                for pt in [p11, p13, p15]:
                    cv2.circle(cv_color, pt, 6, (0, 0, 255), -1)

                c_sh, c_el, c_wr = self.display_cache['cam_coords']
                cv2.putText(cv_color, f"Cam Sh: X={c_sh[0]:.3f} Y={c_sh[1]:.3f} Z={c_sh[2]:.3f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(cv_color, f"Cam El: X={c_el[0]:.3f} Y={c_el[1]:.3f} Z={c_el[2]:.3f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(cv_color, f"Cam Wr: X={c_wr[0]:.3f} Y={c_wr[1]:.3f} Z={c_wr[2]:.3f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                hud_color = (0, 255, 0)
            else:
                if self.state == "CALIBRATED_AND_TRACKING":
                    status_text = "[!] Mat dau canh tay hoac Depth nhieu"
                    hud_color = (0, 0, 255)

            # --- BẢNG VISUAL DEBUGGING T-POSE ---
            if self.state in ["WAITING_FOR_T_POSE", "HOLDING_T_POSE"] and self.display_cache.get('t_pose_debug') is not None:
                debug = self.display_cache['t_pose_debug']
                ratio = debug.get('ratio', 0)
                sl = debug.get('straight_l', 0)
                sr = debug.get('straight_r', 0)
                valid_x = debug.get('valid_x', False)

                info_w = 230
                cv2.putText(cv_color, "T-POSE DEBUG:", (w - info_w, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                c_ratio = (0, 255, 0) if ratio > 2.0 else (0, 0, 255)
                cv2.putText(cv_color, f"S.Tay/Vai: {ratio:.1f}/2.0", (w - info_w, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_ratio, 2)

                c_sl = (0, 255, 0) if sl > 0.85 else (0, 0, 255)
                cv2.putText(cv_color, f"Thang Trai: {sl*100:.0f}%", (w - info_w, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_sl, 2)

                c_sr = (0, 255, 0) if sr > 0.85 else (0, 0, 255)
                cv2.putText(cv_color, f"Thang Phai: {sr*100:.0f}%", (w - info_w, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_sr, 2)

                c_vx = (0, 255, 0) if valid_x else (0, 0, 255)
                cv2.putText(cv_color, f"Huong X: {'OK' if valid_x else 'SAI'}", (w - info_w, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_vx, 2)

            self.update_hud_and_show(cv_color, status_text, hud_color)

        except Exception as e:
            self.get_logger().warn(f"[RealArm] color_callback lỗi: {e}", throttle_duration_sec=2.0)

    def update_hud_and_show(self, cv_image, status_text, text_color):
        h, w, _ = cv_image.shape
        dash_h = 180

        dashboard = np.zeros((dash_h, w, 3), dtype=np.uint8)

        cv2.putText(cv_image, f"Cam FPS: {self.current_fps:.1f} | AI: {self.target_fps} Hz", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, f"STATUS: {status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

        cv2.line(dashboard, (0, 0), (w, 0), (255, 255, 255), 3)
        cv2.putText(dashboard, "GOC TOA DO: TAY NGUOI SO VOI MA APRILTAG", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.line(dashboard, (10, 40), (w - 10, 40), (100, 100, 100), 1)

        if self.display_cache['marker_coords'] is not None and self.state == "CALIBRATED_AND_TRACKING":
            m_sh, m_el, m_wr = self.display_cache['marker_coords']

            font_scale = 0.45
            thickness = 2

            cv2.putText(dashboard, "VAI  (Sh):", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 150, 255), thickness)
            cv2.putText(dashboard, f"X: {m_sh[0]:>6.3f}   Y: {m_sh[1]:>6.3f}   Z: {m_sh[2]:>6.3f}", (150, 80), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)

            cv2.putText(dashboard, "CHO  (El):", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 150, 255), thickness)
            cv2.putText(dashboard, f"X: {m_el[0]:>6.3f}   Y: {m_el[1]:>6.3f}   Z: {m_el[2]:>6.3f}", (150, 120), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)

            cv2.putText(dashboard, "TAY  (Wr):", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 150, 255), thickness)
            cv2.putText(dashboard, f"X: {m_wr[0]:>6.3f}   Y: {m_wr[1]:>6.3f}   Z: {m_wr[2]:>6.3f}", (150, 160), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)

            info_x = w - 200
            cv2.putText(dashboard, "CHIEU DAI:", (info_x, 80), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
            cv2.putText(dashboard, f"Bap: {self.calibrated_bicep_len:.3f}m", (info_x, 120), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
            cv2.putText(dashboard, f"Cang: {self.calibrated_forearm_len:.3f}m", (info_x, 160), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)

        else:
            cv2.putText(dashboard, "(Hay dang T-Pose truoc ong kinh de lay toa do)", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)

        combined_img = np.vstack((cv_image, dashboard))

        # Topic annotated giữ ĐỘ PHÂN GIẢI GỐC (rqt/RViz tự phóng được).
        debug_msg = self.bgr_to_imgmsg(combined_img)
        self.debug_img_pub.publish(debug_msg)

        # Chỉ phóng to cho CỬA SỔ hiển thị (USB2 ảnh nhỏ -> nhìn cho rõ).
        if self.display_scale != 1.0:
            show_img = cv2.resize(combined_img, None,
                                  fx=self.display_scale, fy=self.display_scale,
                                  interpolation=cv2.INTER_LINEAR)
        else:
            show_img = combined_img

        cv2.imshow(self.window_name, show_img)
        cv2.waitKey(1)

    # -------------------------------------------------------------------------
    # HÀM RÚT GỌN: CHỈ PUBLISH CẲNG TAY VÀ CỔ TAY ĐỂ ROBOT TRÁNH VA CHẠM
    # -------------------------------------------------------------------------
    def publish_skeleton(self, elbow, wrist):
        pose_array = PoseArray()
        pose_array.header.frame_id = "world"  # Gốc tọa độ Gazebo
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for pt in [elbow, wrist]:  # BỎ VAI ĐI ĐỂ TỐI ƯU
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(pt[0]), float(pt[1]), float(pt[2])
            p.orientation.w = 1.0
            pose_array.poses.append(p)
        self.skeleton_pub.publish(pose_array)

    def publish_joints_world(self, shoulder, elbow, wrist):
        """3 điểm vai/khuỷu/cổ tay trong world (thứ tự cố định) cho logger."""
        pose_array = PoseArray()
        pose_array.header.frame_id = "world"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for pt in [shoulder, elbow, wrist]:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(pt[0]), float(pt[1]), float(pt[2])
            p.orientation.w = 1.0
            pose_array.poses.append(p)
        self.joints_world_pub.publish(pose_array)

    def publish_visual_markers(self, elbow, wrist):
        m_array = MarkerArray()

        # LUÔN LUÔN VẼ TRỤC APRILTAG
        m_array.markers.extend(self.create_apriltag_axis_markers(100))

        # Chỉ vẽ cánh tay khi đã tracking
        if self.state == "CALIBRATED_AND_TRACKING":
            markers_forearm = self.create_capsule_markers(elbow, wrist, 0.035, (1.0, 0.3, 0.0, 0.8), 200)
            m_array.markers.extend(markers_forearm)

        self.marker_pub.publish(m_array)

    def create_apriltag_axis_markers(self, base_id):
        """ Tạo Marker trực tiếp bằng Ma trận Tĩnh T_w_m để mũi tên không bao giờ bị rung """
        markers = []

        # Gốc và các mũi tên X, Y, Z dài 15cm trong hệ Gazebo World
        orig_w = self.T_w_m[:3, 3]
        x_w = (self.T_w_m @ np.array([0.15, 0, 0, 1.0]))[:3]
        y_w = (self.T_w_m @ np.array([0, 0.15, 0, 1.0]))[:3]
        z_w = (self.T_w_m @ np.array([0, 0, 0.15, 1.0]))[:3]

        colors = [(1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)]
        endpoints = [x_w, y_w, z_w]

        for i in range(3):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = base_id + i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.points = [
                Point(x=float(orig_w[0]), y=float(orig_w[1]), z=float(orig_w[2])),
                Point(x=float(endpoints[i][0]), y=float(endpoints[i][1]), z=float(endpoints[i][2]))
            ]
            m.scale.x = 0.01  # Đường kính thân mũi tên
            m.scale.y = 0.025  # Đường kính đầu mũi tên
            m.scale.z = 0.03  # Độ dài đầu mũi tên
            m.color.r, m.color.g, m.color.b, m.color.a = colors[i]
            m.lifetime = Duration(seconds=0.1).to_msg()
            markers.append(m)

        return markers

    def create_capsule_markers(self, p1, p2, radius, rgba, base_id, show_sphere1=True, show_sphere2=True):
        markers = []
        center = (p1 + p2) / 2.0
        length = np.linalg.norm(p2 - p1)

        vec = (p2 - p1) / length if length > 0 else np.array([0, 0, 1])
        up = np.array([0.0, 0.0, 1.0])
        axis = np.cross(up, vec)
        norm_axis = np.linalg.norm(axis)

        if norm_axis < 1e-6:
            q = [0, 0, 0, 1]
            if np.dot(up, vec) < 0:
                q = [1, 0, 0, 0]
        else:
            axis = axis / norm_axis
            angle = np.arccos(np.clip(np.dot(up, vec), -1.0, 1.0))
            q = quaternion_about_axis(angle, axis)

        m_cyl = Marker()
        m_cyl.header.frame_id = "world"
        m_cyl.header.stamp = self.get_clock().now().to_msg()
        m_cyl.id = base_id
        m_cyl.type = Marker.CYLINDER
        m_cyl.action = Marker.ADD
        m_cyl.pose.position.x, m_cyl.pose.position.y, m_cyl.pose.position.z = float(center[0]), float(center[1]), float(center[2])
        m_cyl.pose.orientation.x, m_cyl.pose.orientation.y, m_cyl.pose.orientation.z, m_cyl.pose.orientation.w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        m_cyl.scale.x = m_cyl.scale.y = radius * 2
        m_cyl.scale.z = float(length)
        m_cyl.color.r, m_cyl.color.g, m_cyl.color.b, m_cyl.color.a = rgba
        m_cyl.lifetime = Duration(seconds=0.1).to_msg()
        markers.append(m_cyl)

        if show_sphere1:
            m_sph1 = Marker()
            m_sph1.header.frame_id = "world"
            m_sph1.header.stamp = self.get_clock().now().to_msg()
            m_sph1.id = base_id + 1
            m_sph1.type = Marker.SPHERE
            m_sph1.action = Marker.ADD
            m_sph1.pose.position.x, m_sph1.pose.position.y, m_sph1.pose.position.z = float(p1[0]), float(p1[1]), float(p1[2])
            m_sph1.pose.orientation.w = 1.0
            m_sph1.scale.x = m_sph1.scale.y = m_sph1.scale.z = radius * 2
            m_sph1.color.r, m_sph1.color.g, m_sph1.color.b, m_sph1.color.a = rgba
            m_sph1.lifetime = Duration(seconds=0.1).to_msg()
            markers.append(m_sph1)

        if show_sphere2:
            m_sph2 = Marker()
            m_sph2.header.frame_id = "world"
            m_sph2.header.stamp = self.get_clock().now().to_msg()
            m_sph2.id = base_id + 2
            m_sph2.type = Marker.SPHERE
            m_sph2.action = Marker.ADD
            m_sph2.pose.position.x, m_sph2.pose.position.y, m_sph2.pose.position.z = float(p2[0]), float(p2[1]), float(p2[2])
            m_sph2.pose.orientation.w = 1.0
            m_sph2.scale.x = m_sph2.scale.y = m_sph2.scale.z = radius * 2
            m_sph2.color.r, m_sph2.color.g, m_sph2.color.b, m_sph2.color.a = rgba
            m_sph2.lifetime = Duration(seconds=0.1).to_msg()
            markers.append(m_sph2)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = RealArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# human_aapf_logger — gom dữ liệu camera + AAPF vào 1 CSV (dùng generic_logger).
#
# Subscribe latest-cache mọi nguồn, timer 50Hz lấy mẫu -> 1 dòng CSV. Bật/tắt
# bằng service ~/start, ~/stop. Ghi streaming (an toàn kill -9).
#
# Nguồn cột:
#   State            <- /aapf_supervisor/mode_text (String)
#   Cam_State        <- /real_arm_node/track_state (String)
#   Min_Dist_m       <- /aapf_supervisor/obstacle_distance (Float64)
#   Robot_XYZ,V_*    <- /aapf_supervisor/log_debug (Float64MultiArray[12])
#   J1..J6_vel       <- /arm_joint_velocities (JointState)
#   AprilTag_*,MP_*,Cam_FPS <- /camera_debug_data (Float64MultiArray[10])

import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger

from custom_ur_logger.generic_logger import DataLogger

COLUMNS = [
    "Step_Number", "Time_ROS", "State", "Cam_State", "Loop_Delta_Time",
    "Min_Dist_m", "Robot_X", "Robot_Y", "Robot_Z",
    "J1_pos", "J2_pos", "J3_pos", "J4_pos", "J5_pos", "J6_pos",       # góc khớp [rad]
    "J1_vel", "J2_vel", "J3_vel", "J4_vel", "J5_vel", "J6_vel",       # vận tốc khớp [rad/s]
    "AprilTag_X", "AprilTag_Y", "AprilTag_Z", "Cam_FPS",
    "CamSh_X", "CamSh_Y", "CamSh_Z",                                  # điểm cam world
    "CamEl_X", "CamEl_Y", "CamEl_Z",
    "CamWr_X", "CamWr_Y", "CamWr_Z",
]

DEFAULT_ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class HumanAapfLogger(Node):
    def __init__(self):
        super().__init__('human_aapf_logger')

        self.declare_parameter('output_dir', os.path.expanduser('~/aapf_logs'))
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('arm_joints', DEFAULT_ARM_JOINTS)
        self.output_dir = self.get_parameter('output_dir').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.arm_joints = list(self.get_parameter('arm_joints').value)

        # --- Cache latest cho từng nguồn ---
        self.mode = None            # str
        self.cam_state = None       # str
        self.min_dist = None        # float
        self.log_debug = None       # list[12] = v_att3,v_rep3,v_cmd3,ee3 (chỉ dùng ee3)
        self.joint_vel = {}         # name -> vel (từ /arm_joint_velocities)
        self.joint_pos = {}         # name -> góc (từ /joint_states)
        self.cam = None             # list[10] = tvec3,rpy3,vis3,fps (chỉ dùng tvec3 + fps)
        self.cam_world = None       # list[9] = sh_xyz, el_xyz, wr_xyz (world)

        # --- Trạng thái ghi ---
        self.logger = None
        self.active = False
        self.step = 0
        self.last_ros_t = None

        # --- Subscriptions (latest-cache) ---
        self.create_subscription(String, '/aapf_supervisor/mode_text',
                                 lambda m: setattr(self, 'mode', m.data), 10)
        self.create_subscription(String, '/real_arm_node/track_state',
                                 lambda m: setattr(self, 'cam_state', m.data), 10)
        self.create_subscription(Float64, '/aapf_supervisor/obstacle_distance',
                                 lambda m: setattr(self, 'min_dist', m.data), 10)
        self.create_subscription(Float64MultiArray, '/aapf_supervisor/log_debug',
                                 lambda m: setattr(self, 'log_debug', list(m.data)), 10)
        self.create_subscription(Float64MultiArray, '/camera_debug_data',
                                 lambda m: setattr(self, 'cam', list(m.data)), 10)
        self.create_subscription(JointState, '/arm_joint_velocities', self._joint_vel_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_pos_cb, 10)
        self.create_subscription(PoseArray, '/real_arm_node/joints_world', self._cam_world_cb, 10)

        # --- Timer lấy mẫu ---
        self.create_timer(1.0 / self.rate_hz, self._sample_tick)

        # --- Services bật/tắt ---
        self.create_service(Trigger, '~/start', self._start_cb)
        self.create_service(Trigger, '~/stop', self._stop_cb)

        self.get_logger().info(
            f"[HumanAapfLogger] sẵn sàng. rate={self.rate_hz}Hz dir='{self.output_dir}'. "
            f"Gọi ~/start để bắt đầu ghi.")

    def _joint_vel_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                self.joint_vel[name] = msg.velocity[i]

    def _joint_pos_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_pos[name] = msg.position[i]

    def _cam_world_cb(self, msg: PoseArray):
        # PoseArray [shoulder, elbow, wrist] world -> flat [9]
        if len(msg.poses) >= 3:
            self.cam_world = [
                msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z,
                msg.poses[1].position.x, msg.poses[1].position.y, msg.poses[1].position.z,
                msg.poses[2].position.x, msg.poses[2].position.y, msg.poses[2].position.z,
            ]

    # ------------------------------------------------------------------ services
    def _start_cb(self, req, res):
        if self.active:
            res.success, res.message = False, "đang ghi rồi"
            return res
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.logger = DataLogger(prefix="human_aapf", output_dir=self.output_dir,
                                 columns=COLUMNS, timestamp=stamp)
        self.step = 0
        self.last_ros_t = None
        self.active = True
        res.success, res.message = True, f"bắt đầu ghi: {self.logger.path}"
        self.get_logger().info(f"[HumanAapfLogger] START -> {self.logger.path}")
        return res

    def _stop_cb(self, req, res):
        if not self.active:
            res.success, res.message = False, "chưa ghi"
            return res
        self.active = False
        path = self.logger.save()
        n = self.logger.rows
        self.logger = None
        res.success, res.message = True, f"đã lưu {n} dòng: {path}"
        self.get_logger().info(f"[HumanAapfLogger] STOP -> {path} ({n} dòng)")
        return res

    # -------------------------------------------------------------------- sample
    @staticmethod
    def _v(arr, i):
        """Lấy phần tử i của list cache, '' nếu thiếu."""
        return arr[i] if (arr is not None and i < len(arr)) else ""

    def _sample_tick(self):
        if not self.active or self.logger is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self.last_ros_t) if self.last_ros_t is not None else 0.0
        self.last_ros_t = now
        self.step += 1

        row = {
            "Step_Number": self.step,
            "Time_ROS": round(now, 6),
            "State": self.mode if self.mode is not None else "",
            "Cam_State": self.cam_state if self.cam_state is not None else "",
            "Loop_Delta_Time": round(dt, 6),
            "Min_Dist_m": self.min_dist if self.min_dist is not None else "",
            "Robot_X": self._v(self.log_debug, 9),
            "Robot_Y": self._v(self.log_debug, 10),
            "Robot_Z": self._v(self.log_debug, 11),
            "AprilTag_X": self._v(self.cam, 0),
            "AprilTag_Y": self._v(self.cam, 1),
            "AprilTag_Z": self._v(self.cam, 2),
            "Cam_FPS": self._v(self.cam, 9),
            "CamSh_X": self._v(self.cam_world, 0),
            "CamSh_Y": self._v(self.cam_world, 1),
            "CamSh_Z": self._v(self.cam_world, 2),
            "CamEl_X": self._v(self.cam_world, 3),
            "CamEl_Y": self._v(self.cam_world, 4),
            "CamEl_Z": self._v(self.cam_world, 5),
            "CamWr_X": self._v(self.cam_world, 6),
            "CamWr_Y": self._v(self.cam_world, 7),
            "CamWr_Z": self._v(self.cam_world, 8),
        }
        for i, name in enumerate(self.arm_joints):
            row[f"J{i+1}_pos"] = self.joint_pos.get(name, "")
            row[f"J{i+1}_vel"] = self.joint_vel.get(name, "")

        self.logger.log(**row)

    def cleanup(self):
        if self.active and self.logger is not None:
            path = self.logger.save()
            self.get_logger().info(f"[HumanAapfLogger] shutdown -> lưu {path}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanAapfLogger()
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

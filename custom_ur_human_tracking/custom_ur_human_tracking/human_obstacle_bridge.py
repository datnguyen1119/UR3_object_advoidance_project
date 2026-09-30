#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# human_obstacle_bridge — cầu nối giữa human tracking và hệ thống né AAPF.
#
# real_arm_node publish /human_skeleton (PoseArray gồm [elbow, wrist] ở frame
# "world"). Còn AAPF supervisor CHỈ đọc obstacle dạng SPHERE từ MoveIt planning
# scene (SupervisorNode::enumerateSpheres). Node này lấp khoảng trống đó: nó
# xấp xỉ đoạn cẳng tay (elbow→wrist) bằng một CHUỖI sphere rồi đẩy diff
# PlanningScene lên /planning_scene — đúng theo pattern của ObstacleManager
# (custom_ur_aapf_2/src/obstacle_manager.cpp).
#
# Vì tất cả sphere nằm trong MỘT CollisionObject duy nhất, mỗi lần cập nhật ta
# gửi REMOVE + ADD trong cùng một message để pose mới luôn thắng (giống
# ObstacleManager::applySphere). Khi mất tracking (không còn /human_skeleton quá
# stale_timeout), obstacle được REMOVE để robot không né "tay ma".

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from rcl_interfaces.msg import SetParametersResult


class HumanObstacleBridge(Node):
    def __init__(self):
        super().__init__('human_obstacle_bridge')

        # --- Tham số ---
        self.declare_parameter('skeleton_topic', '/human_skeleton')
        self.declare_parameter('object_id', 'human_forearm')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('num_spheres', 5)       # số sphere rải dọc cẳng tay
        self.declare_parameter('radius', 0.05)         # bán kính mỗi sphere cẳng tay [m]
        self.declare_parameter('hand_radius', 0.07)    # bán kính cầu BÀN TAY [m] (0 = tắt)
        self.declare_parameter('hand_offset', 0.05)    # tâm bàn tay cách cổ tay [m] theo hướng cẳng tay
        self.declare_parameter('stale_timeout', 0.5)   # mất tracking quá lâu -> gỡ obstacle [s]
        self.declare_parameter('check_rate', 10.0)     # tần suất kiểm tra stale [Hz]

        self.skeleton_topic = self.get_parameter('skeleton_topic').value
        self.object_id      = self.get_parameter('object_id').value
        self.frame_id       = self.get_parameter('frame_id').value
        self.num_spheres    = max(1, int(self.get_parameter('num_spheres').value))
        self.radius         = float(self.get_parameter('radius').value)
        self.hand_radius    = float(self.get_parameter('hand_radius').value)
        self.hand_offset    = float(self.get_parameter('hand_offset').value)
        self.stale_timeout  = float(self.get_parameter('stale_timeout').value)

        # Cho phép chỉnh nóng radius/hand_radius/hand_offset/num_spheres qua
        # `ros2 param set` mà KHÔNG cần restart (tránh bẫy "node đóng băng param").
        self.add_on_set_parameters_callback(self._on_params)
        check_rate          = float(self.get_parameter('check_rate').value)

        # --- Trạng thái ---
        self.last_msg_time = None    # thời điểm nhận skeleton gần nhất [s]
        self.obstacle_present = False

        # --- ROS plumbing ---
        # /planning_scene: RELIABLE + KEEP_LAST(10) — khớp cái move_group/PSM mong đợi
        # (giống QoS trong ObstacleManager).
        scene_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', scene_qos)

        self.create_subscription(PoseArray, self.skeleton_topic, self.skeleton_cb, 1)

        self.create_timer(1.0 / check_rate, self.stale_check)

        self.get_logger().info(
            f"[HumanObstacleBridge] Chuyển '{self.skeleton_topic}' -> {self.num_spheres} sphere cẳng tay "
            f"(r={self.radius:.3f}m) + 1 cầu bàn tay ở cổ tay (r={self.hand_radius:.3f}m) "
            f"id='{self.object_id}' frame='{self.frame_id}'."
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_params(self, params):
        """Cập nhật param nóng (áp dụng từ lần publish kế)."""
        for p in params:
            if p.name == 'radius':
                self.radius = float(p.value)
            elif p.name == 'hand_radius':
                self.hand_radius = float(p.value)
            elif p.name == 'hand_offset':
                self.hand_offset = float(p.value)
            elif p.name == 'num_spheres':
                self.num_spheres = max(1, int(p.value))
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------------------
    # Nhận skeleton (elbow, wrist) -> rải sphere dọc đoạn -> publish diff scene.
    # -------------------------------------------------------------------------
    def skeleton_cb(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return  # publish_skeleton của real_arm_node luôn gửi [elbow, wrist]

        elbow = np.array([msg.poses[0].position.x,
                          msg.poses[0].position.y,
                          msg.poses[0].position.z], dtype=np.float64)
        wrist = np.array([msg.poses[1].position.x,
                          msg.poses[1].position.y,
                          msg.poses[1].position.z], dtype=np.float64)

        # Cẳng tay: num_spheres cầu bán kính 'radius' rải elbow->wrist.
        spheres = [(c, self.radius) for c in self.sample_capsule(elbow, wrist, self.num_spheres)]
        # Bàn tay: 1 cầu 'hand_radius', tâm cách cổ tay 'hand_offset' theo hướng
        # cẳng tay (nối dài elbow->wrist) — tượng trưng bàn tay nhô ra khỏi cổ tay.
        if self.hand_radius > 0.0:
            fdir = wrist - elbow
            n = np.linalg.norm(fdir)
            hand_center = wrist + (fdir / n) * self.hand_offset if n > 1e-6 else wrist
            spheres.append((hand_center, self.hand_radius))
        self.publish_spheres(spheres, add=True)

        self.last_msg_time = self.now_sec()
        self.obstacle_present = True

    def sample_capsule(self, p_start, p_end, n):
        """Rải n điểm đều dọc đoạn p_start->p_end (bao gồm 2 đầu mút)."""
        if n == 1:
            return [(p_start + p_end) / 2.0]
        return [p_start + (p_end - p_start) * (i / (n - 1)) for i in range(n)]

    # -------------------------------------------------------------------------
    # Kiểm tra mất tracking: quá stale_timeout không có skeleton -> gỡ obstacle.
    # -------------------------------------------------------------------------
    def stale_check(self):
        if not self.obstacle_present:
            return
        if self.last_msg_time is None:
            return
        if self.now_sec() - self.last_msg_time > self.stale_timeout:
            self.publish_spheres([], add=False)
            self.obstacle_present = False
            self.get_logger().info("[HumanObstacleBridge] Mất tracking -> đã gỡ obstacle cẳng tay.")

    # -------------------------------------------------------------------------
    # Build CollisionObject (REMOVE + ADD) + gói vào diff PlanningScene + publish.
    # Bám sát ObstacleManager::applySphere để pose/bán kính mới luôn thắng.
    # -------------------------------------------------------------------------
    def publish_spheres(self, spheres, add):
        # spheres: list các (center[xyz], radius). [] khi chỉ REMOVE.
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        stamp = self.get_clock().now().to_msg()

        # 1. REMOVE (no-op nếu id chưa tồn tại — luôn an toàn để gửi trước).
        remove_obj = CollisionObject()
        remove_obj.id = self.object_id
        remove_obj.header.frame_id = self.frame_id
        remove_obj.header.stamp = stamp
        remove_obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove_obj)

        # 2. ADD chuỗi sphere với pose mới nhất (chỉ khi add=True).
        if add and len(spheres) > 0:
            add_obj = CollisionObject()
            add_obj.id = self.object_id
            add_obj.header.frame_id = self.frame_id
            add_obj.header.stamp = stamp
            add_obj.operation = CollisionObject.ADD
            # QUAN TRỌNG: obj.pose phải là identity với w=1.0. Quaternion (0,0,0,0)
            # mặc định là zero-norm -> MoveIt gộp shape_pose = obj.pose * primitive
            # sẽ sập về gốc toạ độ. (Xem ghi chú trong obstacle_manager.cpp.)
            add_obj.pose.orientation.w = 1.0

            for c, r in spheres:
                sphere = SolidPrimitive()
                sphere.type = SolidPrimitive.SPHERE
                sphere.dimensions = [r]
                add_obj.primitives.append(sphere)

                pose = Pose()
                pose.position.x = float(c[0])
                pose.position.y = float(c[1])
                pose.position.z = float(c[2])
                pose.orientation.w = 1.0
                add_obj.primitive_poses.append(pose)

            scene.world.collision_objects.append(add_obj)

        self.scene_pub.publish(scene)

    def cleanup(self):
        # Gỡ obstacle khi tắt node để không để lại "tay ma" trong scene.
        if self.obstacle_present:
            self.publish_spheres([], add=False)


def main(args=None):
    rclpy.init(args=args)
    node = HumanObstacleBridge()
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

# Tổng quan hệ thống — né vật cản AAPF cho UR3

> Tài liệu kiến thức. Đọc file này **trước**, rồi tới
> [`moveit_servo_va_doi_controller.md`](moveit_servo_va_doi_controller.md) (Servo + đổi controller)
> và [`supervisor_giai_thich.md`](supervisor_giai_thich.md) (chi tiết FSM).
> Phạm vi: cả workspace (sim + real). Ngày: 2026-06-16.

---

## 0. Câu chuyện một dòng

> Robot đi theo quỹ đạo MoveIt đã plan → thấy vật cản phía trước → né bằng trường thế năng
> AAPF (qua MoveIt Servo) → né xong quay lại quỹ đạo cũ → nếu nguy hiểm thì dừng.

---

## 1. Kiến trúc 4 tầng (Layered Architecture)

Hệ thống xếp chồng 4 tầng. Tầng trên ra **quyết định**, tầng dưới **thực thi**:

```
┌─ T3: AAPF Supervisor ─────────────────────── "BỘ NÃO" (FSM 5 trạng thái) ┐
│   đọc khoảng cách vật cản + κ + q̇ → quyết định mode → đổi controller     │
└──────────────────────┬────────────────────────────────────────────────┘
                       │ /delta_twist_cmds  hoặc  /delta_joint_cmds
┌─ T2: MoveIt Servo ───▼──────────────────── "DỊCH" vận tốc → vị trí khớp ┐
│   inverse-Jacobian (δθ = J⁺·δx) → stream vị trí khớp ~100 Hz            │
└──────────────────────┬────────────────────────────────────────────────┘
                       │ lệnh khớp (topic tùy sim/real — xem §5)
┌─ T1: ros2_control + move_group ── "TAY/CHÂN" controller_manager ────────┐
│   FOLLOW: (scaled_)joint_trajectory_controller — chạy quỹ đạo MoveIt    │
│   AVOID:  forward_position_controller (real) / cùng JTC (sim)           │
└──────────────────────┬────────────────────────────────────────────────┘
                       │ Gazebo (sim)  /  TCP-RTDE 192.168.1.1 (real)
                  ┌────▼────┐
                  │  UR3    │
                  └─────────┘
```

---

## 2. Vai trò từng package

| Package | Vai trò |
|---|---|
| `custom_ur_description` | URDF/xacro, mesh, SRDF nguồn của robot + gripper |
| `ur3_moveit_config` | Cấu hình MoveIt (kinematics, joint_limits, controllers, OMPL) |
| `custom_ur_simulation` | Launch tầng đáy: Gazebo (sim) **hoặc** `ur_ros2_control_node` (real) + move_group + rviz |
| `custom_ur_servo` | MoveIt Servo đã sửa (`custom_moveit_servo`) — tầng T2 |
| `custom_ur_aapf_2` | **Bộ não** né vật cản — tầng T3 (supervisor + AAPF + obstacle + estimator) |
| `custom_ur_teleop` | Điều khiển tay (teleop) bằng twist |
| `third_party/moveit_servo` | Bản gốc MoveIt Servo (tham chiếu logic scaling singularity) |

---

## 3. Vai trò từng file trong `custom_ur_aapf_2`

Package này theo mẫu **Humble Object**: logic thuần tách hẳn khỏi keo dán ROS.

| File | Vai trò | Loại |
|---|---|---|
| [`src/supervisor.cpp`](../src/supervisor.cpp) | **Bộ não thuần** — FSM 5 trạng thái. Nhận `TickInput` → trả `TickOutput`. Không có rclcpp/MoveIt. | Pure logic |
| [`src/aapf_generator.cpp`](../src/aapf_generator.cpp) | **Trường thế năng AAPF** — lực hút về P1 (`k_att`) + lực đẩy khỏi vật (`k_rep`) → twist. | Pure logic |
| [`src/singularity_handler.cpp`](../src/singularity_handler.cpp) | **Hysteresis coast** — κ vượt `kappa_enter` → JOINT_HOLD; tụt dưới `kappa_exit` → CARTESIAN_TWIST. | Pure logic |
| [`src/aapf_supervisor_node.cpp`](../src/aapf_supervisor_node.cpp) | **Keo ROS (adapter)** — sub topic, FK, liệt kê spheres → gọi `tick()` → publish + **đổi controller**. Chứa toàn bộ khác biệt sim/real. | ROS glue |
| [`src/aapf_supervisor_node_main.cpp`](../src/aapf_supervisor_node_main.cpp) | `main()` — spin node. | Entry |
| [`src/obstacle_manager.cpp`](../src/obstacle_manager.cpp) | Nguồn vật cản "test" — service `spawn`/`remove` vào planning scene. | Node phụ |
| [`src/interactive_obstacle.cpp`](../src/interactive_obstacle.cpp) | Nguồn vật cản kéo-thả trong RViz. | Node phụ |
| [`src/joint_velocity_estimator.cpp`](../src/joint_velocity_estimator.cpp) | Sai-phân `q̇` từ `/joint_states` → `/arm_joint_velocities` (cấp cho coast). | Node phụ |

---

## 4. FSM 5 trạng thái (bộ não quyết định gì)

Định nghĩa ở [`mode.hpp`](../include/custom_ur_aapf_2/mode.hpp). Chi tiết xem
[`supervisor_giai_thich.md`](supervisor_giai_thich.md).

```
FOLLOW_TRAJECTORY ──(vật cản gần)──► CHECK_AAPF_CONDITION ──► AAPF_AVOID
       ▲                                                          │
       │                                              (đường trước trống + EE đủ hở)
       └──────── RETURN_TO_TRAJECTORY ◄──────────────────────────┘
                          │
        (timeout / EE quá gần / lệch path > 0.5m / κ xấu) ──► STOP_OR_REPLAN
```

Mỗi tick supervisor đọc 3 tín hiệu và publish chẩn đoán:
- **Khoảng cách vật cản** → `~/obstacle_distance`
- **Condition number κ** (đo gần singularity) → `~/condition_number`, `~/near_singularity`
- **q̇** từ `JointVelocityEstimator` → `/arm_joint_velocities`

---

## 5. SIM vs REAL — khác gì, đổi ở đâu

**Quan trọng:** sim và real **dùng chung 100% mã nguồn**. Không có file `*_real.cpp`.
Khác biệt chỉ nằm ở **config + launch**. Có **hai** switch độc lập, đừng nhầm:

| | Singularity coast | Controller switch |
|---|---|---|
| Switch cái gì? | Kênh lệnh supervisor gửi **vào Servo** | Controller nào active trong **ros2_control** |
| Từ → đến | twist `/delta_twist_cmds` ↔ JointJog `/delta_joint_cmds` | `scaled_jtc` ↔ `forward_position_controller` |
| Cờ bật | `enable_singularity_coast` | `enable_controller_switch` |
| Sim | **true** ✅ | false |
| Real | **true** ✅ | **true** |
| Lý do tồn tại | bỏ qua scaling của Servo gần singularity | phần cứng thật không chịu được stream vào JTC |

→ Coast chạy **cả sim lẫn real**. Chỉ **controller switch** mới là real-only.
Chi tiết cả hai: [`moveit_servo_va_doi_controller.md`](moveit_servo_va_doi_controller.md).

### Ba thứ đổi khi chuyển sim → real (không sửa .cpp)

1. **Tầng đáy** ([`bringup_real_camera_gripper.launch.py`](../../custom_ur_simulation/launch/bringup_real_camera_gripper.launch.py)):
   Gazebo → `ur_ros2_control_node`; `sim_gazebo=false`, `use_fake_hardware=false`;
   spawn `scaled_jtc` (active) + `forward_position_controller` (inactive).
2. **Servo** ([`spawn_moveit_servo_real.launch.py`](../../custom_ur_servo/launch/spawn_moveit_servo_real.launch.py)):
   `command_out` → `/forward_position_controller/commands`, `use_gazebo=false`, `use_sim_time=false`.
3. **Supervisor** ([`aapf_supervisor_real.launch.py`](../launch/aapf_supervisor_real.launch.py)):
   nạp [`aapf_params_real.yaml`](../config/aapf_params_real.yaml) (bật controller switch) +
   nạp `kinematics.yaml` cho re-IK + `use_sim_time=false`.

> ⚠️ **Không** nạp `aapf_params_real.yaml` trong sim: nó bật switch sang
> `scaled_joint_trajectory_controller` (không tồn tại trong sim) → STRICT switch error → abort.

---

## 6. Workflow chạy ROBOT THẬT (4 terminal)

Lệnh đầy đủ trong [`run_commands.md`](../../run_commands.md). Mỗi terminal `source install/setup.bash`.

```bash
# T1 — bringup tầng đáy + move_group
ros2 launch custom_ur_simulation bringup_real_camera_gripper.launch.py robot_ip:=192.168.1.1
#   → PLAY "External Control" trên pendant, speed slider 10-20%
ros2 control list_controllers          # scaled_jtc + forward_position phải LOADED
ros2 control list_hardware_components   # 'ur' -> active

# T2 — MoveIt Servo (real)
ros2 launch custom_ur_servo spawn_moveit_servo_real.launch.py
ros2 service call /custom_moveit_servo/start_servo std_srvs/srv/Trigger {}

# T3 — AAPF supervisor (real) + obstacle + estimator
ros2 launch custom_ur_aapf_2 aapf_supervisor_real.launch.py
ros2 service call /aapf_supervisor/start std_srvs/srv/Trigger {}

# T4 — quan sát + thả vật cản
ros2 topic echo /aapf_supervisor/mode_text          # FOLLOW / AAPF_AVOID / RETURN / STOP
ros2 service call /obstacle_manager/spawn std_srvs/srv/Trigger {}

# Khôi phục sau STOP
ros2 service call /aapf_supervisor/reset std_srvs/srv/Trigger {}
```

---

## 7. Luồng end-to-end khi gặp vật cản (real)

```
vật cản tới
  → supervisor.tick() đổi mode sang AAPF_AVOID
  → onModeTransition: enable_controller_switch==true
       → cancel scaled goal + requestControllerSwitch(FORWARD)
       → service /controller_manager/switch_controller trả OK
       → callback MỞ gate servo_controller_active_=true
  → từ đó twist/JointJog mới được stream xuống forward_position_controller
  → (nếu gần singularity: coast đổi twist → JointJog, độc lập với switch trên)
  → né xong, mode về FOLLOW → switch ngược về scaled + resume re-IK
```

## Xem thêm
- [`moveit_servo_va_doi_controller.md`](moveit_servo_va_doi_controller.md) — Servo, singularity, đổi controller (chi tiết)
- [`supervisor_giai_thich.md`](supervisor_giai_thich.md) — FSM từng phần
- [`approach3_reik_resume_integration.md`](approach3_reik_resume_integration.md) — re-IK resume khi rejoin

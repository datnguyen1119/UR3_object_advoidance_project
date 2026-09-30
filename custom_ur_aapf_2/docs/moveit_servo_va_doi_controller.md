# MoveIt Servo, Singularity và đổi Controller

> Đọc [`he_thong_tong_quan.md`](he_thong_tong_quan.md) trước. File này đi sâu tầng T2 (Servo)
> và phần thực thi ở T1 (đổi controller). Phạm vi: `custom_ur_servo` + phần adapter của
> `custom_ur_aapf_2`. Ngày: 2026-06-16.

---

## 1. MoveIt Servo làm gì

Servo nhận **vận tốc** và biến thành **vị trí khớp** stream liên tục cho controller:

- **Đầu vào:** twist Cartesian trên `/delta_twist_cmds` (hoặc JointJog trên `/delta_joint_cmds`).
- **Biến đổi:** `δθ = J⁺·δx` (inverse-Jacobian).
- **Đầu ra:** vị trí khớp, ~100 Hz (`publish_joint_positions: true`).

### Vì sao Servo cố ý KHÔNG nạp kinematics

Trong [`spawn_moveit_servo_real.launch.py`](../../custom_ur_servo/launch/spawn_moveit_servo_real.launch.py)
phần `robot_description_kinematics` bị bỏ **có chủ đích**:

> Nếu có IK solver (KDL), Servo dùng đường `searchPositionIK` (lặp, timeout 5ms, nghiệm xấp xỉ)
> → giải thiếu mỗi chu kỳ → EE chậm ~10×. Bỏ kinematics → nhóm không có solver → Servo dùng
> **inverse-Jacobian** `δθ = J⁺·δx`, chính xác cho vận tốc lệnh và nhanh hơn.

> Lưu ý: **supervisor vẫn nạp** kinematics riêng (trong launch real) cho đường **re-IK resume** —
> supervisor và Servo là hai node khác nhau.

---

## 2. Đầu ra của Servo: SIM vs REAL

| | `command_out_type` | `command_out_topic` |
|---|---|---|
| **SIM** ([`moveit_servo.yaml:23,48`](../../custom_ur_servo/config/moveit_servo.yaml)) | `trajectory_msgs/JointTrajectory` | `/joint_trajectory_controller/joint_trajectory` |
| **REAL** ([`spawn_moveit_servo_real.launch.py:79`](../../custom_ur_servo/launch/spawn_moveit_servo_real.launch.py)) | `std_msgs/Float64MultiArray` | `/forward_position_controller/commands` |

Sim: Servo stream **JointTrajectory** vào **đúng cái JTC** mà MoveIt dùng cho FOLLOW — một
controller gánh cả hai việc. Real: stream **vị trí thô** (Float64MultiArray) vào
`forward_position_controller`. Khác biệt này dẫn thẳng tới chuyện đổi controller (§5).

---

## 3. Singularity — có HAI tầng xử lý đối nghịch nhau

### Tầng 1 — Hành vi mặc định của Servo: giảm tốc rồi dừng

Gần singularity `σ_min → 0` nên `J⁺` nổ → `q̇` khổng lồ. Servo chống bằng cách nhân vận tốc với
một hệ số scale — [`utilities.cpp:velocityScalingFactorForSingularity`](../../third_party/moveit_servo/src/utilities.cpp):

```cpp
κ = σ_max / σ_min;                          // condition number
if (κ > lower_threshold && κ < hard_stop) { // 25 < κ < 40 → bò chậm
    velocity_scale = 1 - (κ - lower)/(upper - lower);   // 1 → 0 tuyến tính
    status = DECELERATE_FOR_APPROACHING_SINGULARITY;
} else if (κ >= upper_threshold) {          // κ ≥ 40 → DỪNG HẲN
    velocity_scale = 0;
    status = HALT_FOR_SINGULARITY;
}
```

Ngưỡng trong [`moveit_servo.yaml:39-40`](../../custom_ur_servo/config/moveit_servo.yaml):
`lower_singularity_threshold: 25` (bắt đầu bò chậm), `hard_stop_singularity_threshold: 40` (dừng).
Status publish lên `/custom_moveit_servo/status` (0 ok, 1 vào sing, 6 ra sing, 2 halt).

> **Vấn đề:** hành vi này **đối nghịch với né vật**. Đang né mà gặp singularity, robot bò chậm
> dần rồi đứng im → kẹt khi vật cản đang tới. Đó là lý do có tầng 2.

### Tầng 2 — Singularity coast của supervisor ("trục 2")

**Chỉ trong mode `AAPF_AVOID`.** Tận dụng một bất đối xứng toán học
([`singularity_handler.hpp`](../include/custom_ur_aapf_2/singularity_handler.hpp)):

> `v = J·q̇` (xuôi) **luôn xác định**, kể cả TẠI singularity.
> `q̇ = J⁺·v` (ngược) mới nổ.

Thay vì gửi **vận tốc Cartesian** (buộc Servo đảo Jacobian → bị scale), supervisor **chộp q̇ đang
đo được** ngay trước vùng singular rồi gửi thẳng **vận tốc khớp** đó qua `/delta_joint_cmds`.
Đường JointJog của Servo **không gọi** `velocityScalingFactorForSingularity` → bỏ qua tầng 1 →
robot **trôi (coast)** qua ở tốc độ khớp hằng số.

Logic chọn kênh mỗi tick — [`aapf_supervisor_node.cpp:1135`](../src/aapf_supervisor_node.cpp):

```cpp
kappa = conditionNumber(current_state);          // tính y hệt Servo (σ_max/σ_min)
ChannelDecision d = singularity_->update(kappa, qd_measured);
if (d.channel == JOINT_HOLD) {
    JointJog jog; jog.velocities = d.joint_velocity;   // q̇ đã chộp
    joint_pub_->publish(jog);    // → /delta_joint_cmds (đường JOINT của Servo, KHÔNG scale)
    return;
}
twist_pub_->publish(twist);      // bình thường → /delta_twist_cmds
```

**Hysteresis chống chatter** ([`singularity_handler.cpp`](../src/singularity_handler.cpp)),
ngưỡng trong [`aapf_params_real.yaml`](../config/aapf_params_real.yaml):

```
κ > kappa_enter (20)  →  vào JOINT_HOLD, CHỘP q̇ tại biên
κ < kappa_exit  (18)  →  về CARTESIAN_TWIST
20 ≤ κ ≤ 18 (đệm)     →  giữ nguyên kênh hiện tại
```

`kappa_enter=20` **thấp hơn** `lower_singularity_threshold=25` của Servo — cố ý: coast giành
quyền **trước** khi Servo kịp giảm tốc.

> **Coast chạy CẢ sim lẫn real** (`enable_singularity_coast: true` ở cả hai yaml). Nó chỉ đổi
> *đầu vào* của Servo, không đụng ros2_control → không cần controller switch.

**An toàn dù open-loop:** trong lúc coast không bám đích Cartesian. Lưới chắn bên ngoài:
`emergency_distance` (EE-stop), `enforcePositionLimits` (Servo), `avoid_timeout`, `RETURN_TO_TRAJECTORY`
(tự hội tụ lại). Bẫy đã ghi chú: nếu q̇ chộp ≈ 0 thì hold sẽ stall — chỉ feed khi cánh tay đang
chuyển động né ổn định.

---

## 4. Chỉnh thuật ngữ: "đổi từ gửi vị trí sang gửi góc"

Coast **không** đổi giữa "vị trí" và "góc". Nó đổi giữa **hai loại vận tốc đưa VÀO Servo**:

- Bình thường: **vận tốc Cartesian** (twist — điều khiển EE trong không gian task)
- Gần singularity: **vận tốc khớp** (JointJog — q̇ trực tiếp trong không gian khớp)

Còn "gửi vị trí" là **đầu ra** của Servo xuống controller — Servo *luôn* xuất vị trí khớp,
bất kể đầu vào là twist hay JointJog. Cái đổi là **đầu vào**, không phải đầu ra.

---

## 5. Đổi controller (scaled_jtc ↔ forward_position_controller) — REAL ONLY

### Vì sao sim chỉ cần MỘT controller mà real phải có HAI

Sim dùng một JTC cho cả FOLLOW (quỹ đạo) lẫn AVOID (stream Servo). Real **không** làm vậy được,
vì hai lý do:

**1) `scaled_jtc` "thù địch" với streaming tần số cao.** JTC nội suy/ngoại suy giữa các điểm mỗi
chu kỳ. Khi Servo bắn trajectory 1-điểm ở 100 Hz, JTC nội suy lại từng cái → vận tốc gãy khúc →
**spike** → UR thật kích **protective stop**. Ghi chú trong launch real:

> *"scaled_jtc không hợp streaming → rung → spike → protective stop. JointGroupPositionController
> áp setpoint vị trí NGAY xuống servoj → mượt."*

Nên real dùng `forward_position_controller` (áp thẳng vị trí, không nội suy) để stream.

**2) Hai controller không thể cùng active — "command-interface exclusivity".** Cả `scaled_jtc`
lẫn `forward_position_controller` đều đòi sở hữu **cùng 6 position command-interface**. Trong
ros2_control mỗi interface chỉ thuộc **một** controller tại một thời điểm → bật cái này phải tắt
cái kia → **đó là switch** (`STRICT`).

> Sim "thoát" được vì Gazebo **không có** protective-stop: stream vào JTC vẫn rung nhưng không có
> motor thật để hỏng, không có safety để dừng. Đó là **xa xỉ của mô phỏng**, không phải bằng chứng
> switch là thừa.

```
                 FOLLOW (chạy quỹ đạo)        AVOID/coast (stream Servo)
SIM:             joint_trajectory_controller  ← cùng một cái, sim dung thứ
REAL:            scaled_jtc                    forward_position_controller
                 (tôn trọng speed slider)      (servoj mượt, không spike)
                        └──────── switch STRICT khi đổi mode ────────┘
```

### Chỗ trigger switch — 3 điểm trong code

Tất cả xoay quanh cờ `enable_controller_switch`.

**① Khởi tạo** ([`aapf_supervisor_node.cpp:275`](../src/aapf_supervisor_node.cpp)):
```cpp
if (enable_controller_switch) {                       // REAL
    switch_client_ = create_client<SwitchController>("/controller_manager/switch_controller");
    servo_cmd_pub_ = create_publisher<...>(servo_controller + "/commands", ...);
    servo_controller_active_ = false;   // GATE đóng — chưa cho stream
} else {                                              // SIM
    servo_controller_active_ = true;    // GATE mở vĩnh viễn
}
```

**② Trái tim — `onModeTransition()`** ([`aapf_supervisor_node.cpp:569`](../src/aapf_supervisor_node.cpp)):
cờ chia đôi hành vi.
```cpp
if (!enable_controller_switch) { /* SIM: chỉ cancelTrajectoryController() */ return; }
// REAL: switch + side-effect HOÃN vào callback thành công của switch
case AAPF_AVOID:        cancelTrajectoryController();
                        requestControllerSwitch(FORWARD, [this]{ servo_controller_active_ = true; });
case FOLLOW_TRAJECTORY: requestControllerSwitch(SCALED, [...]{ resumeTrajectoryController(ee); });
case STOP_OR_REPLAN:    requestControllerSwitch(SCALED, []{});
```

**③ Cổng stream (Gate)** ([`aapf_supervisor_node.cpp:1102`](../src/aapf_supervisor_node.cpp)):
```cpp
if (enable_controller_switch && !servo_controller_active_.load())
    return;   // chưa switch xong → tuyệt đối không stream (tránh 2 controller đánh nhau)
```

> Side-effect cần controller đích active (mở gate, resume quỹ đạo) đều **hoãn vào callback thành
> công** của switch — vì controller đích phải active trước. Switch FAIL → HALT.

---

## 6. Hai switch — đừng nhầm

| | Singularity coast (§3) | Controller switch (§5) |
|---|---|---|
| Switch cái gì? | Kênh lệnh **vào Servo** | Controller active trong **ros2_control** |
| Từ → đến | twist ↔ JointJog | scaled_jtc ↔ forward_position_controller |
| Cờ | `enable_singularity_coast` | `enable_controller_switch` |
| Sim / Real | true / true | false / true |
| Kích khi | κ vượt ngưỡng (trong AVOID) | đổi mode FSM (FOLLOW↔AVOID) |

## Xem thêm
- [`he_thong_tong_quan.md`](he_thong_tong_quan.md) — kiến trúc, workflow, sim vs real
- [`supervisor_giai_thich.md`](supervisor_giai_thich.md) — FSM từng phần
- [`approach3_reik_resume_integration.md`](approach3_reik_resume_integration.md) — re-IK resume

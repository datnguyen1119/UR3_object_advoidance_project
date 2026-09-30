# Tích hợp Cách 3 — Resume bằng Re-IK theo chuỗi seed

> Tài liệu thiết kế tích hợp. **Chưa sửa code thật** — đây là bản kế hoạch để duyệt logic trước.
> Phạm vi: gói `custom_ur_aapf_2`. Ngày: 2026-06-12.

---

## 1. Vấn đề đang giải

Sau khi né vật (AAPF_AVOID), supervisor đưa EE về gần quỹ đạo cũ rồi gọi
`resumeTrajectoryController()` → `buildResumeFrom()` để phát lại **đoạn đuôi quỹ đạo gốc**
cho `joint_trajectory_controller` (JTC).

`buildResumeFrom()` hiện tại:
- tìm waypoint gốc gần EE nhất theo **toạ độ Descartes**,
- cắt đoạn đuôi và phát lại **đúng góc khớp gốc**.

**Lỗi gặp phải:**
```
State tolerances failed for joint 5:  Position Error: 2.395 rad  (Tolerance: 0.200)
Aborted due to state tolerance violation
```

Nguyên nhân gốc: né vật chạy bằng MoveIt Servo (Cartesian thuần, không khống chế cấu hình
khớp) → robot trôi sang **một nhánh IK khác** (cổ tay lật ~137°). Khi resume bằng góc khớp
gốc (nhánh ban đầu), JTC so điểm đầu quỹ đạo với trạng thái khớp thật → lệch 2.4 rad → abort.

> Gần nhau trong không gian Descartes ≠ gần nhau trong không gian khớp.

---

## 2. Nguyên lý Cách 3

**Giữ HÌNH HỌC đường EE, tính lại NGHIỆM KHỚP.**

EE vẫn đi đúng đường cũ trong không gian, nhưng chuỗi góc khớp được giải lại để **bắt đầu
từ cấu hình thật hiện tại** và biến đổi liên tục — không có cú nhảy nhánh nào.

Kỹ thuật then chốt — **IK theo chuỗi seed (seeded continuation IK):**
KDL là solver cục bộ, trả nghiệm gần "seed" nhất. Vì vậy:

```
seed₀ = q_now (góc khớp THẬT, đang ở nhánh lật)
q[start]   = IK(pose[start],  seed = q_now)        → gần q_now, vẫn nhánh lật
q[start+1] = IK(pose[start+1], seed = q[start])     → tiếp nối nhánh lật
q[i]       = IK(pose[i],       seed = q[i-1])        → toàn chuỗi liên tục
```

Điểm đầu ≈ `q_now` → JTC kiểm `state tolerance` PASS. Cổ tay giữ nguyên trạng thái lật suốt
đoạn còn lại — hoàn toàn ổn vì EE vẫn vẽ đúng đường.

---

## 3. Ràng buộc kiến trúc PHẢI tôn trọng

`Supervisor` (trong `src/supervisor.cpp`, `include/.../supervisor.hpp`) là **core thuần**:

```cpp
// It does NOT depend on rclcpp / MoveIt at runtime.   (supervisor.hpp:13)
```

→ **TUYỆT ĐỐI không** đưa `RobotState::setFromIK` / `trajectory_processing` (đều là MoveIt)
vào `supervisor.cpp`. Toàn bộ logic Re-IK + TOTG phải nằm ở **tầng node**
(`aapf_supervisor_node.cpp`) — nơi đã sở hữu `psm_`, `RobotModel`, `jmg_`, `RobotState`.

Hệ quả thiết kế:
- `Supervisor::buildResumeFrom()` **giữ nguyên** — nó vẫn trả về đoạn đuôi (góc khớp gốc +
  thời gian gốc). Ta dùng nó như **nguồn hình học**: từ góc khớp gốc, node sẽ FK ra pose.
- Logic mới (`reSolveResumeIK`) là **method mới của node**, không đụng core.

---

## 4. Kiến trúc tích hợp — khe cắm (seam)

Không đổi FSM. Chỉ thay phần thân của hook resume.

```
RETURN_TO_TRAJECTORY ──(rejoin)──► FOLLOW_TRAJECTORY
        onModeTransition()  [aapf_supervisor_node.cpp:576-583 và 613-623]
                │
                └─► resumeTrajectoryController(ee_now, q_now)   ← SỬA HÀM NÀY
                        │
                        ├─ suffix = supervisor_->buildResumeFrom(ee_now)   (CORE, giữ nguyên)
                        │           → JointTrajectory với GÓC KHỚP GỐC
                        │
                        ├─ reSolveResumeIK(suffix, q_now)        ← METHOD MỚI (tầng node)
                        │     for mỗi point:
                        │        pose = FK(point.positions)       (node đã có computeEE pattern)
                        │        q    = IK(pose, seed)            (setFromIK trên RobotState)
                        │        seed = q
                        │     → RobotTrajectory (chưa có thời gian)
                        │     → TOTG re-time                      (trajectory_processing)
                        │     → JointTrajectory mới
                        │
                        └─ follow_client_->async_send_goal(traj mới)
```

**Tổng kết "ai làm gì":**

| Thành phần | Thay đổi | Lý do |
|---|---|---|
| `supervisor.cpp/.hpp` (core) | **Không đổi** | Giữ core ROS/MoveIt-free |
| `buildResumeFrom()` | **Không đổi** | Vẫn cấp đoạn đuôi (nguồn hình học) |
| `aapf_supervisor_node.hpp` | + members + khai báo method | Chỗ chứa IK state + cờ bật |
| `aapf_supervisor_node.cpp` | + `reSolveResumeIK()`, sửa `resumeTrajectoryController()` | Logic Re-IK + TOTG |
| `aapf_params.yaml` | + cờ `reik_resume` | Bật/tắt, an toàn rollback |
| `aapf_supervisor.launch.py` | + nạp `kinematics.yaml` | `setFromIK` cần solver trong namespace node |
| `CMakeLists.txt` / `package.xml` | xác nhận link `trajectory_processing` | TOTG nằm trong moveit_core |

---

## 5. Danh sách thay đổi cụ thể

### 5.1 Launch — nạp kinematics vào namespace supervisor *(điều kiện tiên quyết)*

Hiện `kinematics.yaml` (`ur3_moveit_config/config/kinematics.yaml`,
solver `kdl_kinematics_plugin/KDLKinematicsPlugin`, group `ur3_manipulator`) **chỉ** được
nạp cho node Servo, KHÔNG nạp cho `aapf_supervisor`.

`RobotState::setFromIK` cần param `robot_description_kinematics` trong namespace của **chính
node gọi nó**. PSM của supervisor (`PlanningSceneMonitor(..., "robot_description")`) sẽ nạp
solver qua RobotModelLoader **nếu** param này hiện diện.

→ Trong `aapf_supervisor.launch.py:59-65`, bổ sung kinematics.yaml vào `parameters=`:
- nạp file `ur3_moveit_config/config/kinematics.yaml` (hoặc dùng `MoveItConfigsBuilder`),
- key kỳ vọng: `aapf_supervisor.robot_description_kinematics.ur3_manipulator.kinematics_solver: ...`

**Sanity check sau khi sửa:** `jmg_->getSolverInstance() != nullptr` (log một lần lúc init).
Nếu null → solver chưa nạp được → Cách 3 không chạy.

> Ghi chú: launch real cố tình KHÔNG nạp kinematics cho *Servo* (để Servo dùng inverse-Jacobian
> nhanh hơn). Điều này KHÔNG xung đột: Re-IK của ta chạy **một lần khi rejoin** (offline), không
> phải mỗi chu kỳ servo, nên timeout KDL 5 ms chấp nhận được.

### 5.2 Build — xác nhận `trajectory_processing`

`moveit_core` đã là dependency (`CMakeLists.txt:23,65`; `package.xml`). Header TOTG
(`moveit/trajectory_processing/time_optimal_trajectory_generation.h`) nằm trong moveit_core.
→ Nhiều khả năng **không cần thêm dependency**, chỉ cần `#include` và build thử. Nếu linker
báo thiếu, thêm target `moveit_trajectory_processing` vào `target_link_libraries`.

### 5.3 Node header (`aapf_supervisor_node.hpp`)

Thêm:
- một `moveit::core::RobotStatePtr ik_state_;` (state làm việc cho IK, khởi tạo ở `deferredInit`),
- khai báo `trajectory_msgs::msg::JointTrajectory reSolveResumeIK(const trajectory_msgs::msg::JointTrajectory& suffix, const std::vector<double>& q_now) const;` (hoặc trả `std::optional`),
- đổi chữ ký `resumeTrajectoryController(const Eigen::Vector3d& ee_now)` → thêm `q_now` (góc khớp hiện tại). Cả hai call site (`:582`, `:621`) đều đang ở chỗ có sẵn `RobotState` để lấy `q_now`.

### 5.4 Node cpp (`aapf_supervisor_node.cpp`)

**(a) Method mới `reSolveResumeIK` — pseudocode minh hoạ (KHÔNG phải code cuối):**

```cpp
// TẦNG NODE — được phép dùng MoveIt. Đây là pseudocode để duyệt logic.
std::optional<JointTrajectory>
SupervisorNode::reSolveResumeIK(const JointTrajectory& suffix,
                                const std::vector<double>& q_now) const
{
  if (suffix.points.size() < 2) return std::nullopt;

  // RobotTrajectory để chứa nghiệm mới + đưa vào TOTG.
  auto rt = std::make_shared<robot_trajectory::RobotTrajectory>(
                psm_->getRobotModel(), node_params_.move_group_name);

  moveit::core::RobotState seed(*ik_state_);
  seed.setJointGroupPositions(node_params_.move_group_name, q_now);  // seed₀ = q_now
  seed.update();

  for (const auto& pt : suffix.points)
  {
    // 1) FK: góc khớp GỐC -> pose 6-DOF đầy đủ (vị trí + hướng).
    moveit::core::RobotState pose_state(psm_->getRobotModel());
    pose_state.setJointGroupPositions(node_params_.move_group_name, pt.positions);
    pose_state.update();
    const Eigen::Isometry3d target = pose_state.getGlobalLinkTransform(node_params_.ee_link);

    // 2) IK theo chuỗi seed: nghiệm gần `seed` nhất.
    moveit::core::RobotState sol(seed);
    const bool ok = sol.setFromIK(jmg_, target, node_params_.ee_link,
                                  /*timeout=*/0.05 /* nới rộng vì offline */);
    if (!ok)
    {
      // fallback: bỏ qua điểm này, hoặc nếu fail nhiều -> trả nullopt (đẩy STOP).
      continue;
    }

    rt->addSuffixWayPoint(sol, 0.0 /* dt set lại bởi TOTG */);
    seed = sol;  // tiếp nối nhánh
  }

  if (rt->getWayPointCount() < 2) return std::nullopt;

  // 3) Re-time: TOTG gán lại vận tốc/gia tốc theo giới hạn robot.
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(*rt)) return std::nullopt;

  // 4) Xuất ra JointTrajectory cho JTC.
  moveit_msgs::msg::RobotTrajectory msg;
  rt->getRobotTrajectoryMsg(msg);
  return msg.joint_trajectory;
}
```

**(b) Sửa `resumeTrajectoryController` (`:645-667`):**

```cpp
void SupervisorNode::resumeTrajectoryController(const Eigen::Vector3d& ee_now,
                                                const std::vector<double>& q_now)
{
  if (!follow_client_) return;

  auto suffix = supervisor_->buildResumeFrom(ee_now);          // CORE, giữ nguyên
  if (!suffix || suffix->points.size() < 2) { /* hết đường */ return; }

  trajectory_msgs::msg::JointTrajectory to_send;
  if (node_params_.reik_resume) {
    auto reik = reSolveResumeIK(*suffix, q_now);               // MỚI
    if (!reik) { /* IK/TOTG fail -> log + return (KHÔNG gửi đoạn lệch) */ return; }
    to_send = *reik;
  } else {
    to_send = *suffix;                                          // hành vi cũ (rollback)
  }

  // MẸO: ép điểm đầu = q_now để start tolerance ≈ 0 tuyệt đối.
  // (chèn 1 point t=0 với positions = q_now nếu cần)

  if (!follow_client_->action_server_is_ready()) { /* warn */ return; }
  FollowJointTrajectory::Goal goal;
  goal.trajectory = to_send;
  follow_client_->async_send_goal(goal);
}
```

**(c) Lấy `q_now` tại call site:** trong `onModeTransition`, đọc trạng thái khớp thật qua PSM
(đúng pattern đang dùng ở `supervisorTick`):
```cpp
planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
const auto q_now = readArmJoints(scene->getCurrentState());   // readArmJoints có sẵn :297
resumeTrajectoryController(ee_now, q_now);
```

### 5.5 Param (`aapf_params.yaml`)

Thêm cờ bật/tắt để rollback an toàn:
```yaml
reik_resume: true     # true = Cách 3 (Re-IK); false = hành vi cũ (phát lại góc khớp gốc)
```

---

## 6. Luồng runtime sau tích hợp (timeline một lần rejoin)

```
t0  né xong, đường thông, EE cách đường ≤ 5cm     → FSM: RETURN → FOLLOW
t1  onModeTransition(FOLLOW):
      publish zero-twist (Servo nhả tay)
      lock PSM → q_now = readArmJoints(getCurrentState)   (nhánh lật)
      resumeTrajectoryController(ee_now, q_now)
t2    ├─ buildResumeFrom(ee_now)        → suffix (góc khớp gốc, nhánh ban đầu)
      ├─ reSolveResumeIK(suffix, q_now):
      │     FK từng point → pose 6-DOF
      │     chained-seed IK từ q_now → q[...] (toàn nhánh lật, liên tục)
      │     TOTG → gán thời gian
      └─ async_send_goal(traj mới)       (điểm đầu ≈ q_now)
t3  JTC nhận goal → state tolerance PASS → chạy tiếp tới đích
```

Bước t1→t2 **đồng bộ, vài ms** (IK + TOTG cho vài chục điểm rất nhanh) → không cần async,
không cần holding-state, **không đụng FSM**.

---

## 7. Tình huống biên & fallback

| Tình huống | Xử lý |
|---|---|
| `setFromIK` fail tại 1 điểm | bỏ qua điểm đó (continue); nếu rớt quá nhiều → trả `nullopt` |
| `reSolveResumeIK` trả `nullopt` | KHÔNG gửi đoạn lệch; log; (tuỳ chọn) đẩy FSM về `STOP_OR_REPLAN` |
| TOTG `computeTimeStamps` fail | trả `nullopt` → như trên |
| Solver chưa nạp (`getSolverInstance()==nullptr`) | log lỗi cứng lúc init; coi như `reik_resume=false` |
| Vật cản vẫn quanh đoạn nối | **Cách 3 KHÔNG tự né** — giả định lookahead đã xác nhận đường thông; nếu cần né → dùng Cách 4 |

---

## 8. Cách kiểm chứng

1. **Build:** `colcon build --packages-select custom_ur_aapf_2` (xác nhận TOTG link OK).
2. **Solver loaded:** chạy supervisor, kiểm log `getSolverInstance() != nullptr`.
3. **Kịch bản lỗi cũ:** spawn obstacle giữa đường, để robot né + rejoin.
   - Trước: JTC abort `state tolerance` joint 5.
   - Sau: JTC nhận goal, chạy tới đích, **không** abort.
4. **Đối chứng:** `reik_resume:=false` → tái hiện lỗi cũ; `true` → hết lỗi. Xác nhận đúng nguyên nhân.
5. **EE bám đường:** so quỹ đạo EE thực tế với đường gốc (RViz) — phải trùng hình học.

---

## 9. Rủi ro & rollback

- **Rollback tức thì:** đặt `reik_resume: false` → về đúng hành vi cũ, không cần build lại.
- **Rủi ro chính:** kinematics.yaml chưa vào đúng namespace (mục 5.1) → IK luôn fail. Kiểm bằng sanity check mục 8.2.
- **Không ảnh hưởng** đường đi khi KHÔNG có vật cản (chỉ kích hoạt ở nhánh rejoin sau né).
- **Core không đổi** → các unit test core (`test_singularity_handler`) không bị ảnh hưởng.

---

## 10. Thứ tự thực hiện đề xuất

1. Sửa launch nạp kinematics.yaml + thêm sanity check log → xác nhận solver loaded.
2. Thêm cờ `reik_resume` (mặc định false) + đổi chữ ký `resumeTrajectoryController` (chưa bật Re-IK).
3. Viết `reSolveResumeIK` (FK → chained-seed IK → TOTG).
4. Bật `reik_resume:=true`, chạy kịch bản mục 8.
5. Tinh chỉnh: timeout IK, mẹo ép điểm đầu = q_now, ngưỡng fallback.
```

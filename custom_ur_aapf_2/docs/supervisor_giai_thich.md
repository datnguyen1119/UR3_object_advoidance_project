# Giải thích chi tiết `supervisor.cpp`

> File này giải thích **từng phần** của [`src/supervisor.cpp`](../src/supervisor.cpp) —
> mỗi thành phần làm gì và vì sao có nó. Đọc kèm header
> [`include/custom_ur_aapf_2/supervisor.hpp`](../include/custom_ur_aapf_2/supervisor.hpp).

## 0. Bức tranh tổng thể

`Supervisor` là **bộ não ra quyết định** của hệ thống né vật cản AAPF. Nó:

- Quản lý một **máy trạng thái (FSM) 5 trạng thái**.
- Lưu **đường đi đã chia nhỏ** (dense cache) để suy luận về vật cản.
- Mỗi nhịp (`tick`) nhận vào *vị trí robot + danh sách vật cản*, trả ra *nên làm gì*.

**Điểm quan trọng:** class này **không phụ thuộc rclcpp/MoveIt**. Nó chỉ dùng
`Eigen` + kiểu message `trajectory_msgs` (chỉ là dữ liệu). Nhờ vậy có thể test
mà không cần chạy ROS. Phần FK (động học) được *truyền vào từ ngoài* dưới dạng
`std::function`.

Câu chuyện một dòng:

> Robot đi theo đường vạch sẵn → thấy vật cản phía trước → né → né xong quay lại
> đường cũ → nếu nguy hiểm thì dừng.

---

## 1. Constructor

```cpp
Supervisor::Supervisor(Params params) : params_(std::move(params)) {}
```

- **Mục đích:** nhận toàn bộ cấu hình (`Params`) một lần khi tạo object và lưu lại.
- `Params` chứa mọi ngưỡng: khoảng cách an toàn, tầm nhìn trước, timeout… (xem
  `supervisor.hpp`). Truyền vào từ ngoài → core không tự đọc ROS param.
- `std::move` để tránh copy cái struct `Params` (có cả `vector<string>` tên khớp).

---

## 2. Lifecycle — bật/tắt và reset

Ba hàm điều khiển vòng đời, tất cả đều khóa `mutex_` vì có thể bị gọi từ thread
khác (service callback của node).

### `setRunning(bool)`
```cpp
void Supervisor::setRunning(bool running)
```
- **Mục đích:** bật/tắt bộ giám sát.
- Khi **tắt** (`running=false`): ép `mode_ = STOP_OR_REPLAN` (dừng an toàn) và xóa lỗi.
- Khi **bật**: chỉ đặt cờ `running_`, *giữ nguyên* mode hiện tại (node thường gọi
  `reset()` trước rồi mới `setRunning(true)`).

### `isRunning()`
- **Mục đích:** trả về cờ `running_`. Node dùng nó để chặn `supervisorTick` ngay
  dòng đầu khi chưa bật.

### `reset()`
```cpp
void Supervisor::reset()
```
- **Mục đích:** đưa FSM về trạng thái đầu `FOLLOW_TRAJECTORY` và xóa lỗi.
- Đây là cách **duy nhất** để thoát khỏi trạng thái terminal `STOP_OR_REPLAN`
  (gọi qua service `~/reset`).

---

## 3. Trajectory ingestion — nạp đường đi

### `ingestTrajectory(jt, fk)`
```cpp
std::optional<IngestStats> Supervisor::ingestTrajectory(
    const trajectory_msgs::msg::JointTrajectory& jt, const FKFunction& fk)
```

- **Mục đích:** nhận một quỹ đạo khớp (joint trajectory) từ MoveIt và biến nó
  thành dữ liệu mà FSM dùng được. Chạy **một lần mỗi khi có đường mới**.
- **`fk` là gì:** hàm forward-kinematics được *inject* — đổi góc khớp → vị trí
  đầu robot (EE). Core không tự tính FK (cần MoveIt), mà nhận từ node.

**Các bước bên trong:**
1. `jt.points.empty()` → trả `nullopt` (không có gì để làm).
2. `traj::filter(...)` → lọc lấy **đúng 6 khớp tay** theo thứ tự `arm_joints`.
   Nếu thiếu khớp → bắt exception, trả `nullopt`.
3. Tính `orig_ee` — vị trí EE của **từng điểm gốc**, dùng cho lúc *resume*.
4. `traj::subdivideCartesian(...)` → **chia nhỏ** đường sao cho mỗi đoạn ≈ 1cm
   trong không gian Cartesian. Kết quả là "dense cache" để dò vật cản mịn.
5. Khóa `mutex_`, ghi vào các biến thành viên:
   - `cached_ee_positions_` ← chuỗi vị trí EE dày đặc (dùng để suy luận né).
   - `original_traj_` / `original_ee_positions_` ← đường gốc (dùng để resume).
   - `current_waypoint_idx_ = 0` (bắt đầu từ đầu đường).
   - Nếu đang chạy → đặt lại `mode_ = FOLLOW_TRAJECTORY` (đường mới ghi đè mọi
     trạng thái né/dừng cũ).
6. Trả `IngestStats` (số điểm gốc / số điểm sau chia / số khớp) để node log.

> Ghi chú: `cached_traj_` được ghi nhưng **không được đọc lại** ở đâu — đây là
> phần state thừa (chỉ `cached_ee_positions_` được dùng cho suy luận).

---

## 4. Hàm phụ trợ quét cửa sổ (yêu cầu `mutex_` đã khóa bởi hàm gọi)

Ba hàm này là "công cụ" cho `tick`/`runCheck`. Chúng **không tự khóa mutex** vì
luôn được gọi từ trong `tick()` (đã khóa rồi).

### `clearanceAt(idx, spheres)`
```cpp
double Supervisor::clearanceAt(std::size_t idx, ...)
```
- **Mục đích:** trả về **khoảng trống (clearance)** tại waypoint thứ `idx` — tức
  khoảng cách từ điểm EE đó tới mặt quả cầu vật cản gần nhất.
- Số dương = thoáng, số âm = đã chạm/đè vật. Dùng `traj::signedDistanceToSpheres`.

### `scanWindow(spheres, count)`
```cpp
WindowScan Supervisor::scanWindow(...) const
```
- **Mục đích:** quét `count` waypoint *phía trước* EE và tổng hợp tình hình vật cản.
- Trả về struct `WindowScan` gồm:
  - `min_clearance` — khoảng trống nhỏ nhất trong cửa sổ (mức độ "nguy hiểm").
  - `any_blocked` — có waypoint nào bị chặn không (clearance < `blocked_distance_threshold`).
  - `first_blocked` / `last_blocked` — biên đầu/cuối của vùng bị chặn.
- **Dùng 2 kiểu:** cửa sổ *ngắn* (`lookahead_waypoint_count`) để **phát hiện**;
  cửa sổ *dài* (`p1_search_horizon`) để **tìm lối thoát P1**.

### `findSafeIndex(spheres, lo, hi, pref)`
```cpp
std::optional<std::size_t> Supervisor::findSafeIndex(...)
```
- **Mục đích:** trong khoảng index `[lo, hi)`, tìm waypoint **an toàn** (clearance
  ≥ `safe_distance`) **gần `pref` nhất**.
- Cách làm: bung đối xứng ra hai phía từ `pref` (pref, pref±1, pref±2…) → chọn
  được điểm an toàn sát vị trí mong muốn nhất.
- Trả `nullopt` nếu cả khoảng không có điểm nào an toàn.
- Đây là "viên gạch" để xác định **P1** (điểm thoát sau vật) và **P2** (điểm an
  toàn trước vật).

---

## 5. `runCheck` — kiểm tra điều kiện né (state CHECK_AAPF_CONDITION)

```cpp
Mode Supervisor::runCheck(const TickInput& in, TickOutput& o)
```

- **Mục đích:** khi vừa phát hiện vật cản, hàm này quyết định **có né được không**
  bằng cách xác định cặp điểm P1/P2 và kiểm tra các điều kiện kích hoạt.
- Trả về **mode kế tiếp**; nếu hỏng thì set `o.error` và trả `STOP_OR_REPLAN`.

**P1 và P2 là gì:**
- **P2** = điểm an toàn **ngay trước** vùng vật cản (gần EE) → xác nhận robot
  đang ở chỗ an toàn để bắt đầu né.
- **P1** = điểm an toàn **ngay sau** vùng vật cản (xa) → đây là **đích hút
  (attractor)** mà robot sẽ tiến về để vòng qua vật.

**Các bước:**
1. Quét cửa sổ **dài** (`p1_search_horizon`) để thấy trọn vùng bị chặn.
2. Nếu `!any_blocked` → đường đã tự thông → quay lại `FOLLOW_TRAJECTORY`.
3. Tìm **P2** trong `[current_idx, first_blocked)`. Không có → lỗi `INVALID_P2`
   → STOP (điều kiện 3 hỏng).
4. Tìm **P1** trong `(last_blocked, window_end)`. Không có → lỗi `NO_VALID_P1`
   → STOP (điều kiện 2 hỏng).
5. Tính khoảng cách `sep = |P1 − P2|`. Nếu quá nhỏ, đẩy P1 ra xa dần để đích
   có ý nghĩa.
6. Kiểm tra **điều kiện 4** (kích thước vật):
   - `sep > max_p1_p2_distance` → vật quá lớn → lỗi `OBSTACLE_TOO_LARGE` → STOP.
   - `sep < min_p1_p2_distance` → vật không đáng kể → cứ `FOLLOW` tiếp.
7. Đủ điều kiện → đặt `o.goal = P1`, ghi `avoid_start_sec_` (mốc tính timeout),
   chuyển sang `AAPF_AVOID`.

> "5 điều kiện" được tham chiếu theo §4 trong tài liệu thuật toán gốc. Điều kiện 5
> (khẩn cấp) **không** nằm ở đây — nó được xử lý sớm hơn trong `tick()`.

---

## 6. `tick` — trái tim FSM (gọi mỗi nhịp, ~30 Hz)

```cpp
Supervisor::TickOutput Supervisor::tick(const TickInput& input)
```

Đây là hàm trung tâm. Khóa `mutex_` suốt cả hàm. Cấu trúc gồm **guard ưu tiên
trước, rồi mới đến `switch` theo state**.

### 6.1 Khởi tạo output + đọc số liệu cơ bản
- Tạo `TickOutput o`, lưu `previous_mode`, kích thước cache, số vật cản.
- `o.ee_clearance` = khoảng trống tại vị trí EE hiện tại (không phải waypoint).
- `last_goal_.reset()` — xóa đích cũ; chỉ các state né/return mới đặt lại đích.

### 6.2 Guard 1 — không chạy / chưa có đường
```cpp
if (!running_ || cached_ee_positions_.empty()) { ... return; }
```
- Chưa bật hoặc chưa nạp đường → giữ nguyên state, thoát sớm.

### 6.3 Guard 2 — input cũ (stale)
```cpp
if (!input.inputs_fresh) { ... STOP_OR_REPLAN / SCENE_STALE ... }
```
- Khi node báo dữ liệu scene không còn tươi → dừng an toàn (trừ khi đã đang STOP).

### 6.4 Cập nhật vị trí trên đường
```cpp
current_waypoint_idx_ = traj::nearestWaypointIndex(...)
o.path_deviation = ...
```
- Tìm waypoint **gần EE nhất** (tìm tăng dần, trong cửa sổ trước mặt — robot
  không đi lùi).
- `path_deviation` = robot lệch khỏi đường bao xa.

### 6.5 Điểm lookahead debug (luôn bật)
- Tính 2 điểm cố định phía trước EE (`vis_p2_offset`, `vis_p1_offset`) **chỉ để
  hiển thị** trong RViz. Không liên quan tới P1/P2 thật.

### 6.6 Guard 3 — khẩn cấp (điều kiện 5, ưu tiên cao nhất)
```cpp
const bool emergency = o.ee_clearance < params_.emergency_distance;
if (emergency && mode_ != STOP_OR_REPLAN) { ... EE_TOO_CLOSE_TO_OBSTACLE ... }
```
- EE quá sát vật → **dừng ngay**, bất kể đang ở state nào. Đặt **trước** `switch`
  nên nó override mọi trạng thái.

### 6.7 `switch (mode_)` — logic từng state

| State | Làm gì | Chuyển sang khi |
|---|---|---|
| `FOLLOW_TRAJECTORY` | quét lookahead **ngắn** | thấy bị chặn → `CHECK_AAPF_CONDITION` |
| `CHECK_AAPF_CONDITION` | gọi `runCheck` | tùy kết quả (AVOID / FOLLOW / STOP) |
| `AAPF_AVOID` | kiểm tra lệch/timeout; tái xác định P1; đặt `goal` | đường trước sạch + EE thoáng → `RETURN`; lỗi → `STOP` |
| `RETURN_TO_TRAJECTORY` | lái về đường; đặt `goal` waypoint phía trước | gần đường → `FOLLOW`; vật mới → `CHECK`; timeout → `STOP` |
| `STOP_OR_REPLAN` | không làm gì | terminal, chỉ thoát qua `reset()` |

Chi tiết đáng chú ý trong `AAPF_AVOID`:
- Dùng **2 lần quét**: *gần* (`w_near`, quyết định khi nào đoạn trước sạch) và
  *xa* (`w`, định vị vùng vật + tìm P1).
- **Tái xác định P1 mỗi nhịp** vì vật cản có thể di chuyển. Mất P1 → lỗi
  `P1_UNSTABLE_OR_INVALID` → STOP.

### 6.8 Commit chuyển trạng thái (mẫu FSM kinh điển)
```cpp
o.mode_changed = (next != mode_);
mode_ = next;          // ← chuyển state ở đây
o.mode = mode_; o.error = error_;
last_goal_ = o.goal;   // mirror cho vòng twist nhanh đọc
```
- `last_goal_` được "soi gương" để `twistTick` (200 Hz) đọc đích mà không phải
  chạy lại toàn bộ logic quyết định.
- Cuối cùng, đổi `p1_idx`/`p2_idx` → vị trí Cartesian `p1_pos`/`p2_pos` cho RViz.

---

## 7. Resume support — nối lại đường sau khi né xong

### `buildResumeFrom(ee)`
```cpp
std::optional<trajectory_msgs::msg::JointTrajectory>
Supervisor::buildResumeFrom(const Eigen::Vector3d& ee) const
```

- **Mục đích:** sau khi né xong và rejoin, tạo **phần đuôi** của đường **gốc** để
  gửi lại cho controller chạy tiếp.
- **Các bước:**
  1. Tìm waypoint gốc gần vị trí `ee` hiện tại nhất (`nearestWaypointIndexGlobal`).
  2. Nếu đã ở cuối đường → `nullopt`.
  3. Copy các điểm từ `start` đến hết.
  4. **Rebase thời gian:** trừ đi `time_from_start` của điểm bắt đầu để đoạn đuôi
     bắt đầu từ t=0 (controller cần vậy). Tính trực tiếp trên `sec/nanosec` để
     **giữ core không dính rclcpp**.

---

## 8. Accessors — đọc trạng thái từ ngoài

Tất cả khóa `mutex_` (vì bị đọc từ thread khác — ví dụ `twistTick`).

| Hàm | Trả về | Ai dùng |
|---|---|---|
| `mode()` | mode hiện tại | `twistTick` kiểm tra có đang né không |
| `error()` | mã lỗi hiện tại | publish status |
| `cacheSize()` | số waypoint trong cache | chẩn đoán |
| `currentGoal()` | đích Cartesian mới nhất (`last_goal_`) | `twistTick` lấy đích để tính twist |

---

## 9. An toàn đa luồng (thread-safety)

- Một `std::mutex mutex_` bảo vệ **toàn bộ** state dùng chung (`mode_`,
  `cached_ee_positions_`, `current_waypoint_idx_`, `last_goal_`…).
- Các hàm **công khai** (`tick`, `setRunning`, `ingestTrajectory`, accessors)
  tự khóa mutex.
- Các hàm **phụ trợ** (`scanWindow`, `findSafeIndex`, `clearanceAt`, `runCheck`)
  **không** tự khóa — chúng giả định hàm gọi (`tick`) đã giữ khóa. Comment
  "`mutex_ held by caller`" đánh dấu điều này.
- Vì sao cần: node có **2 timer + 3 service** chạy trên các thread của executor,
  tất cả đụng tới cùng một `Supervisor`.

---

## 10. Bảng tóm tắt nhanh các thành phần

| Phần | Hàm chính | Mục đích một câu |
|---|---|---|
| Constructor | `Supervisor(Params)` | lưu cấu hình |
| Lifecycle | `setRunning` / `isRunning` / `reset` | bật/tắt/đưa về đầu |
| Nạp đường | `ingestTrajectory` | lọc + chia nhỏ + cache đường đi |
| Công cụ | `clearanceAt` / `scanWindow` / `findSafeIndex` | đo khoảng trống, quét cửa sổ, tìm điểm an toàn |
| Quyết định né | `runCheck` | xác định P1/P2 + kiểm tra điều kiện kích hoạt |
| FSM | `tick` | guard ưu tiên + `switch` theo state → ra quyết định mỗi nhịp |
| Resume | `buildResumeFrom` | cắt phần đuôi đường gốc, rebase thời gian |
| Đọc state | `mode`/`error`/`cacheSize`/`currentGoal` | cho node/vòng twist đọc |

---

## Xem thêm
- Thuật toán FSM 5 state + mã lỗi: [`include/custom_ur_aapf_2/mode.hpp`](../include/custom_ur_aapf_2/mode.hpp)
- Khai báo cấu trúc `Params` / `TickInput` / `TickOutput`: [`include/custom_ur_aapf_2/supervisor.hpp`](../include/custom_ur_aapf_2/supervisor.hpp)
- Hàm hình học/quỹ đạo thuần: [`include/custom_ur_aapf_2/trajectory_utils.hpp`](../include/custom_ur_aapf_2/trajectory_utils.hpp)
- Lớp ROS bọc bộ não này: [`src/aapf_supervisor_node.cpp`](../src/aapf_supervisor_node.cpp)

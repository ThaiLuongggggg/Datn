# Hướng dẫn cấu hình Điểm Khởi Đầu (Start) và Điểm Đích (Goal) Cố Định

Tài liệu này hướng dẫn bạn cách tùy chỉnh và thay đổi tọa độ điểm xuất phát và điểm đích đến cố định cho robot trong dự án **Nav2 A\* Global Path Planning Demo**.

---

## 1. Cơ chế hoạt động & Đánh dấu trực quan trên RViz2

- **Trực quan hóa nổi bật, dễ nhìn trên RViz2**:
  - **Điểm Khởi Đầu (START)**: Được đánh dấu bằng **MÀU XANH LÁ (Neon Green)** gồm:
    - Vòng tròn đế nổi bật trên sàn (đường kính 0.85m).
    - Khối cầu 3D tâm điểm tại tọa độ xuất phát.
    - Mũi tên 3D thẳng đứng màu xanh lá chỉ thẳng xuống vị trí.
    - Nhãn chữ nổi 3D: `START (X, Y)`.
  - **Điểm Đích Đến (GOAL)**: Được đánh dấu bằng **MÀU ĐỎ (Crimson Red)** gồm:
    - Vòng tròn đế nổi bật trên sàn (đường kính 0.85m).
    - Khối cầu 3D tâm điểm tại tọa độ đích đến.
    - Mũi tên 3D thẳng đứng màu đỏ chỉ thẳng xuống vị trí.
    - Nhãn chữ nổi 3D: `GOAL (X, Y)`.
  - **Đường đi A\* (`/plan`)**: Được vẽ bằng dải màu **XANH DƯƠNG SÁNG (Cyan)** nối từ điểm Xanh sang điểm Đỏ để tạo độ tương phản cao, tách biệt rõ ràng với 2 điểm đánh dấu.
- **Tự động lập đường đi (`auto_plan`)**: Ngay khi node `planner_server` sẵn sàng, thuật toán **A\*** sẽ tự động tính toán đường đi tối ưu và hiển thị ngay trên RViz2 mà bạn **không cần phải click chuột thủ công**.
- **Vẫn hỗ trợ click chuột tùy biến**: Bất cứ lúc nào bạn muốn thử vị trí mới, bạn chỉ cần dùng công cụ **"2D Pose Estimate"** (đổi điểm Xanh) hoặc **"2D Goal Pose"** (đổi điểm Đỏ) trên thanh công cụ RViz2. Marker và đường đi sẽ lập tức nhảy sang vị trí mới!

---

## 2. Cách 1: Thay đổi vĩnh viễn trong file `config/nav2_params.yaml` (Khuyên dùng)

Đây là cách thuận tiện nhất để lưu lại cấu hình cố định cho các lần chạy sau.

1. Mở file cấu hình bằng trình soạn thảo (VS Code, Nano, Gedit):
   ```bash
   code ~/Test/config/nav2_params.yaml
   # hoặc: nano ~/Test/config/nav2_params.yaml
   ```

2. Cuộn xuống cuối file, tìm đến mục **`goal_pose_bridge`**:
   ```yaml
   # ------------------------------------------------------------------------------
   # 5. CẤU HÌNH ĐIỂM XUẤT PHÁT (START) VÀ ĐÍCH ĐẾN (GOAL) CỐ ĐỊNH CHO ROBOT
   # ------------------------------------------------------------------------------
   goal_pose_bridge:
     ros__parameters:
       use_sim_time: false
       auto_plan: true             # Tự động tìm đường A* ngay khi khởi động
       publish_markers: true       # Hiển thị Marker Start (Xanh) và Goal (Đỏ) trong RViz2
       
       # --- Thay đổi tọa độ Điểm Xuất Phát (Start) tại đây ---
       start_x: -3.0               # Tọa độ X (đơn vị: mét)
       start_y: -3.0               # Tọa độ Y (đơn vị: mét)
       start_yaw: 0.0              # Hướng quay robot (radian)

       # --- Thay đổi tọa độ Điểm Đích Đến (Goal) tại đây ---
       goal_x: 3.0                 # Tọa độ X (đơn vị: mét)
       goal_y: 3.0                 # Tọa độ Y (đơn vị: mét)
       goal_yaw: 0.0               # Hướng quay đích đến (radian)
   ```

3. Lưu file và khởi chạy lại:
   ```bash
   cd ~/Test
   ./run_demo.sh
   ```

---

## 3. Cách 2: Thay đổi nhanh bằng đối số dòng lệnh (Command Line Arguments)

Nếu bạn muốn thử nhanh các tọa độ khác nhau mà không cần sửa file YAML:

### Chạy qua script `run_demo.sh`:
```bash
cd ~/Test
./run_demo.sh start_x:=-2.0 start_y:=-2.0 goal_x:=2.0 goal_y:=2.0
```

### Chạy qua lệnh `ros2 launch`:
```bash
ros2 launch nav2_astar_demo astar_demo.launch.py start_x:=-2.0 start_y:=-2.0 goal_x:=2.0 goal_y:=2.0
```

### Tắt chế độ tự động lập đường đi (để tự click chuột thủ công hoàn toàn):
```bash
./run_demo.sh auto_plan:=False
```

---

## 4. Cách 3: Chỉnh sửa giá trị mặc định trong `launch/astar_demo.launch.py`

Nếu bạn muốn thay đổi giá trị fallback mặc định trong mã nguồn launch:

Mở file [`launch/astar_demo.launch.py`](file:///home/luong/Test/launch/astar_demo.launch.py), tìm các dòng từ 80 đến 105:
```python
declare_start_x_cmd = DeclareLaunchArgument('start_x', default_value='-3.0', ...)
declare_start_y_cmd = DeclareLaunchArgument('start_y', default_value='-3.0', ...)
declare_goal_x_cmd  = DeclareLaunchArgument('goal_x',  default_value='3.0', ...)
declare_goal_y_cmd  = DeclareLaunchArgument('goal_y',  default_value='3.0', ...)
declare_auto_plan_cmd = DeclareLaunchArgument('auto_plan', default_value='True', ...)
```

Sau khi sửa, biên dịch lại package:
```bash
cd ~/Test
colcon build --symlink-install
```

---

## 5. Quy tắc hệ tọa độ & Bảng gợi ý tọa độ trên bản đồ mẫu (`sample_map`)

Bản đồ mẫu đi kèm dự án có thông số:
- **Kích thước**: `15.0m x 15.0m` (300 x 300 ô lưới, độ phân giải 0.05m/ô).
- **Gốc tọa độ `(0.0, 0.0)`**: Nằm ở **chính giữa sảnh trung tâm** của bản đồ (vùng trống không có vật cản).
- **Phạm vi tọa độ hợp lệ**:
  - Trục **X**: từ `-7.0` (biên trái) đến `+7.0` (biên phải).
  - Trục **Y**: từ `-7.0` (biên dưới) đến `+7.0` (biên trên).

### Bảng các cặp tọa độ đẹp để demo thuật toán A*:

| Kịch bản demo | Điểm bắt đầu (Start X, Y) | Điểm đích đến (Goal X, Y) | Mô tả đường đi A* |
|---|---|---|---|
| **Mặc định (Đường chéo lớn)** | `(-3.0, -3.0)` | `(3.0, 3.0)` | Đi xuyên qua 2 phòng, lách qua cửa hẹp và vòng qua các cột cản |
| **Dọc hành lang trái** | `(-3.0, -5.0)` | `(-3.0, 5.0)` | Đi từ góc dưới bên trái lên thẳng góc trên bên trái |
| **Băng ngang qua tâm sảnh** | `(-5.0, 0.0)` | `(5.0, 0.0)` | Đi từ phòng bên trái xuyên qua tâm `(0, 0)` sang phòng bên phải |
| **Góc dưới sang góc trên phải** | `(-5.0, -5.0)` | `(5.0, 5.0)` | Hành trình dài tối đa trên bản đồ với nhiều khúc ngoặt phức tạp |
| **Vòng quanh cột cản hẹp** | `(-1.5, -1.5)` | `(1.5, 1.5)` | Minh họa khả năng bo cua mượt quanh vùng thổi phồng (`inflation_layer`) |

> [!CAUTION]
> **Lưu ý tránh vật cản:**
> Nếu bạn đặt điểm Start hoặc Goal quá sát bức tường hoặc rơi trúng vào cột vật cản, `planner_server` sẽ báo lỗi:
> `"Không tìm thấy đường đi khả thi (bị chặn bởi vật cản hoặc ngoài bản đồ)"`.
> Hãy giữ điểm cách tường tối thiểu `0.45m` (bằng với bán kính an toàn `inflation_radius`).

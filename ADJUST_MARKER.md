# Hướng dẫn Điều Chỉnh Kích Thước & Hình Tròn Đánh Dấu (Start / Goal)

Tài liệu này giải thích chi tiết cơ chế hiển thị và hướng dẫn bạn tùy chỉnh kích cỡ, màu sắc, độ nổi của hai hình tròn đánh dấu **Điểm Đầu (Start)** và **Điểm Cuối (Goal)** trên giao diện RViz2.

---

## 1. Những Nâng Cấp Để Hình Tròn Hiển Thị Rõ Nét Trên RViz2

Trước đây hình tròn có thể khó nhìn hoặc không thấy rõ do:
1. **Lớp Costmap đè lên**: Hình tròn nằm sát sàn (`z = 0.02m`) nên bị lớp màu chi phí (`costmap`) đè lên gây mờ hoặc mất nét (hiện tượng z-fighting).
2. **Kém tương phản với nền**: Màu xanh lá đặt trực tiếp trên nền xám/trắng của bản đồ sẽ bị nhạt màu.
3. **Cấu hình topic RViz**: Trường cấu hình RViz đã được cập nhật chuẩn `Topic: /planning_markers`.

### Thiết kế cải tiến hiện tại:
- **Độ nổi 3D**: Hình tròn được nâng lên độ cao `z = 0.08m` với độ dày `0.08m`, giúp hình tròn **nằm nổi hoàn toàn lên trên** mặt sàn bản đồ, lớp Costmap và dải đường đi A*.
- **Viền tương phản (Double Rim)**:
  - 🟢 **Điểm đầu (START)**: Lõi hình tròn màu **Xanh lá tươi (Neon Green)** được bao quanh bởi một lớp viền màu **Xanh đậm** (đường kính lớn hơn 35%), giúp hình tròn nổi bật trên mọi nền (sàn trắng hay tường đen).
  - 🔴 **Điểm cuối (GOAL)**: Lõi hình tròn màu **Đỏ cờ (Crimson Red)** được bao quanh bởi một lớp viền màu **Đỏ đậm**, tạo độ tương phản cao.
- **Kích thước chuẩn**: Mặc định đường kính lõi là **`0.22m`** (bằng đúng **~1/3 chiều rộng vật cản cột 0.50m** trên bản đồ).

---

## 2. Cách 1: Chỉnh sửa kích thước trong `config/nav2_params.yaml` (Khuyên dùng)

Mở file cấu hình:
```bash
code ~/Test/config/nav2_params.yaml
# hoặc: nano ~/Test/config/nav2_params.yaml
```

Tìm đến mục **`goal_pose_bridge`** ở cuối file:

```yaml
goal_pose_bridge:
  ros__parameters:
    use_sim_time: false
    auto_plan: true
    publish_markers: true
    
    # Tọa độ điểm Start & Goal:
    start_x: -3.0
    start_y: -3.0
    goal_x: 3.0
    goal_y: 3.0

    # ==========================================================================
    # ĐIỀU CHỈNH KÍCH THƯỚC HÌNH TRÒN TẠI ĐÂY:
    # ==========================================================================
    marker_diameter: 0.22       # Đường kính lõi hình tròn (mét). Mặc định 0.22m (~1/3 vật cản)
    marker_height: 0.08         # Độ dày nổi khối của hình tròn (mét)
```

- Nếu muốn hình tròn **to hơn, dễ nhìn hơn**: Đổi thành `marker_diameter: 0.30` hoặc `0.35`
- Nếu muốn hình tròn **nhỏ hơn**: Đổi thành `marker_diameter: 0.18`

Lưu file và khởi chạy lại:
```bash
cd ~/Test
./run_demo.sh
```

---

## 3. Cách 2: Chỉnh kích thước tức thì bằng dòng lệnh

Bạn có thể truyền trực tiếp `marker_diameter:=...` khi gõ lệnh mà không cần sửa file YAML:

```bash
cd ~/Test
# Đổi đường kính thành 0.30m:
./run_demo.sh marker_diameter:=0.30

# Vừa đổi kích thước vừa đổi tọa độ:
./run_demo.sh marker_diameter:=0.25 start_x:=-2.0 start_y:=-2.0 goal_x:=2.0 goal_y:=2.0
```

---

## 4. Cách 3: Điều chỉnh màu sắc trong mã nguồn `scripts/goal_pose_bridge.py`

Mở file [`scripts/goal_pose_bridge.py`](file:///home/luong/Test/scripts/goal_pose_bridge.py), tìm đến hàm `_publish_markers()`:

### Màu Điểm Đầu (Start) - Xanh lá:
```python
m_start_core.color.r = 0.0   # Đỏ
m_start_core.color.g = 1.0   # Xanh lá
m_start_core.color.b = 0.0   # Xanh dương
m_start_core.color.a = 1.0   # Độ đậm
```

### Màu Điểm Cuối (Goal) - Đỏ:
```python
m_goal_core.color.r = 1.0    # Đỏ
m_goal_core.color.g = 0.0    # Xanh lá
m_goal_core.color.b = 0.0    # Xanh dương
m_goal_core.color.a = 1.0    # Độ đậm
```

---

## 5. Mẹo xem trên giao diện RViz2

Nếu bạn thấy màn hình RViz2 hơi mờ hoặc khó nhìn:
1. **Thu phóng (Zoom)**: Dùng con lăn chuột để zoom lại gần vị trí hai hình tròn.
2. **Ẩn/Hiện lớp Costmap**: Trong ô bên trái **Displays**, bạn có thể bỏ tích tạm thời vào mục `Global Costmap` để chỉ xem bản đồ tĩnh, hình tròn Xanh/Đỏ sẽ càng nổi rõ hơn nữa.
3. **Kiểm tra topic**: Đảm bảo mục `Start (Green) & Goal (Red) Circles` trong danh sách Displays đang được tích chọn (Enabled = True).

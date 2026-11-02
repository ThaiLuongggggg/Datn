# HƯỚNG DẪN CHI TIẾT PLUGIN SCSO PLANNER (SAND CAT SWARM OPTIMIZATION) TRUYỀN THỐNG TRONG ROS 2 NAV2

> **Tài liệu tham chiếu:** [`Howreasearch.md`](./Howreasearch.md) & [`BENCHMARK_COST_FUNCTION_GUIDE.md`](./BENCHMARK_COST_FUNCTION_GUIDE.md)  
> **Plugin ID:** `SCSO` (Plugin thứ 2 hoạt động song song sau `GridBased` - A*)  
> **Môi trường hoạt động:** Cùng bản đồ tĩnh `sample_map.yaml` và cùng Global Costmap của Nav2.

---

## 1. Tổng Quan Kiến Trúc & Vị Trí Plugin

Trong hệ thống kiểm thử `nav2_astar_demo`, node `planner_server` được nạp đồng thời **2 Plugin** để so sánh hiệu năng theo nguyên lý đã nêu trong [`Howreasearch.md`](./Howreasearch.md):

```
                               ┌────────────────────────────────────────────────────────┐
                               │                     planner_server                     │
                               │                                                        │
Action Request                 │  ┌──────────────────────┐    ┌──────────────────────┐  │
/compute_path_to_pose ────────►│  │ Plugin 1: GridBased  │    │ Plugin 2: SCSO       │  │
                               │  │ (A* NavfnPlanner)    │    │ (SCSO Planner)       │  │
                               │  └──────────────────────┘    └──────────────────────┘  │
                               └────────────────────────────────────────────────────────┘
```

1. **Plugin 1: `GridBased`** (Thuật toán cổ điển A* - `nav2_navfn_planner/NavfnPlanner`).
2. **Plugin 2: `SCSO`** (Thuật toán trí tuệ bầy đàn Mèo cát - `scso_planner_benchmark::ScsoPlanner`).

---

## 2. Loại Bỏ B-Spline & Khôi Phục SCSO Truyền Thống

### 2.1 Tại sao phải loại bỏ B-Spline?
* Đường cong B-Spline tạo ảo giác đường đi trơn mượt nhưng làm biến dạng các góc lái thực tế mà cá thể bầy đàn tìm ra.
* B-Spline không đảm bảo tính bao lồi khi các điểm điều khiển nằm sát vật cản (đường cong có thể phồng ra ngoài đâm vào tường).
* Trong bài báo khoa học về tối ưu hóa bầy đàn, việc đánh giá trực tiếp trên các **đoạn thẳng nối tiếp (Piece-wise Linear Segments)** giúp phản ánh chính xác 100% năng lực hội tụ và tính tối ưu của thuật toán bầy đàn.

### 2.2 Biểu diễn nghiệm & Giải mã quỹ đạo thuần túy
* **Điểm cố định:** $P_1 = P_{\text{start}} = (x_1, y_1)$ và $P_m = P_{\text{goal}} = (x_m, y_m)$.
* **Số điểm nút:** $m$ điểm nút (mặc định $m = 7$, tức $5$ đoạn thẳng).
* **Số chiều tìm kiếm:** $d = 2(m-2) = 10$ chiều. Mỗi cá thể mèo cát quản lý tọa độ thực tế $(x, y)$ của $m-2$ điểm trung gian:

$$\vec{X}_i = [x_2, y_2, x_3, y_3, \dots, x_{m-1}, y_{m-1}]$$

* Khi đánh giá Fitness hoặc xuất đường đi cho robot, ghép Start ở đầu và Goal ở cuối để tạo thành chuỗi $m$ điểm mốc:

$$\mathcal{P}_i = \{P_1, P_2, \dots, P_m\}$$

---

## 3. Đặc Tả Toán Học Thuật Toán Mèo Cát (SCSO Truyền Thống)

Thuật toán mô phỏng hai hoạt động cơ bản của mèo cát: **Thăm dò (Searching for prey)** và **Tấn công (Attacking prey)**.

```
                  ┌───────────────────────────────────────────────┐
                  │   r_G = S_M - (S_M * t) / t_max (Giảm từ 2->0)│
                  │   r = r_G * rand()                            │
                  │   R = 2 * r_G * rand() - r_G                  │
                  └───────────────────────┬───────────────────────┘
                                          │
                     ┌────────────────────┴────────────────────┐
                     ▼                                         ▼
           [ Khi |R| > 1.0 ]                         [ Khi |R| <= 1.0 ]
       THĂM DÒ TOÀN CỤC (SEARCHING)              KHAI THÁC CỤC BỘ (ATTACKING)
  X(t+1) = r * (X_b - rand * X_c)           X_md = |rand * X_b - X_c|
                                            X(t+1) = X_b - r * X_md * cos(θ)
```

### 3.1 Khởi tạo quần thể (Initialization)
Quần thể gồm $N$ cá thể mèo cát ($N = 50$). Mỗi cá thể có vector vị trí $\vec{X}_i \in \mathbb{R}^d$:
* Cá thể thứ $0$ được khởi tạo nội suy thẳng từ Start đến Goal.
* $N-1$ cá thể còn lại được phân bố ngẫu nhiên xung quanh đường nối với biên độ nhiễu kiểm soát, đảm bảo nằm trong giới hạn bản đồ $[x_{\min}, x_{\max}]$ và $[y_{\min}, y_{\max}]$.

### 3.2 Tần số thính giác suy giảm tuyến tính ($r_G$)
Đặc tính thính giác bắt tần số âm thanh thấp ($< 2\text{kHz}$) của mèo cát được biểu diễn bằng tham số $S_M = 2.0$:

$$r_G = S_M - \frac{S_M \cdot t}{t_{\max}} = 2.0 \cdot \left( 1 - \frac{t}{t_{\max}} \right)$$

* Đầu quá trình ($t = 0$): $r_G = 2.0$ (ưu tiên mở rộng vùng tìm kiếm).
* Cuối quá trình ($t \to t_{\max}$): $r_G \to 0.0$ (tập trung co cụm quanh con mồi tối ưu).

Độ nhạy của từng cá thể:

$$\vec{r} = \vec{r}_G \times \text{rand}(0, 1)$$

### 3.3 Hệ số chuyển pha thích nghi ($R$)
Cơ chế chuyển đổi giữa thăm dò toàn cục và khai thác cục bộ được quyết định bởi hệ số $R$:

$$R = 2 \times r_G \times \text{rand}(0, 1) - r_G \in [-r_G, r_G]$$

### 3.4 Quy luật cập nhật vị trí cá thể (Position Update)
Tại thế hệ $t$, gọi $\vec{X}_b(t)$ là vị trí con mồi tối ưu toàn đàn (Global Best) và $\vec{X}_c(t)$ là vị trí hiện tại của cá thể thứ $i$:

$$\vec{X}_i(t + 1) = \begin{cases}
r \cdot \left( \vec{X}_b(t) - \text{rand}() \cdot \vec{X}_c(t) \right) & \text{khi } |R| > 1 \quad \text{(Thăm dò toàn cục)} \\
\vec{X}_b(t) - r \cdot \vec{X}_{md} \cdot \cos(\theta) & \text{khi } |R| \le 1 \quad \text{(Khai thác / Tấn công)}
\end{cases}$$

Trong đó:
* $\vec{X}_{md} = \left| \text{rand}() \cdot \vec{X}_b(t) - \vec{X}_c(t) \right|$ biểu thị khoảng cách ngẫu nhiên tới con mồi.
* $\theta$ là góc tấn công ngẫu nhiên trong khoảng $[0, 2\pi]$ (hoặc $[0, 360^\circ]$).

---

## 4. Tích Hợp Hàm Chi Phí Không Thứ Nguyên & Beta Tự Thích Nghi

Toàn bộ quá trình tiến hóa hướng tới mục tiêu cực tiểu hóa hàm thích nghi chuẩn hóa:

$$\min F(\vec{X}_i) = w_1 J_1(\vec{X}_i) + w_2 J_2(\vec{X}_i) + w_3 J_3(\vec{X}_i)$$

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ 1. Chi phí độ dài chuẩn hóa (J1):                                                      │
│    J1 = (Tổng chiều dài các đoạn thẳng) / L_min           Miền giá trị: [1, +∞)        │
│                                                                                        │
│ 2. Chi phí độ mượt không dùng acos (J2):                                               │
│    J2 = (1 / (m - 2)) * Σ [ (1 - cos(Δθ_k)) / 2 ]          Miền giá trị: [0, 1]         │
│                                                                                        │
│ 3. Chi phí an toàn & Phạt va chạm nội suy (J3):                                        │
│    J3 = (1 / K_total) * Σ c_j                             Miền giá trị: [0, +∞)        │
│                                                                                        │
│    - Vùng cấm (d_j <= d_min):     c_j = β * [ 1 + (d_min - d_j) / d_min ]              │
│    - Vùng cảnh báo (d_min < d_j <= d_safe): c_j = [ ((d_safe - d_j)/(d_safe - d_min)) │
│                                                   * (d_min / d_j) ]^2                  │
│    - Vùng tự do (d_j > d_safe):   c_j = 0                                              │
│                                                                                        │
│ 4. Công thức Beta tự thích nghi:                                                       │
│    β = κ * K_total * [ (w1 / w3) * ((m - 1) * D_map / L_min) + (w2 / w3) + 1 ]         │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Cấu Trúc Mã Nguồn & Cấu Hình Đã Thiết Lập

### 5.1 Các file đã tạo và tích hợp trong Workspace:
* **Header:** [`scso_planner_benchmark/include/scso_planner_benchmark/scso_planner.hpp`](file:///home/luong/Test/scso_planner_benchmark/include/scso_planner_benchmark/scso_planner.hpp)
* **Source C++:** [`scso_planner_benchmark/src/scso_planner.cpp`](file:///home/luong/Test/scso_planner_benchmark/src/scso_planner.cpp)
* **Plugin Description:** [`scso_planner_benchmark/scso_planner_plugins.xml`](file:///home/luong/Test/scso_planner_benchmark/scso_planner_plugins.xml)
* **Cập nhật Build:** Đã đăng ký target `scso_planner_benchmark` vào [`CMakeLists.txt`](file:///home/luong/Test/CMakeLists.txt) và [`package.xml`](file:///home/luong/Test/package.xml).

### 5.2 Cấu hình trong `config/nav2_params.yaml`:
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    use_sim_time: false
    # Danh sách 2 plugin hoạt động đồng thời:
    planner_plugins: ["GridBased", "SCSO"]

    # PLUGIN 1: A* Truyền thống
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      use_astar: true
      tolerance: 0.5
      allow_unknown: true

    # PLUGIN 2: Sand Cat Swarm Optimization (SCSO)
    SCSO:
      plugin: "scso_planner_benchmark::ScsoPlanner"
      max_iterations: 150         # t_max
      population_size: 50         # N
      num_waypoints: 7            # m (Start + 5 Waypoint + Goal)
      samples_per_segment: 15     # N_s
      max_sensitivity_range: 2.0  # S_M
      seed: 0                     # 0: random mỗi lần chạy
      w1: 1.0                     # Trọng số độ dài
      w2: 1.5                     # Trọng số độ mượt
      w3: 1.0                     # Trọng số an toàn
      kappa: 1.5                  # Hệ số an toàn phạt vi phạm
      robot_radius: 0.22          # Bán kính robot
      safety_margin: 0.05         # Dung sai an toàn
```

---

## 6. Hướng Dẫn Chạy Kiểm Thử & So Sánh Hiệu Năng (Benchmark)

### Bước 1: Khởi động hệ thống Nav2 (Terminal 1)
```bash
cd /home/luong/Test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_astar_demo astar_demo.launch.py
```
Hệ thống sẽ nạp bản đồ tĩnh `sample_map.pgm`, khởi động `planner_server` với cả 2 plugin `GridBased` và `SCSO`.

---

### Bước 2: Chạy kiểm thử A* (Terminal 2)
```bash
cd /home/luong/Test
source /opt/ros/humble/setup.bash
python3 scripts/test_plan_client.py --planner GridBased --start "-2.0 -2.0" --goal "2.5 2.5"
```
*Kết quả:* A* sẽ tìm ra đường đi dạng lưới ô cờ 8 hướng (grid-based).

---

### Bước 3: Chạy kiểm thử SCSO Planner (Terminal 2)
```bash
cd /home/luong/Test
source /opt/ros/humble/setup.bash
python3 scripts/test_plan_client.py --planner SCSO --start "-2.0 -2.0" --goal "2.5 2.5"
```
*Kết quả:* Bầy mèo cát 50 cá thể sẽ tiến hóa qua 150 thế hệ, tìm ra đường đi tối ưu liên tục trong không gian thực và in log chi tiết các chỉ số:
```
[ScsoPlanner] Hoàn thành tối ưu: Best Fitness: 2.1450 | J1: 1.1200 | J2: 0.0840 | J3: 0.2100 | Feasible: TRUE | Beta: 1845.2
```

---

## 7. Bảng Chỉ Số Thu Thập Cho Báo Cáo Khoa Học

Để đưa vào bài báo hoặc báo cáo so sánh, thực hiện 20 lần chạy lặp lại cho mỗi thuật toán (theo quy chuẩn [`Howreasearch.md`](./Howreasearch.md)):

| Tiêu chí so sánh | A* Planner (`GridBased`) | SCSO Planner (`SCSO`) | Nhận xét khoa học |
| :--- | :---: | :---: | :--- |
| **Bản chất thuật toán** | Tìm kiếm rời rạc trên đồ thị ô lưới (Discrete Graph) | Tối ưu hóa bầy đàn liên tục (Continuous Metaheuristic) | SCSO tìm kiếm trong $\mathbb{R}^d$ không bị giới hạn 8 hướng. |
| **Chiều dài đường đi ($L$)** | Thường bị hiện tượng giật ziczac theo ô lưới | Ngắn hơn do đường đi chéo liên tục tự do | SCSO tiệm cận tốt hơn với đường thẳng tối ưu. |
| **Độ mượt ($\Theta_{\text{smooth}}$)** | Gập góc $45^\circ$ và $90^\circ$ thường xuyên | Độ cong trơn mượt hơn nhờ thành phần $J_2$ | Robot bám vết êm hơn, giảm rung lắc động cơ. |
| **Khoảng cách vật cản** | Thường đi sát mép ô phồng của vật cản | Được đẩy xa khỏi tường nhờ vùng đệm $1/D$ | $J_3$ tạo hành lang thông thoáng an toàn. |
| **Thời gian tính toán ($T_{\text{plan}}$)** | Nhanh, có tính xác định ($5 \sim 15\text{ms}$) | Chậm hơn do số vòng lặp bầy đàn ($50 \sim 150\text{ms}$) | Đánh đổi giữa tính tối ưu hình học và thời gian CPU. |

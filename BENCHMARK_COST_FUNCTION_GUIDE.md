# HƯỚNG DẪN THIẾT KẾ & PHÂN TÍCH TOÁN HỌC HÀM CHI PHÍ KHÔNG THỨ NGUYÊN (DIMENSIONLESS COST FUNCTION) CHO BENCHMARK CÁC THUẬT TOÁN DẪN ĐƯỜNG BẦY ĐÀN (SWARM INTELLIGENCE PATH PLANNING)

> **Tài liệu tham chiếu đi kèm:** [`Howreasearch.md`](./Howreasearch.md)  
> **Ứng dụng mục tiêu:** So sánh hiệu năng (Benchmarking) giữa các thuật toán tối ưu hóa bầy đàn / siêu phỏng sinh học (Metaheuristics / Swarm Intelligence như SCSO, PSO, GWO, WOA, ABC, GA, ACO,...) trong bài toán quy hoạch đường đi cho robot di động tự hành (AGV/AMR).

---

## MỤC LỤC
1. [Đặt vấn đề & Tại sao cần chuẩn hóa Không Thứ Nguyên (Dimensionless)?](#1-đặt-vấn-đề--tại-sao-cần-chuẩn-hóa-không-thứ-nguyên-dimensionless)
2. [Đặc tả Toán học Chi tiết của Hàm Chi phí Chuẩn hóa](#2-đặc-tả-toán-học-chi-tiết-của-hàm-chi-phí-chuẩn-hóa)
   - 2.1 [Biểu diễn nghiệm & Giải mã quỹ đạo (Solution Representation)](#21-biểu-diễn-nghiệm--giải-mã-quỹ-đạo-solution-representation)
   - 2.2 [Hàm chi phí độ dài chuẩn hóa ($J_1$)](#22-hàm-chi-phí-độ-dài-chuẩn-hóa-j_1)
   - 2.3 [Hàm chi phí độ mượt không lượng giác ngược ($J_2$)](#23-hàm-chi-phí-độ-mượt-không-lượng-giác-ngược-j_2)
   - 2.4 [Hàm chi phí an toàn & phạt va chạm không thứ nguyên ($J_3$)](#24-hàm-chi-phí-an-toàn--phạt-va-chạm-không-thứ-nguyên-j_3)
3. [Phân tích Chuyên sâu Cơ chế Phạt $\beta$ Thích nghi Tự động (Auto-Adaptive Penalty)](#3-phân-tích-chuyên-sâu-cơ-chế-phạt-beta-thích-nghi-tự-động-auto-adaptive-penalty)
   - 3.1 [Vấn đề chí tử của hệ số phạt cố định trong Metaheuristics](#31-vấn-đề-chí-tử-của-hệ-số-phạt-cố-định-trong-metaheuristics)
   - 3.2 [Chứng minh toán học cho điều kiện loại trừ nghiệm vi phạm](#32-chứng-minh-toán-học-cho-điều-kiện-loại-trừ-nghiệm-vi-phạm)
   - 3.3 [Công thức $\beta$ thích nghi tối ưu & Ý nghĩa thực tiễn](#33-công-thức-beta-thích-nghi-tối-ưu--ý-nghĩa-thực-tiễn)
4. [Phân tích Cảnh quan Tối ưu (Fitness Landscape) & Động lực học Bầy đàn](#4-phân-tích-cảnh-quan-tối-ưu-fitness-landscape--động-lực-học-bầy-đàn)
   - 4.1 [Tính liên tục & trơn mượt tại các đường biên chuyển tiếp](#41-tính-liên-tục--trơn-mượt-tại-các-đường-biên-chuyển-tiếp)
   - 4.2 [Gradient thoát hiểm trong lõi vật cản (Penetration Depth Gradient)](#42-gradient-thoát-hiểm-trong-lõi-vật-cản-penetration-depth-gradient)
   - 4.3 [Cân bằng Thăm dò (Exploration) và Khai thác (Exploitation)](#43-cân-bằng-thăm-dò-exploration-và-khai-thác-exploitation)
5. [Khung Thực nghiệm So sánh (Benchmarking Framework) chuẩn Khoa học](#5-khung-thực-nghiệm-so-sánh-benchmarking-framework-chuẩn-khoa-học)
   - 5.1 [Quy trình 20 lần chạy độc lập (Fair Benchmark)](#51-quy-trình-20-lần-chạy-độc-lập-fair-benchmark)
   - 5.2 [Bộ chỉ số đánh giá (Evaluation Metrics Suite)](#52-bộ-chỉ-số-đánh-giá-evaluation-metrics-suite)
6. [Mã Nguồn Cài Đặt Mẫu Tối Ưu (Python Vectorized & C++ Nav2)](#6-mã-nguồn-cài-đặt-mẫu-tối-ưu-python-vectorized--c-nav2)
   - 6.1 [Cài đặt Python tối ưu hóa với NumPy & Distance Transform](#61-cài-đặt-python-tối-ưu-hóa-với-numpy--distance-transform)
   - 6.2 [Cài đặt C++ hiệu năng cao tích hợp ROS 2 Costmap2D](#62-cài-đặt-c-hiệu-năng-cao-tích-hợp-ros-2-costmap2d)
7. [Bảng Cấu Hình Tham Số Khuyến Nghị](#7-bảng-cấu-hình-tham-số-khuyến-nghị)

---

## 1. Đặt vấn đề & Tại sao cần chuẩn hóa Không Thứ Nguyên (Dimensionless)?

Trong hầu hết các bài báo khoa học và công trình nghiên cứu về quy hoạch đường đi sử dụng thuật toán bầy đàn (Swarm Intelligence: PSO, GWO, SCSO, WOA,...), hàm mục tiêu thường được xây dựng dưới dạng tổng có trọng số:

$$F_{traditional} = w_1 \cdot \text{Length} + w_2 \cdot \text{Smoothness} + w_3 \cdot \text{Collision}$$

### Các khuyết tật nghiêm trọng của hàm chi phí truyền thống:
1. **Xung đột thứ nguyên (Dimensional Incompatibility):**
   - Độ dài tính bằng mét ($\text{m}$), giá trị thường dao động $5 \sim 50\text{m}$.
   - Độ gãy khúc tính bằng radian ($\text{rad}$) hoặc độ ($^\circ$), giá trị từ $0 \sim 10\text{rad}$.
   - Hàm thế đẩy vật cản thường dùng thế nhân tạo (APF) $1/d^2$ (đơn vị $\text{m}^{-2}$), khi robot đến gần vật cản ($d = 0.1\text{m}$), giá trị vọt lên $100$, nhưng khi ở bản đồ lớn tính bằng pixel/centimet thì độ lớn chênh lệch hàng triệu lần.
   - **Hệ quả:** Phép cộng trên vi phạm nguyên lý đồng nhất thứ nguyên trong vật lý và toán học. Bộ trọng số $w_1, w_2, w_3$ bị phụ thuộc hoàn toàn vào kích thước cụ thể của một bản đồ và hệ đơn vị đo. Khi chuyển sang bản đồ khác, thuật toán hoàn toàn mất kiểm soát.
2. **Hiện tượng "Thiên vị chiều dài" (Length-Bias) gây đâm xuyên tường:**
   - Khi quãng đường vòng tránh vật cản quá dài, giá trị phạt va chạm không đủ lớn để bù đắp, cá thể bầy đàn sẽ "chọn" đâm xuyên qua góc tường để giảm chiều dài tổng thể.
3. **Hiện tượng "Bức tường phạt vô hạn" (Death Penalty Trap):**
   - Nếu phạt cá thể đâm tường bằng giá trị vô cực ($F = \infty$), cá thể đó mất toàn bộ gradient thông tin về vị trí tương đối với vật cản, làm bầy đàn mất đa dạng và dẫn đến kẹt cục bộ (Premature Convergence).

### Giải pháp Chuẩn hóa Không Thứ Nguyên (Dimensionless Formulation):
Tất cả các thành phần hàm mục tiêu được chuẩn hóa về các tỉ số phi thứ nguyên (Dimensionless Ratios) với miền giá trị xác định rõ ràng:
- Chi phí chiều dài: $J_1 \in [1, +\infty)$ (tỉ số so với khoảng cách thẳng lý tưởng).
- Chi phí độ mượt: $J_2 \in [0, 1]$ (tỉ lệ gập góc chuẩn hóa).
- Chi phí an toàn: $c_j \in [0, 1]$ trong vùng cảnh báo và tăng tuyến tính theo độ sâu xâm nhập trong vùng cấm.
- Hệ số phạt $\beta$ được tính thích nghi tự động (Auto-adaptive) dựa trên đặc tính hình học bản đồ.

---

## 2. Đặc tả Toán học Chi tiết của Hàm Chi phí Chuẩn hóa

```
  Start (P1)
     ●───────────────○────────────────○───────────────●  Goal (Pm)
                    P2               P3
     │◄─── l1 ──────►│◄───── l2 ─────►│◄──── l3 ──────►│
     
     Tổng độ dài: L = l1 + l2 + l3
     Độ dài thẳng: Lmin = ||Pm - P1||
     Tỉ số không thứ nguyên: J1 = L / Lmin >= 1.0
```

### 2.1 Biểu diễn nghiệm & Giải mã quỹ đạo (Solution Representation)
- Điểm xuất phát: $P_{\text{start}} = (x_1, y_1)$
- Điểm đích: $P_{\text{goal}} = (x_m, y_m)$
- Mỗi cá thể bầy đàn (ví dụ: Mèo cát - Sand Cat trong SCSO, Hạt trong PSO, Sói xám trong GWO) quản lý vị trí của $m-2$ điểm nút trung gian (Waypoints).
- Vector trạng thái của cá thể thứ $i$ trong không gian tìm kiếm liên tục $\mathbb{R}^D$ (với số chiều $D = 2(m-2)$):

$$\vec{X}_i = [x_2, y_2, x_3, y_3, \dots, x_{m-1}, y_{m-1}]^T$$

- Quỹ đạo đầy đủ gồm $m$ điểm nút được giải mã bằng cách ghép cố định Start và Goal:

$$\mathcal{P}_i = \{P_1, P_2, \dots, P_m\}, \quad \text{trong đó } P_1 = P_{\text{start}}, \; P_m = P_{\text{goal}}$$

---

### 2.2 Hàm chi phí độ dài chuẩn hóa ($J_1$)
Gọi $L_{\min}$ là khoảng cách Euclidean đường chim bay giữa Start và Goal:

$$L_{\min} = \Vert P_m - P_1 \Vert = \sqrt{(x_m - x_1)^2 + (y_m - y_1)^2}$$

Độ dài của từng đoạn thẳng nối hai waypoint liên tiếp $P_k$ và $P_{k+1}$ ($k \in \{1, \dots, m-1\}$):

$$l_k = \Vert P_{k+1} - P_k \Vert = \sqrt{(x_{k+1} - x_k)^2 + (y_{k+1} - y_k)^2}$$

Hàm chi phí độ dài chuẩn hóa $J_1(\vec{X}_i)$ biểu thị độ dôi dư hình học:

$$J_1(\vec{X}_i) = \frac{\sum_{k=1}^{m-1} l_k}{L_{\min}}$$

- **Tính chất toán học:** $J_1(\vec{X}_i) \in [1, +\infty)$.
- **Ý nghĩa:** Khi quỹ đạo là một đường thẳng hoàn hảo nối Start và Goal, $J_1 = 1.0$. Khi quỹ đạo dài hơn 25% do phải đi vòng tránh vật cản, $J_1 = 1.25$. Trực quan, độc lập với đơn vị đo (mét, centimet hay pixel).

---

### 2.3 Hàm chi phí độ mượt không lượng giác ngược ($J_2$)

Tại mỗi điểm nút trung gian $P_{k+1}$ ($k \in \{1, \dots, m-2\}$), xét hai vector chuyển hướng liên tiếp:

$$\vec{u}_k = P_{k+1} - P_k = (x_{k+1} - x_k, \; y_{k+1} - y_k)$$

$$\vec{v}_k = P_{k+2} - P_{k+1} = (x_{k+2} - x_{k+1}, \; y_{k+2} - y_{k+1})$$

Cosine của góc chuyển hướng $\Delta\theta_k$ giữa hai vector:

$$\cos\Delta\theta_k = \frac{\vec{u}_k \cdot \vec{v}_k}{\Vert \vec{u}_k \Vert \Vert \vec{v}_k \Vert + \epsilon} = \frac{(x_{k+1}-x_k)(x_{k+2}-x_{k+1}) + (y_{k+1}-y_k)(y_{k+2}-y_{k+1})}{l_k \cdot l_{k+1} + \epsilon}$$

*(Trong đó $\epsilon = 10^{-9}$ để triệt tiêu lỗi chia cho 0 khi 2 waypoint ngẫu nhiên trùng nhau).*

Hàm độ mượt chuẩn hóa toàn tuyến:

$$J_2(\vec{X}_i) = \frac{1}{m-2} \sum_{k=1}^{m-2} \left( \frac{1 - \cos\Delta\theta_k}{2} \right)$$

```
  cos(Δθ) = 1  (Đi thẳng 0°)    ==> (1 - 1)/2 = 0.0  (Tối ưu tuyệt đối)
  cos(Δθ) = 0  (Cua vuông 90°)  ==> (1 - 0)/2 = 0.5
  cos(Δθ) = -1 (Quay đầu 180°)  ==> (1 - (-1))/2 = 1.0 (Tệ nhất)
```

- **Tính chất toán học:** $J_2(\vec{X}_i) \in [0, 1]$.
- **Đột phá về hiệu năng tính toán:**
  - Không cần gọi hàm lượng giác ngược `std::acos()` hay `math.acos()`. Hàm `acos` trong C++/Python tiêu tốn từ 50-100 chu kỳ CPU và tiềm ẩn rủi ro lỗi `NaN` khi sai số dấu phẩy động làm đối số vượt ra ngoài khoảng $[-1, 1]$.
  - Công thức $\frac{1 - \cos\Delta\theta}{2} = \sin^2\left(\frac{\Delta\theta}{2}\right)$ bảo toàn quan hệ đơn điệu: góc bẻ lái càng gắt thì chi phí tăng càng cao, đạo hàm theo tọa độ các nút cực kỳ trơn tru.

---

### 2.4 Hàm chi phí an toàn & phạt va chạm không thứ nguyên ($J_3$)

#### A. Kỹ thuật lấy mẫu nội suy tuyến tính (Continuous Interpolation Sampling)
Để ngăn chặn triệt để hiện tượng "xuyên tường" (hai waypoint $P_k$ và $P_{k+1}$ nằm ở vùng tự do nhưng đoạn thẳng nối chúng cắt qua chướng ngại vật), trên mỗi đoạn thẳng thứ $k$, ta lấy mẫu $N_s$ điểm nội suy đều:

$$S_{k, p} = P_k + \frac{p}{N_s} (P_{k+1} - P_k), \quad p \in \{0, 1, \dots, N_s - 1\}$$

Tổng số điểm kiểm tra trên toàn bộ quỹ đạo:

$$K_{\text{total}} = (m-1) \times N_s + 1$$

Tại mỗi điểm kiểm tra thứ $j \in \{1, \dots, K_{\text{total}}\}$, gọi $d_j$ là khoảng cách ngắn nhất từ điểm đó đến vật cản gần nhất (truy vấn từ Euclidean Distance Transform hoặc 2D Costmap).

```
   ●───────────x───────────x───────────x───────────●
  P_k      S_{k,1}     S_{k,2}     S_{k,3}       P_{k+1}
          (d_j tra cứu từ Euclidean Distance Map)
```

#### B. Phân vùng vật lý và Hàm phạt không thứ nguyên $c_j$
Thiết lập 2 ngưỡng vật lý:
- $d_{\min} = r_{\text{robot}} + \delta_{\text{margin}}$: Vùng cấm tuyệt đối (bán kính vật lý robot cộng dung sai điều khiển bám vết).
- $d_{\text{safe}}$: Ngưỡng kích hoạt cảnh báo an toàn ($d_{\text{safe}} > d_{\min}$, thường chọn $d_{\text{safe}} \approx 2.0 \sim 2.5 \times d_{\min}$).

Hàm chi phí thành phần tại điểm lấy mẫu thứ $j$:

$$c_j = \begin{cases} 
\beta \cdot \left( 1 + \frac{d_{\min} - d_j}{d_{\min}} \right) & \text{nếu } d_j \le d_{\min} \quad \text{(Vùng va chạm)} \\ 
\left( \frac{d_{\text{safe}} - d_j}{d_{\text{safe}} - d_{\min}} \cdot \frac{d_{\min}}{d_j} \right)^2 & \text{nếu } d_{\min} < d_j \le d_{\text{safe}} \quad \text{(Vùng cảnh báo tỉ lệ nghịch chuẩn hóa)} \\ 
0 & \text{nếu } d_j > d_{\text{safe}} \quad \text{(Vùng tự do)} 
\end{cases}$$

```
 Chi phí c_j
   ▲
2β ┤                * (Đâm xuyên sâu tới tâm vật cản d_j = 0)
   │               /
 β ┤              * (Chạm ranh giới cấm d_j = d_min)
   │              :
 1 ┤              * (Biên trong vùng cảnh báo d_j -> d_min+)
   │               \
   │                \___
 0 ┼───────────────┬────┴─────────────► Khoảng cách d_j
   0             d_min  d_safe
       [CẤM]       [CẢNH BÁO]  [TỰ DO]
```

Tổng hợp hàm chi phí an toàn toàn tuyến (lấy trung bình số điểm mẫu):

$$J_3(\vec{X}_i) = \frac{1}{K_{\text{total}}} \sum_{j=1}^{K_{\text{total}}} c_j$$

#### C. Ưu điểm toán học vượt trội của hàm $c_j$ chuẩn hóa:
1. **Tại biên ngoài $d_j = d_{\text{safe}}$:**
   $$c_j = \left( \frac{0}{d_{\text{safe}} - d_{\min}} \cdot \frac{d_{\min}}{d_{\text{safe}}} \right)^2 = 0$$
   Hàm liên tục cấp $C^1$ với vùng tự do (đạo hàm bậc nhất tiếp xúc bằng 0), không gây nhảy sốc thế năng đột ngột.
2. **Tại biên trong $d_j \to d_{\min}^+$:**
   $$c_j = \left( \frac{d_{\text{safe}} - d_{\min}}{d_{\text{safe}} - d_{\min}} \cdot \frac{d_{\min}}{d_{\min}} \right)^2 = (1 \cdot 1)^2 = 1.0$$
   Giá trị luôn đạt chuẩn hóa chính xác bằng $1.0$ bất kể đơn vị đo hay kích thước robot!
3. **Bảo toàn tính chất vật lý của APF:** Trong vùng $[d_{\min}, d_{\text{safe}}]$, đại lượng $\frac{d_{\min}}{d_j}$ tạo nên đường cong hyperbol tỉ lệ nghịch, độ dốc tăng mạnh khi áp sát mép tường, tạo lực đẩy tự nhiên xua các cá thể bầy đàn ra xa vùng nguy hiểm.
4. **Bất biến theo độ phân giải ($K_{\text{total}}$):** Việc chia trung bình cho $K_{\text{total}}$ giúp giá trị $J_3$ không bị phình to khi người dùng tăng độ mịn nội suy $N_s$.

---

## 3. Phân tích Chuyên sâu Cơ chế Phạt $\beta$ Thích nghi Tự động (Auto-Adaptive Penalty)

### 3.1 Vấn đề chí tử của hệ số phạt cố định trong Metaheuristics
Trong bài toán tối ưu có ràng buộc (Constrained Optimization), nếu chọn hệ số phạt va chạm $\beta$ dạng hằng số tĩnh (ví dụ $\beta = 1000$):
- **Nếu $\beta$ quá nhỏ:** Cá thể sẵn sàng "cắt góc" xuyên qua vật cản vì tổng hàm fitness khi đâm xuyên ngắn hơn việc đi vòng một đoạn đường dài hợp lệ.
- **Nếu $\beta$ quá lớn:** Cảnh quan fitness bị biến dạng thành "vực thẳm". Chỉ cần 1 điểm mẫu dính tường nhẹ, cá thể bị gán chi phí hàng triệu. Bầy đàn bị triệt tiêu đột ngột sự đa dạng sinh học (Diversity loss), dẫn đến hiện tượng kẹt bầy đàn tại cực tiểu địa phương sơ khai (Premature Stagnation).

---

### 3.2 Chứng minh toán học cho điều kiện loại trừ nghiệm vi phạm

Hàm Fitness tổng hợp của một cá thể là:

$$F(\vec{X}_i) = w_1 J_1(\vec{X}_i) + w_2 J_2(\vec{X}_i) + w_3 J_3(\vec{X}_i)$$

Ta đặt ra **Nguyên lý Tối thượng về Tính Hợp lệ của Nghiệm (Fundamental Feasibility Principle)**:
> *"Một quỹ đạo dù chỉ có duy nhất **1 điểm mẫu vi phạm vùng cấm** ($d_j \le d_{\min}$), thì chi phí Fitness của nó bắt buộc phải lớn hơn chi phí của đường đi hợp lệ tệ nhất có thể có trong toàn bộ không gian bản đồ."*

#### Bước 1: Ước lượng chặn trên của đường đi hợp lệ tệ nhất ($F_{\text{valid}}^{\max}$)
Xét một đường đi hoàn toàn hợp lệ (không có điểm nào chạm vùng cấm $d_j \le d_{\min}$):
- **Độ dài tệ nhất ($J_1^{\max}$):** Trong bản đồ có đường chéo $D_{\text{map}}$, đường đi gồm $m-1$ đoạn thẳng ziczac dài tối đa không vượt quá $(m-1) D_{\text{map}}$. Do đó:
  $$J_1^{\max} \le \frac{(m-1) D_{\text{map}}}{L_{\min}}$$
- **Độ mượt tệ nhất ($J_2^{\max}$):** Khi robot quay đầu $180^\circ$ tại mọi nút trung gian:
  $$J_2^{\max} = 1.0$$
- **Độ an toàn tệ nhất ($J_3^{\max}$):** Đường đi chạy hoàn toàn men sát mép vùng cấm ($d_j \to d_{\min}^+$ trên toàn tuyến):
  $$J_3^{\max} = \frac{1}{K_{\text{total}}} \sum_{j=1}^{K_{\text{total}}} 1.0 = 1.0$$

Chi phí của đường đi hợp lệ tệ nhất bị chặn trên bởi:

$$F_{\text{valid}}^{\max} = w_1 \frac{(m-1) D_{\text{map}}}{L_{\min}} + w_2 (1.0) + w_3 (1.0)$$

#### Bước 2: Thiết lập chi phí của đường đi vi phạm nhẹ nhất ($F_{\text{infeasible}}^{\min}$)
Xét một quỹ đạo đi gần như thẳng tắp ($J_1 \ge 1, J_2 \ge 0$), nhưng rủi ro có đúng **1 điểm mẫu duy nhất** vừa chạm mép vùng cấm ($d_j = d_{\min} \implies c_j = \beta$):

$$F_{\text{infeasible}}^{\min} \ge w_1 (1.0) + w_2 (0) + w_3 \left( \frac{\beta}{K_{\text{total}}} \right) > w_3 \frac{\beta}{K_{\text{total}}}$$

#### Bước 3: Giải bất đẳng thức điều kiện
Để $F_{\text{infeasible}}^{\min} > F_{\text{valid}}^{\max}$, ta cần:

$$w_3 \frac{\beta}{K_{\text{total}}} > w_1 \frac{(m-1) D_{\text{map}}}{L_{\min}} + w_2 + w_3$$

Chia hai vế cho $w_3$ và nhân với $K_{\text{total}}$:

$$\beta > K_{\text{total}} \cdot \left( \frac{w_1}{w_3} \cdot \frac{(m - 1) D_{\text{map}}}{L_{\min}} + \frac{w_2}{w_3} + 1 \right)$$

---

### 3.3 Công thức $\beta$ thích nghi tối ưu & Ý nghĩa thực tiễn

Đưa vào hệ số khuếch đại an toàn $\kappa \ge 1.2 \sim 1.5$ (Safety Margin Factor) để đảm bảo nghiệm vi phạm luôn nằm trên nghiệm hợp lệ với một biên cách biệt an toàn:

$$\beta = \kappa \cdot K_{\text{total}} \cdot \left( \frac{w_1}{w_3} \cdot \frac{(m - 1) D_{\text{map}}}{L_{\min}} + \frac{w_2}{w_3} + 1 \right)$$

```
                                  CÔNG THỨC CHỐT
  ┌────────────────────────────────────────────────────────────────────────────────────────┐
  │                                                                                        │
  │   β = κ · K_total · [ (w1 / w3) · ((m - 1) · D_map / L_min) + (w2 / w3) + 1 ]          │
  │                                                                                        │
  └────────────────────────────────────────────────────────────────────────────────────────┘
```

#### Những lợi ích vượt trội khi sử dụng công thức này:
1. **Tự thích nghi theo cấu hình bài toán (Zero Manual Tuning):** Khi đổi bản đồ từ nhỏ sang lớn ($D_{\text{map}}$ thay đổi), khi chọn điểm Start-Goal xa hay gần ($L_{\min}$ thay đổi), hoặc khi tăng số lượng waypoint $m$, giá trị $\beta$ tự động điều chỉnh chính xác. Người nghiên cứu không cần tốn hàng tuần để "dò tay" hệ số phạt.
2. **Cân bằng hoàn hảo giữa các thành phần:** Đối với mọi đường đi hợp lệ, $J_1 \in [1, 5]$, $J_2 \in [0, 1]$, $J_3 \in [0, 1]$. Cả ba thành phần đều có cùng độ lớn bậc đơn vị, giúp bộ trọng số $w_1, w_2, w_3$ phản ánh trung thực ý đồ ưu tiên của người thiết kế.
3. **Đảm bảo tính hội tụ an toàn 100%:** Thuật toán bầy đàn tuyệt đối không bao giờ chọn một đường đi cắt góc xuyên qua tường khi nghiệm dừng được tìm thấy.

---

## 4. Phân tích Cảnh quan Tối ưu (Fitness Landscape) & Động lực học Bầy đàn

```
   Không gian tìm kiếm: R^{2(m-2)}
   
         Cực tiểu địa phương
               ▼
   Fitness   (Bẫy vật cản)           Nghiệm tối ưu toàn cục (Global Min)
      ▲         _                           _
      │        / \                         / \
      │       /   \                       /   \
      │  ____/     \_____________________/     \________
      │
      └─────────────────────────────────────────────────► Tọa độ các Waypoint
```

### 4.1 Tính liên tục & trơn mượt tại các đường biên chuyển tiếp
Hàm chi phí $c_j$ được chia làm 3 vùng nhưng liên tục tại các điểm tiếp giáp:
- Tại $d_j = d_{\text{safe}}$: $c_j = 0$, đạo hàm $\frac{\partial c_j}{\partial d_j} = 0$. Không xảy ra hiện tượng bước nhảy bậc thang (Discontinuity) gây giật vector vận tốc của các cá thể bầy đàn.
- Tại $d_j = d_{\min}$: Biên cảnh báo đạt giá trị $1.0$, trong khi vùng cấm bắt đầu từ $\beta \ge 1.0$.

### 4.2 Gradient thoát hiểm trong lõi vật cản (Penetration Depth Gradient)
Xét một cá thể có waypoint nằm lọt vào sâu bên trong vật cản:
- Nếu dùng phương pháp phạt cổ điển (gán hằng số phạt cố định $\beta$): Mọi vị trí bên trong vật cản đều có cùng chi phí. Đạo hàm bằng 0 ($\nabla F = 0$). Cá thể bị "mù phương hướng" hoàn toàn, việc thoát khỏi vật cản chỉ phụ thuộc vào đột biến ngẫu nhiên (Random Walk).
- Với công thức cải tiến: $c_j = \beta \left( 1 + \frac{d_{\min} - d_j}{d_{\min}} \right)$.
  - Càng đi sâu vào tâm vật cản ($d_j \to 0$ hoặc $d_j < 0$), chi phí càng tăng cao.
  - Càng tiến ra gần mép vỏ vật cản ($d_j \to d_{\min}$), chi phí giảm dần về $\beta$.
  - **Tác dụng:** Cung cấp một vector gradient hướng tâm đẩy rõ rệt hướng từ trong lõi vật cản ra ngoài không gian tự do, giúp các cá thể bầy đàn nhanh chóng tìm được đường thoát hiểm chỉ sau vài thế hệ ban đầu!

### 4.3 Cân bằng Thăm dò (Exploration) và Khai thác (Exploitation)
- **Giai đoạn đầu (Exploration):** Nhờ cơ chế phạt liên tục và gradient thoát hiểm, bầy đàn tự do phân tán khám phá toàn bộ không gian bản đồ mà không lo bị kẹt chết trong các hố vô cực.
- **Giai đoạn sau (Exploitation):** Vùng cảnh báo tỉ lệ nghịch $1/d_j$ đóng vai trò như đệm khí lực đẩy ép quỹ đạo co lại gần vật cản ở mức vừa đủ để tối ưu chiều dài $J_1$, nhưng vẫn giữ khoảng cách an toàn $d > d_{\min}$.

---

## 5. Khung Thực nghiệm So sánh (Benchmarking Framework) chuẩn Khoa học

Khi sử dụng hàm chi phí này để so sánh giữa nhiều thuật toán bầy đàn khác nhau (ví dụ: Sand Cat Swarm Optimization - **SCSO**, Particle Swarm Optimization - **PSO**, Grey Wolf Optimizer - **GWO**, Whale Optimization Algorithm - **WOA**, Genetic Algorithm - **GA**), cần tuân thủ nghiêm ngặt phương pháp luận khoa học đã nêu tại [`Howreasearch.md`](./Howreasearch.md).

```
   ┌────────────────────────────────────────────────────────────────────────┐
   │                  BỘ THUẬT TOÁN BẦY ĐÀN SO SÁNH                         │
   │      [ SCSO ]        [ PSO ]        [ GWO ]        [ WOA ]             │
   └──────────┬──────────────┬──────────────┬──────────────┬────────────────┘
              ▼              ▼              ▼              ▼
   ┌────────────────────────────────────────────────────────────────────────┐
   │                  HÀM CHI PHÍ KHÔNG THỨ NGUYÊN DUY NHẤT                 │
   │               F(X) = w1*J1(X) + w2*J2(X) + w3*J3(X)                    │
   │                (Cùng bản đồ, Cùng β thích nghi, Cùng m)                │
   └──────────────────────────────────┬─────────────────────────────────────┘
                                      ▼
   ┌────────────────────────────────────────────────────────────────────────┐
   │         TIẾN TRÌNH THỰC NGHIỆM ĐỘC LẬP 20 LẦN (N = 20 RUNS)            │
   │               Warm-up -> 20 Runs -> Excel / Boxplot                    │
   └────────────────────────────────────────────────────────────────────────┘
```

### 5.1 Quy trình 20 lần chạy độc lập (Fair Benchmark)
Các thuật toán bầy đàn mang tính chất ngẫu nhiên ngẫu kế (Stochastic Nature). Do đó:
1. **Cố định cấu hình chung:**
   - Kích thước quần thể: $N_{\text{pop}} = 50$ (hoặc $30, 100$).
   - Số vòng lặp tối đa: $T_{\max} = 200$ thế hệ.
   - Số lượng điểm nút: $m = 7$ (tức 5 đoạn thẳng, $D = 2(7-2) = 10$ chiều tìm kiếm).
   - Bản đồ tĩnh và cặp Start-Goal cố định tuyệt đối.
2. **Chạy lặp 20 lần độc lập (Independent Runs):** Sử dụng các seed ngẫu nhiên khác nhau cho mỗi lần chạy để đảm bảo tính khách quan thống kê.
3. **Loại bỏ hiệu ứng Warm-up:** Lần chạy đầu tiên được bỏ qua hoặc làm nóng bộ đệm RAM/CPU trước khi ghi nhận dữ liệu đo.

### 5.2 Bộ chỉ số đánh giá (Evaluation Metrics Suite)

Bên cạnh thời gian tính toán và độ dài, nghiên cứu bầy đàn cần tổng hợp bảng chỉ số đa chiều sau:

| Chỉ số đánh giá | Ký hiệu | Ý nghĩa khoa học |
| :--- | :---: | :--- |
| **Giá trị Fitness Tốt nhất** | $F_{\text{best}}$ | Đánh giá khả năng tiệm cận cực trị toàn cục ($\text{Mean} \pm \text{Std}$). |
| **Tỷ lệ Tìm thấy Quỹ đạo Hợp lệ** | $SR$ (%) | Phần trăm số lần chạy bầy đàn tìm ra đường không chạm tường ($J_3 < 1.0$). |
| **Chiều dài Thực tế** | $L$ ($\text{m}$) | Tổng khoảng cách Euclidean thực tế của quỹ đạo tốt nhất. |
| **Độ mượt Toàn tuyến** | $\Theta_{\text{smooth}}$ ($\text{rad}$) | Tổng góc bẻ lái tích lũy (kiểm chứng robot thật có chạy êm không). |
| **Khoảng cách Tĩnh Tối thiểu** | $D_{\text{clearance}}^{\min}$ ($\text{m}$) | Điểm sát vật cản nhất trên toàn bộ quỹ đạo (phải $> d_{\min}$). |
| **Thời gian Thực thi Mỗi Lần chạy** | $T_{\text{run}}$ ($\text{ms}$) | Đo lường độ phức tạp tính toán thực tế trên CPU. |
| **Thế hệ Hội tụ Sớm nhất** | $G_{\text{converge}}$ | Vòng lặp mà tại đó bầy đàn bắt đầu ổn định nghiệm (đo tốc độ hội tụ). |

---

## 6. Mã Nguồn Cài Đặt Mẫu Tối Ưu (Python Vectorized & C++ Nav2)

### 6.1 Cài đặt Python tối ưu hóa với NumPy & Distance Transform
*Đoạn mã dưới đây sử dụng kỹ thuật Vectorization để đánh giá Fitness toàn bộ quần thể bầy đàn cùng lúc, kết hợp Euclidean Distance Transform (`scipy.ndimage.distance_transform_edt`) cho tốc độ tra cứu khoảng cách $O(1)$.*

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: dimensionless_cost_evaluator.py
Đặc tả toán học hàm chi phí không thứ nguyên cho thuật toán bầy đàn (SCSO, PSO, GWO,...)
"""

import numpy as np
from scipy.ndimage import distance_transform_edt

class DimensionlessCostEvaluator:
    def __init__(self, binary_map, resolution, start_pos, goal_pos, 
                 num_waypoints=7, samples_per_segment=15, 
                 robot_radius=0.22, safety_margin=0.05,
                 w1=1.0, w2=1.5, w3=1.0, kappa=1.5):
        """
        Khởi tạo bộ đánh giá hàm chi phí không thứ nguyên.
        :param binary_map: Mảng 2D NumPy (1: Chướng ngại vật, 0: Tự do)
        :param resolution: Độ phân giải bản đồ (m/cell)
        :param start_pos: Tọa độ bắt đầu (x, y) theo mét
        :param goal_pos: Tọa độ đích (x, y) theo mét
        """
        self.resolution = resolution
        self.start = np.array(start_pos, dtype=np.float64)
        self.goal = np.array(goal_pos, dtype=np.float64)
        self.m = num_waypoints
        self.Ns = samples_per_segment
        self.K_total = (self.m - 1) * self.Ns + 1
        
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.kappa = kappa
        
        # Thiết lập các ngưỡng an toàn
        self.d_min = robot_radius + safety_margin
        self.d_safe = 2.5 * self.d_min
        
        # 1. Tiền xử lý Euclidean Distance Transform (EDT)
        free_space = (binary_map == 0).astype(np.uint8)
        self.dist_map_meters = distance_transform_edt(free_space) * self.resolution
        self.map_h, self.map_w = binary_map.shape
        
        # 2. Đặc tính hình học bản đồ
        self.L_min = float(np.linalg.norm(self.goal - self.start))
        self.D_map = float(np.sqrt((self.map_w * self.resolution)**2 + 
                                   (self.map_h * self.resolution)**2))
        
        # 3. Tính toán trước hệ số phạt thích nghi beta
        worst_length_ratio = ((self.m - 1) * self.D_map) / max(self.L_min, 1e-6)
        bracket_term = (self.w1 / self.w3) * worst_length_ratio + (self.w2 / self.w3) + 1.0
        self.beta = self.kappa * self.K_total * bracket_term
        
        print(f"[Init] L_min: {self.L_min:.2f}m | D_map: {self.D_map:.2f}m | K_total: {self.K_total} | Adaptive Beta: {self.beta:.2f}")

    def decode_path(self, X_vector):
        """
        Ghép cố định Start và Goal vào vector nghiệm để tạo thành quỹ đạo đầy đủ.
        :param X_vector: Mảng 1D độ dài 2*(m-2)
        :return: Mảng 2D (m, 2)
        """
        intermediate = X_vector.reshape((self.m - 2, 2))
        return np.vstack([self.start, intermediate, self.goal])

    def query_distance(self, points):
        """
        Tra cứu khoảng cách tới vật cản gần nhất cho một danh sách điểm (mét).
        :param points: Mảng (N, 2) tọa độ thế giới (x, y)
        """
        grid_x = np.clip(np.round(points[:, 0] / self.resolution).astype(int), 0, self.map_w - 1)
        grid_y = np.clip(np.round(points[:, 1] / self.resolution).astype(int), 0, self.map_h - 1)
        return self.dist_map_meters[grid_y, grid_x]

    def evaluate(self, X_vector):
        """
        Tính toán giá trị Fitness không thứ nguyên F = w1*J1 + w2*J2 + w3*J3
        """
        P = self.decode_path(X_vector)
        
        # --- THÀNH PHẦN J1: ĐỘ DÀI CHUẨN HÓA ---
        diffs = P[1:] - P[:-1]  # Vector từng đoạn (m-1, 2)
        segment_lengths = np.linalg.norm(diffs, axis=1)  # (m-1,)
        total_length = np.sum(segment_lengths)
        J1 = total_length / max(self.L_min, 1e-9)

        # --- THÀNH PHẦN J2: ĐỘ MƯỢT CHUẨN HÓA ---
        # Vector u_k = P[k+1] - P[k], v_k = P[k+2] - P[k+1]
        u = diffs[:-1]  # (m-2, 2)
        v = diffs[1:]   # (m-2, 2)
        dot_uv = np.sum(u * v, axis=1)
        norm_uv = segment_lengths[:-1] * segment_lengths[1:] + 1e-9
        cos_theta = np.clip(dot_uv / norm_uv, -1.0, 1.0)
        J2 = np.mean((1.0 - cos_theta) / 2.0)

        # --- THÀNH PHẦN J3: AN TOÀN & PHẠT VA CHẠM NỘI SUY ---
        # Nội suy Ns điểm trên mỗi đoạn
        t = np.linspace(0.0, 1.0, self.Ns, endpoint=False)[:, None, None]  # (Ns, 1, 1)
        # Điểm bắt đầu từng đoạn P[:-1], điểm kết thúc P[1:]
        interp_points = P[:-1][None, :, :] + t * diffs[None, :, :]  # (Ns, m-1, 2)
        all_sampled_points = interp_points.reshape(-1, 2)
        # Thêm điểm đích cuối cùng
        all_sampled_points = np.vstack([all_sampled_points, self.goal])

        d_vals = self.query_distance(all_sampled_points)

        # Phân 3 vùng:
        # Vùng 1: Va chạm (d <= d_min)
        mask_collision = (d_vals <= self.d_min)
        # Vùng 2: Cảnh báo (d_min < d <= d_safe)
        mask_warning = (d_vals > self.d_min) & (d_vals <= self.d_safe)
        # Vùng 3: Tự do (d > d_safe) -> chi phí 0

        c_vals = np.zeros_like(d_vals)
        
        # Tính c_j cho vùng va chạm
        c_vals[mask_collision] = self.beta * (1.0 + (self.d_min - d_vals[mask_collision]) / self.d_min)
        
        # Tính c_j cho vùng cảnh báo
        if np.any(mask_warning):
            dw = d_vals[mask_warning]
            term1 = (self.d_safe - dw) / (self.d_safe - self.d_min)
            term2 = self.d_min / dw
            c_vals[mask_warning] = (term1 * term2) ** 2

        J3 = np.mean(c_vals)

        # Fitness tổng hợp
        Fitness = self.w1 * J1 + self.w2 * J2 + self.w3 * J3
        
        return Fitness, {
            "Fitness": Fitness,
            "J1_LengthRatio": J1,
            "J2_Smoothness": J2,
            "J3_Safety": J3,
            "PathLength_m": total_length,
            "IsFeasible": not np.any(mask_collision)
        }
```

---

### 6.2 Cài đặt C++ hiệu năng cao tích hợp ROS 2 Costmap2D
*Đoạn mã C++ được thiết kế tối ưu hóa bộ nhớ đệm CPU, không cấp phát heap trong vòng lặp lặp của thuật toán bầy đàn, dễ dàng nhúng vào Nav2 `GlobalPlanner`.*

```cpp
#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

class DimensionlessCostEvaluator
{
public:
  DimensionlessCostEvaluator(
    nav2_costmap_2d::Costmap2D* costmap,
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & goal,
    int num_waypoints = 7,
    int samples_per_segment = 15,
    double robot_radius = 0.22,
    double safety_margin = 0.05,
    double w1 = 1.0, double w2 = 1.5, double w3 = 1.0,
    double kappa = 1.5)
  : costmap_(costmap), start_(start), goal_(goal),
    m_(num_waypoints), Ns_(samples_per_segment),
    w1_(w1), w2_(w2), w3_(w3), kappa_(kappa)
  {
    K_total_ = (m_ - 1) * Ns_ + 1;
    resolution_ = costmap_->getResolution();
    d_min_ = robot_radius + safety_margin;
    d_safe_ = 2.5 * d_min_;

    // Tính L_min
    double dx = goal_.x - start_.x;
    double dy = goal_.y - start_.y;
    L_min_ = std::hypot(dx, dy);

    // Tính đường chéo bản đồ D_map
    double map_w_m = costmap_->getSizeInMetersX();
    double map_h_m = costmap_->getSizeInMetersY();
    D_map_ = std::hypot(map_w_m, map_h_m);

    // Tính hệ số phạt thích nghi Beta
    double worst_length_ratio = ((m_ - 1) * D_map_) / std::max(L_min_, 1e-6);
    double bracket = (w1_ / w3_) * worst_length_ratio + (w2_ / w3_) + 1.0;
    beta_ = kappa_ * K_total_ * bracket;
  }

  // Đánh giá Fitness cho một cá thể
  // X chứa 2 * (m - 2) giá trị tọa độ x2, y2, ..., xm-1, ym-1
  double evaluate(const std::vector<double> & X, bool & is_feasible)
  {
    is_feasible = true;

    // 1. Giải mã quỹ đạo m điểm
    std::vector<double> px(m_), py(m_);
    px[0] = start_.x; py[0] = start_.y;
    for (int i = 0; i < m_ - 2; ++i) {
      px[i + 1] = X[2 * i];
      py[i + 1] = X[2 * i + 1];
    }
    px[m_ - 1] = goal_.x; py[m_ - 1] = goal_.y;

    // 2. Tính J1 (Độ dài chuẩn hóa) & lưu vector từng đoạn
    std::vector<double> l(m_ - 1);
    std::vector<double> ux(m_ - 1), uy(m_ - 1);
    double total_length = 0.0;

    for (int k = 0; k < m_ - 1; ++k) {
      ux[k] = px[k + 1] - px[k];
      uy[k] = py[k + 1] - py[k];
      l[k] = std::hypot(ux[k], uy[k]);
      total_length += l[k];
    }
    double J1 = total_length / std::max(L_min_, 1e-9);

    // 3. Tính J2 (Độ mượt chuẩn hóa) - Tuyệt đối không dùng acos()
    double sum_smooth = 0.0;
    for (int k = 0; k < m_ - 2; ++k) {
      double dot = ux[k] * ux[k + 1] + uy[k] * uy[k + 1];
      double denom = l[k] * l[k + 1] + 1e-9;
      double cos_theta = std::clamp(dot / denom, -1.0, 1.0);
      sum_smooth += (1.0 - cos_theta) * 0.5;
    }
    double J2 = sum_smooth / static_cast<double>(m_ - 2);

    // 4. Tính J3 (An toàn & Phạt va chạm nội suy)
    double sum_c = 0.0;
    for (int k = 0; k < m_ - 1; ++k) {
      for (int p = 0; p < Ns_; ++p) {
        double factor = static_cast<double>(p) / Ns_;
        double sx = px[k] + factor * ux[k];
        double sy = py[k] + factor * uy[k];
        double c_j = computePenaltyAtPoint(sx, sy, is_feasible);
        sum_c += c_j;
      }
    }
    // Điểm đích cuối cùng
    sum_c += computePenaltyAtPoint(goal_.x, goal_.y, is_feasible);
    double J3 = sum_c / static_cast<double>(K_total_);

    return w1_ * J1 + w2_ * J2 + w3_ * J3;
  }

  double getAdaptiveBeta() const { return beta_; }

private:
  inline double computePenaltyAtPoint(double wx, double wy, bool & is_feasible)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      // Vượt ra ngoài biên bản đồ -> Phạt va chạm nặng
      is_feasible = false;
      return beta_ * 2.0;
    }

    unsigned char cost = costmap_->getCost(mx, my);
    // Nếu ô là LETHAL_OBSTACLE hoặc INSCRIBED_INFLATED
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      is_feasible = false;
      return beta_;
    }

    // Ước lượng khoảng cách từ ô chi phí của Costmap
    // Trong Nav2: cost = 253 * exp(-cost_scaling_factor * (d - d_min))
    // Hoặc sử dụng mảng Distance Map tra cứu O(1)
    double d_j = estimateDistance(cost);

    if (d_j <= d_min_) {
      is_feasible = false;
      return beta_ * (1.0 + (d_min_ - d_j) / d_min_);
    } else if (d_j <= d_safe_) {
      double t1 = (d_safe_ - d_j) / (d_safe_ - d_min_);
      double t2 = d_min_ / d_j;
      double ratio = t1 * t2;
      return ratio * ratio;
    }
    return 0.0; // Vùng tự do
  }

  inline double estimateDistance(unsigned char cost)
  {
    if (cost == 0) return d_safe_ + 1.0;
    // Chuyển đổi gần đúng chi phí costmap 0-254 thành khoảng cách vật lý mét
    double factor = 1.0 - (static_cast<double>(cost) / 254.0);
    return d_min_ + factor * (d_safe_ - d_min_);
  }

  nav2_costmap_2d::Costmap2D* costmap_;
  geometry_msgs::msg::Point start_, goal_;
  int m_, Ns_, K_total_;
  double resolution_, d_min_, d_safe_;
  double L_min_, D_map_, beta_;
  double w1_, w2_, w3_, kappa_;
};
```

---

## 7. Bảng Cấu Hình Tham Số Khuyến Nghị

Để đảm bảo kết quả thực nghiệm đạt độ tin cậy khoa học cao và có thể xuất bản bài báo quốc tế, dưới đây là bộ tham số tiêu chuẩn:

| Tham số | Ký hiệu | Giá trị Khuyên dùng | Ý nghĩa Vật lý & Tác dụng Tối ưu |
| :--- | :---: | :---: | :--- |
| **Số lượng Waypoint** | $m$ | $6 \sim 10$ | Số điểm nút cấu thành đường đi. $m = 7$ tạo ra không gian tìm kiếm 10 chiều ($D = 10$), rất cân bằng cho các thuật toán bầy đàn. |
| **Số mẫu nội suy / đoạn** | $N_s$ | $15 \sim 25$ | Đảm bảo khoảng cách lấy mẫu $\Delta s = \frac{l_k}{N_s} \le \text{Resolution}$ bản đồ, triệt tiêu lỗi xuyên tường. |
| **Trọng số Chiều dài** | $w_1$ | $1.0$ | Hệ số nền tảng chuẩn hóa ($J_1 \ge 1.0$). |
| **Trọng số Độ mượt** | $w_2$ | $1.5$ | Ưu tiên đường cong trơn tru, hạn chế đổi hướng đột ngột giúp bộ điều khiển bám vết (DWB/TEB) không bị khựng vận tốc. |
| **Trọng số An toàn** | $w_3$ | $1.0$ | Giữ quỹ đạo chạy cách xa mép tường trong vùng cảnh báo. |
| **Hệ số Dự phòng Phạt** | $\kappa$ | $1.5$ | Đảm bảo mọi nghiệm vi phạm tường có chi phí lớn gấp ít nhất $1.5$ lần đường hợp lệ tệ nhất. |
| **Bán kính Vùng cấm** | $d_{\min}$ | $r_{\text{robot}} + 0.05\text{m}$ | $0.22\text{m} + 0.05\text{m} = 0.27\text{m}$ (bán kính vật lý + sai số bám vết quỹ đạo). |
| **Bán kính Cảnh báo** | $d_{\text{safe}}$ | $2.5 \times d_{\min}$ | Vùng chuyển tiếp lực đẩy hyperbol ($0.65 \sim 0.70\text{m}$). |

---

## TỔNG KẾT
Hàm chi phí không thứ nguyên trên giải quyết triệt để 3 nút thắt lớn nhất của việc ứng dụng các thuật toán bầy đàn vào bài toán lập kế hoạch đường đi:
1. **Loại bỏ sự phụ thuộc vào kích thước bản đồ và đơn vị đo** (chuẩn hóa toàn bộ về $[0, 1]$ và $[1, \infty)$).
2. **Loại bỏ công đoạn dò tay hệ số phạt $\beta$** nhờ công thức giải tích thích nghi tự động có chứng minh toán học.
3. **Cung cấp gradient thông tin liên tục** giúp bầy đàn thoát khỏi lõi vật cản và trơn hóa góc rẽ mà không cần các hàm lượng giác đắt đỏ.

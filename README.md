# XE TỰ CÂN BẰNG 2 BÁNH — STM32F407VET6

Firmware điều khiển xe tự cân bằng hai bánh: **STM32F407VET6** + **MPU6050** (IMU) + **TB6612** (motor driver) + **2 encoder** JGA25-370.

Mục tiêu: xe tự giữ thăng bằng tại chỗ khi được giữ thẳng đứng trên hai bánh.

---

## Các lỗi đã sửa (khiến xe không cân bằng)

| # | Lỗi | Hậu quả | Đã sửa |
|---|---|---|---|
| 1 | **Dấu gyro ngược với góc accel** trong complementary filter (`angle += gx*dt` nhưng `angle = atan2(-Ay, Az)` có đạo hàm `-gx`) | Góc bị sai khi xe đang nghiêng, term D thành **positive feedback** → xe dao động/ngã càng nhanh | `mpu6050.c`: `gyro_x_dps = -gx_dps`, filter dùng `gyro_x_dps`; `gyro_sign` luôn đi cùng `angle_sign` |
| 2 | **min PWM ghi đè toàn bộ đầu ra PD** (`magnitude = min_pwm` cho mọi lệnh khác 0) | Kp/Kd chỉnh bao nhiêu cũng vô ích — motor chỉ chạy đúng một mức PWM cố định | `motor.c`: chỉ nâng lên `min_pwm` khi `|lệnh| < min_pwm`; mặc định `g_min_pwm_* = 0` |
| 3 | `g_arm_request = 1` ngay khi boot | Xe có thể tự ARM khi đang được giữ thẳng đứng, trái với tài liệu an toàn | `g_arm_request = 0`; phải tự đặt 1 trong Live Expressions |
| 4 | `g_motor_test` phụ thuộc `g_motor_test_enable` mặc định 0 | README hướng dẫn đặt `g_motor_test=500` nhưng motor không chạy → dễ kết luận sai chiều motor | `g_motor_test_enable = 1`; chỉ cần đặt `g_motor_test` |
| 5 | Chiều lệnh motor ngược với chiều ngã thực tế (mặc định cũ `g_output_sign = +1`) | Bánh đẩy **ngược hướng ngã** → xe ngã nhanh hơn | Đặt mặc định `g_output_sign = -1` (đã kiểm chứng trên phần cứng); nếu thay motor/đảo dây, test lại Bước 4 |

---

## Sơ đồ chân

| Thiết bị | Chân |
|---|---|
| MPU6050 SCL/SDA | PB10 / PB11 (I2C2, 400 kHz, địa chỉ 0x68) |
| TB6612 STBY | PC4 |
| TB6612 PWMA / AIN1 / AIN2 | PE5 (TIM9_CH1) / PE0 / PE1 — motor trái |
| TB6612 PWMB / BIN1 / BIN2 | PE6 (TIM9_CH2) / PE2 / PC1 — motor phải |
| Encoder trái A/B | PC6 / PC7 (TIM3) |
| Encoder phải A/B | PA5 / PA1 (TIM2) |
| SWD | PA13 / PA14 |

**Lưu ý:** encoder trái là **PC6/PC7** (TIM3), encoder phải **PA5/PA1** (TIM2) — không phải PA6/PA7 hay PA0/PA1.

Nguồn: `VCC` TB6612 = 3.3 V, `VM` = nguồn motor (12 V), `VCC` MPU6050 = 3.3 V, GND chung.

---

## Build & nạp

- Mở project trong **STM32CubeIDE → Build → Debug** (ST-LINK).
- File nạp: `Debug/XECANBANG.elf`.

---

## Quy trình sử dụng

### Bước 0 — Khởi động

Giữ xe **thẳng đứng, đứng yên ~3 giây** sau khi cấp nguồn để IMU hiệu chỉnh. Xe **không tự chạy** (armed=false, STBY=LOW) — đây là tính năng an toàn.

### Bước 1 — Live Expressions

Trong CubeIDE debug, mở **Live Expressions**, thêm:

```
g_imu.who_am_i   g_imu.connected   g_imu.valid   g_imu.calibrated
g_imu.angle      g_imu.gyro_x_dps  g_imu.az_g    g_imu.total_err
g_state          g_fault_code      g_arm_request g_auto_arm_enable
g_disarm_request g_clear_fault_request           g_calib_request
g_motor_test     g_kp              g_kd          g_pwm_limit
g_output_sign    g_angle_sign      g_target_angle_deg
g_enc_left.count_total    g_enc_right.count_total
g_encoder_left_cps        g_encoder_right_cps
g_tick_overrun_count      g_control_tick_count
```

### Bước 2 — Kiểm tra IMU

| Biến | Kỳ vọng |
|---|---|
| `who_am_i` | `104` (0x68) |
| `connected` | `true` ổn định |
| `angle` | `≈ 0°` khi thẳng đứng, mượt khi nghiêng |
| `gyro_x_dps` | cùng dấu với chiều thay đổi của `angle` (đã sửa lỗi dấu) |
| `az_g` | `≈ ±1.0` khi xe thẳng đứng |
| `total_err` | không tăng |

### Bước 3 — Test motor + encoder (bánh NHẤC KHỎI ĐẤT)

Điều kiện: `g_arm_request = 0`, `g_auto_arm_enable = 0`.

1. `g_motor_test = 500` → 2 bánh quay. Xác định chiều "tiến".
   - Bánh nào quay ngược chiều tiến → `g_motor.invert_left` hoặc `g_motor.invert_right = true`.
2. `g_motor_test = 0` → dừng.
3. Quay tay từng bánh về phía trước → `g_enc_left.count_total` / `g_enc_right.count_total` **tăng**.
   - Nếu một bên giảm → đổi `g_encoder_left_invert` / `g_encoder_right_invert` (0↔1).

### Bước 4 — Kiểm tra DẤU phản hồi ⭐ (bánh vẫn nhấc khỏi đất)

1. `g_kp = 50`, `g_kd = 0`, `g_pwm_limit = 1500`.
2. `g_arm_request = 1` → `g_state` = `SYS_RUN(1)`.
3. Nghiêng nhẹ xe về phía trước → bánh phải quay **cùng hướng xe sắp ngã** (bánh chạy về phía trọng tâm).
   - Mặc định hiện tại là `g_output_sign = -1` (đã kiểm chứng đúng trên xe này).
   - Nếu bánh vẫn quay ngược → đổi sang `g_output_sign = +1`, disarm rồi thử lại.
   - Nếu bánh quay đúng → giữ nguyên.
4. `g_disarm_request = 1` → dừng ngay.
5. Trả `g_kp`, `g_kd`, `g_pwm_limit` về giá trị cân bằng (`120`, `10`, `3000`).

### Bước 5 — Cân bằng trên đất

Giá trị khởi đầu: `g_kp = 120`, `g_kd = 10`, `g_pwm_limit = 3000`.

- `g_kp`: tăng 80 → 120 → 160 → 200 đến khi xe có xu hướng tự đứng nhưng còn dao động.
- `g_kd`: tăng 5 → 8 → 12 → 16 đến khi dao động tắt.
- Nếu bánh chỉ rung mà không quay ở lệnh nhỏ (deadband hộp giảm tốc): tăng `g_min_pwm_left` / `g_min_pwm_right` 200 → 400 → 600, nhưng không đặt quá cao vì sẽ gây dao động.
- Nếu xe luôn nghiêng về một phía: giữ xe đúng điểm cân bằng thật rồi đặt `g_capture_target_request = 1` (target sẽ lấy góc hiện tại).
- Mất kiểm soát → `g_disarm_request = 1` ngay.

### Auto-arm (tuỳ chọn)

`g_auto_arm_enable = 1` → xe **tự ARM khi giữ thẳng đứng ~1 s**; nằm ngang không tự ARM. Chỉ bật sau khi đã kiểm tra dấu phản hồi ở Bước 4.

---

## Biến điều khiển qua debugger

| Biến | Tác dụng |
|---|---|
| `g_arm_request` | ghi `1`=ARM (edge-trigger; phải về 0 trước khi ARM lại) |
| `g_disarm_request` | ghi `1`=DISARM ngay |
| `g_clear_fault_request` | ghi `1`=xóa fault sau khi đã xử lý nguyên nhân (IMU phải ok) |
| `g_calib_request` | ghi `1`=hiệu chỉnh lại IMU (chỉ khi DISARM) |
| `g_capture_target_request` | ghi `1`=lấy góc hiện tại làm `target_angle` (giữ xe đúng điểm cân bằng thật) |
| `g_auto_arm_enable` | `1`=tự ARM khi giữ thẳng đứng ~1 s; mặc định `0` |
| `g_motor_test` | `−3000..+3000` chạy motor trực tiếp (bánh nhấc khỏi đất, chưa ARM) |
| `g_kp` / `g_kd` / `g_ki` | hệ số P/D/I (mặc định 120 / 10 / 0) |
| `g_target_angle_deg` | góc đặt (mặc định 0°) |
| `g_pwm_limit` | giới hạn PWM đầu ra (mặc định 3000/8399) |
| `g_output_sign` | mặc định `−1` (đã kiểm chứng trên xe này); đổi `+1` nếu bánh đẩy ngược hướng ngã |
| `g_angle_sign` | `+1` theo quy ước; đổi `−1` nếu telemetry góc ngược (gyro tự đi cùng dấu) |
| `g_angle_limit_deg` / `g_rate_limit_dps` | ngưỡng fault góc / tốc độ góc (mặc định 45° / 700°/s) |
| `g_encoder_left_invert` / `g_encoder_right_invert` | đảo dấu encoder trái/phải (0↔1) |
| `g_trim_left` / `g_trim_right` | bù lệch PWM tĩnh từng motor |
| `g_min_pwm_left` / `g_min_pwm_right` | sàn PWM khắc phục deadband hộp giảm tốc (mặc định 0) |
| `g_speed_loop_enable` | `1`=bật vòng tốc độ ngoài (mặc định `0`, chỉ bật sau khi vòng góc ổn định) |

---

## Lỗi thường gặp

| Lỗi | Nguyên nhân | Sửa |
|---|---|---|
| `connected` nhảy true/false | — | Kiểm tra `g_imu.last_hal_err`: `0x04`=NACK (dây/AD0/nguồn), `0x20`=timeout (bus treo); **AD0 phải nối GND**, đo VCC=3.3 V |
| `angle` không về ~0 | chưa calibrate | `g_calib_request=1` khi xe thẳng đứng, yên |
| Motor quay ngược | chiều sai | `g_motor.invert_left`/`invert_right = true` |
| Xe ngã nhanh khi ARM | dấu phản hồi sai | đổi dấu còn lại của `g_output_sign`, test lại Bước 4 |
| `g_state` = `SYS_FAULT(2)` | IMU/góc/rate/timing | Xem `g_fault_code`; xử lý nguyên nhân rồi `g_clear_fault_request=1`, sau đó ARM lại |
| Xe giật/dao động | gains | `g_kp` quá cao → giảm; `g_kd` thấp → tăng |
| Bánh rung nhưng không quay | deadband hộp giảm tốc | tăng `g_min_pwm_left` / `g_min_pwm_right` (200 → 400 → 600) |

> **Lịch sử lỗi `connected` nhảy:** code cũ gọi `Reconnect` (~3 s) ngay trong chu kỳ 4 ms → trạng thái hỗn loạn. Đã sửa — `Update` chỉ đọc, `connected` chỉ false sau 3 lỗi liên tiếp.

---

## Cấu trúc mã nguồn

```
Core/Inc/    mpu6050.h, motor.h, encoder.h, balance_control.h
Core/Src/    main.c, mpu6050.c, motor.c, encoder.c, balance_control.c,
             stm32f4xx_hal_msp.c, stm32f4xx_it.c
XECANBANG.ioc    cấu hình CubeMX
IMPLEMENTATION_REPORT.md    báo cáo chi tiết + changelog
```

**Chu kỳ điều khiển:** 4 ms (250 Hz), polling `HAL_GetTick` — đơn giản, không RTOS, không dynamic memory.

---

## An toàn

- Test chiều quay/dấu: **bánh nhấc khỏi đất**.
- `g_pwm_limit` = 3000 (~36%) — không tăng trước khi biết dòng motor (TB6612: 1.2 A liên tục).
- ARM trên đất: tay đỡ sẵn, `g_arm_request = 0` để dừng ngay.
- Khuyến nghị công tắc ngắt `VM` motor bằng phần cứng.

---

## Giới hạn

**Có:** cân bằng góc PD, an toàn đầy đủ, motor test, encoder.
**Chưa có:** di chuyển tiến/lùi theo lệnh, remote, Bluetooth/Wi-Fi, định hướng, giữ vị trí.

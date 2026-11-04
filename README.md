# XE TỰ CÂN BẰNG 2 BÁNH — STM32F407VET6

Firmware điều khiển xe tự cân bằng hai bánh: **STM32F407VET6** + **MPU6050** (IMU) + **TB6612** (motor driver) + **2 encoder** JGA25-370.

Mục tiêu: xe tự giữ thăng bằng tại chỗ khi được giữ thẳng đứng trên hai bánh.

---

## Sơ đồ chân

| Thiết bị | Chân |
|---|---|
| MPU6050 SCL/SDA | PB10 / PB11 (I2C2, 100 kHz, địa chỉ 0x68) |
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
g_imu.angle      g_imu.az_g        g_imu.total_err
g_state          g_arm_request     g_auto_arm    g_motor_test
g_kp             g_kd
g_enc_left.count_total    g_enc_right.count_total
g_motor.invert_left       g_motor.invert_right
```

### Bước 2 — Kiểm tra IMU

| Biến | Kỳ vọng |
|---|---|
| `who_am_i` | `104` (0x68) |
| `connected` | `true` ổn định |
| `angle` | `≈ 0°` khi thẳng đứng, mượt khi nghiêng |
| `az_g` | `≈ ±1.0` |
| `total_err` | không tăng |

### Bước 3 — Test motor + encoder (bánh NHẤC KHỎI ĐẤT)

1. `g_motor_test = 500` → 2 bánh quay. Xác định chiều "tiến".
   - Ngược chiều → `g_motor.invert_left` / `invert_right = true`.
2. `g_motor_test = 0` → dừng.
3. Quay tay bánh về trước → `count_total` tương ứng **tăng**.

### Bước 4 — Kiểm tra DẤU phản hồi ⭐ (bánh vẫn nhấc khỏi đất)

1. `g_kp = 50`, `g_kd = 0`, `g_balance.pwm_limit = 1500`.
2. `g_arm_request = 1` → `g_state` = `ST_RUN(1)`.
3. Nghiêng nhẹ xe về trước → bánh quay **theo hướng xe sắp ngã**.
   - Ngược → `g_balance.out_sign = -1` rồi thử lại.
4. `g_arm_request = 0` → dừng.

### Bước 5 — Cân bằng trên đất

- `g_kp`: tăng 50→100→150+ đến khi xe tự đứng nhưng dao động.
- `g_kd`: thêm 1→2→5→10 đến khi dao động tắt.
- Mất kiểm soát → `g_arm_request = 0` ngay.

### Auto-arm (tuỳ chọn)

`g_auto_arm = 1` → xe **tự ARM khi giữ thẳng đứng ~1 s**; nằm ngang không tự ARM.

---

## Biến điều khiển qua debugger

| Biến | Tác dụng |
|---|---|
| `g_arm_request` | `1`=ARM (edge-trigger), `0`=DISARM |
| `g_auto_arm` | `1`=tự ARM khi giữ thẳng đứng ~1 s |
| `g_calib_request` | `1`=hiệu chỉnh lại IMU |
| `g_motor_test` | `−3000..+3000` chạy motor trực tiếp |
| `g_kp` / `g_kd` | hệ số P/D |
| `g_balance.pwm_limit` | giới hạn PWM (mặc định 3000/8399) |
| `g_balance.out_sign` | đảo dấu lệnh motor |
| `g_motor.invert_left` / `invert_right` | đảo chiều từng motor |

---

## Lỗi thường gặp

| Lỗi | Nguyên nhân | Sửa |
|---|---|---|
| `connected` nhảy true/false | — | Kiểm tra `g_imu.last_hal_err`: `0x04`=NACK (dây/AD0/nguồn), `0x20`=timeout (bus treo); **AD0 phải nối GND**, đo VCC=3.3 V |
| `angle` không về ~0 | chưa calibrate | `g_calib_request=1` khi xe thẳng đứng, yên |
| Motor quay ngược | chiều sai | `g_motor.invert_left`/`invert_right = true` |
| Xe ngã nhanh khi ARM | dấu phản hồi sai | `g_balance.out_sign = -1` |
| `status` = FAULT_* | IMU/góc/rate/timing | Xem mã lỗi; không tự ARM lại — reset `g_arm_request` |
| Xe giật/dao động | gains | `g_kp` quá cao → giảm; `g_kd` thấp → tăng |

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
- `pwm_limit` = 3000 (~36%) — không tăng trước khi biết dòng motor (TB6612: 1.2 A liên tục).
- ARM trên đất: tay đỡ sẵn, `g_arm_request = 0` để dừng ngay.
- Khuyến nghị công tắc ngắt `VM` motor bằng phần cứng.

---

## Giới hạn

**Có:** cân bằng góc PD, an toàn đầy đủ, motor test, encoder.
**Chưa có:** di chuyển tiến/lùi theo lệnh, remote, Bluetooth/Wi-Fi, định hướng, giữ vị trí.

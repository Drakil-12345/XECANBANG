# XE TỰ CÂN BẰNG 2 BÁNH — STM32F407VET6

Firmware điều khiển xe tự cân bằng hai bánh dùng **STM32F407VET6**, **MPU6050** (IMU), **TB6612FNG** (driver motor) và **2 encoder** trên động cơ JGA25-370.

Mục tiêu hiện tại: **xe tự giữ thăng bằng tại chỗ** khi được giữ thẳng đứng trên hai bánh.

---

## 1. Kiến trúc hệ thống

```
TIM9 (PWM 20kHz) ──> TB6612 ──> 2 motor DC
TIM3 / TIM2      ──> Encoder trái / phải (quadrature)
I2C2             ──> MPU6050 (100 kHz)
TIM6 (4ms tick)  ──> dùng làm tham chiếu thời gian (đã chuyển sang polling
                     HAL_GetTick trong main loop để tránh phức tạp flag)
Main loop        ──> đọc IMU -> Complementary Filter -> PD control -> PWM
```

**Chu kỳ điều khiển:** 4 ms (250 Hz), polling `HAL_GetTick` — đơn giản, dễ kiểm chứng, tránh phức tạp ngắt.

---

## 2. Kết nối phần cứng

| Thiết bị | Chân STM32 | Ghi chú |
|---|---|---|
| MPU6050 SCL / SDA | PB10 / PB11 | I2C2, 100 kHz, địa chỉ 0x68 (AD0 = GND) |
| TB6612 STBY | PC4 | Output, init LOW |
| TB6612 PWMA | PE5 | TIM9_CH1 — motor trái |
| TB6612 AIN1 / AIN2 | PE0 / PE1 | Chiều motor trái |
| TB6612 PWMB | PE6 | TIM9_CH2 — motor phải |
| TB6612 BIN1 / BIN2 | PE2 / PC1 | Chiều motor phải |
| Encoder trái A/B | PC6 / PC7 | TIM3_CH1/CH2, Encoder Mode |
| Encoder phải A/B | PA5 / PA1 | TIM2_CH1/CH2, Encoder Mode |
| Debug SWD | PA13 / PA14 | SWDIO / SWCLK |

**Lưu ý quan trọng về chân encoder:**
- `TIM3` (trái) → **PC6/PC7** (AF2), **KHÔNG PHẢI** PA6/PA7.
- `TIM2` (phải) → **PA5/PA1** (AF1), **KHÔNG PHẢI** PA0/PA1.
- Sơ đồ pinout đã được sửa đồng bộ giữa `.ioc` và `stm32f4xx_hal_msp.c`. Trước khi tái sinh code từ CubeMX, kiểm tra lại pinout.

**Nguồn:**
- `VCC` logic TB6612 = 3.3 V.
- `VM` TB6612 = nguồn motor (12 V cho JGA25-370 12V).
- `VCC` MPU6050 = 3.3 V.
- GND nối chung toàn hệ thống.

---

## 3. Cấu hình ngoại vi (trong `XECANBANG.ioc`)

| Ngoại vi | Thông số |
|---|---|
| Clock | HSI → PLL: PLLM=16, PLLN=336, PLLP=2 → SYSCLK=168 MHz; APB1=42 MHz, APB2=84 MHz |
| TIM9 | PSC=0, ARR=8399 → PWM 20 kHz, CH1=PE5, CH2=PE6 |
| TIM3 | Encoder Mode TI1&TI2, ARR=65535 (16-bit), IC Filter=4 |
| TIM2 | Encoder Mode TI1&TI2, ARR=0xFFFFFFFF (32-bit), IC Filter=4 |
| TIM6 | PSC=83, ARR=3999 → 4 ms (đang init nhưng loop chính dùng `HAL_GetTick`) |
| I2C2 | 100 kHz, PB10/PB11 |

---

## 4. Cách build và nạp

### Build (headless, không mở CubeIDE)

```bash
# Trong thư mục Debug/
export PATH="/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin:$PATH"
"/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.200.202604021615/tools/bin/make.exe" -j8 all
```

Hoặc mở project trong **STM32CubeIDE → Build**.

### Nạp

- Dùng **STM32CubeIDE Debug** (ST-LINK) → `XECANBANG.elf` trong `Debug/`.
- Hoặc `STM32CubeProgrammer` → nạp `.elf` / `.hex`.

---

## 5. Cách sử dụng (quy trình chuẩn)

### Bước 0 — Khởi động an toàn

- Sau khi nạp và cấp nguồn, **giữ xe thẳng đứng, đứng yên ~3 giây** để IMU hiệu chỉnh gyro offset.
- Xe **không tự chạy motor** — `armed=false`, `STBY=LOW`, `PWM=0`. Đây là cố ý (an toàn).

### Bước 1 — Mở debug + Live Expressions

Trong CubeIDE: **Debug As → STM32 Cortex-M**, Resume (F8), mở **Live Expressions**, thêm:

```
g_imu.who_am_i        g_imu.connected    g_imu.valid
g_imu.calibrated      g_imu.angle        g_imu.gyro_x_dps
g_imu.ax_g            g_imu.ay_g         g_imu.az_g
g_imu.total_err       g_state
g_arm_request         g_auto_arm         g_motor_test
g_kp                  g_kd
g_enc_left.count_total    g_enc_right.count_total
g_motor.invert_left       g_motor.invert_right
```

### Bước 2 — Kiểm tra IMU

| Biến | Kỳ vọng |
|---|---|
| `g_imu.who_am_i` | `104` (0x68) |
| `g_imu.connected` | `true` ổn định |
| `g_imu.valid` | `true` |
| `g_imu.calibrated` | `true` |
| `g_imu.az_g` | `≈ ±1.0` khi thẳng đứng |
| `g_imu.angle` | `≈ 0°` khi thẳng đứng, đổi mượt khi nghiêng |
| `g_imu.total_err` | không tăng |

Nếu `connected` chập chờn → xem mục **6. Lỗi thường gặp**.

### Bước 3 — Test motor + encoder (bánh NHẤC KHỎI ĐẤT)

1. `g_motor_test = 500` → cả hai bánh quay.
2. Xác định chiều "tiến": bánh quay hướng nào đó là chiều xe sẽ lăn tới.
   - Nếu bánh quay ngược → chỉnh `g_motor.invert_left` / `invert_right` = `true`.
3. `g_motor_test = 0` → motor dừng (short-brake).
4. Quay bánh tay về trước → `count_total` của encoder tương ứng **tăng**.
   - Encoder trái đã `invert=true` trong code (đếm âm khi tiến).
   - Nếu phải vẫn âm → báo để đảo `g_enc_right.invert`.

### Bước 4 — Xác minh DẤU phản hồi (bánh vẫn nhấc khỏi đất) ⭐

Sai dấu = xe tăng tốc ngã. Kiểm tra trước khi đặt xuống đất.

1. `g_kp = 50`, `g_kd = 0`, `g_balance.pwm_limit = 1500`.
2. `g_arm_request = 1` → `g_state` chuyển `ST_RUN(1)`, motor được phép.
3. **Nghiêng nhẹ xe về trước** → bánh phải quay **theo hướng xe sắp ngã** (lăn về phía dưới trọng tâm).
   - ✅ Đúng → sang bước 5.
   - ❌ Ngược → `g_balance.out_sign = -1` rồi thử lại.
4. `g_arm_request = 0` → dừng ngay.

### Bước 5 — Cân bằng trên mặt đất

1. Mặt phẳng nhẵn, tay đỡ sẵn hai bên.
2. Tăng `g_kp` từ ~50 → 100 → 150 → 200+ (PWM/độ): xe có xu hướng tự đứng nhưng dao động.
3. Thêm `g_kd` từ ~1 → 2 → 5 → 10: dao động tắt dần.
4. Xe trôi một phía → `g_ki` nhỏ (~0.05–0.1) hoặc chỉnh `g_balance.target_angle`.
5. Mất kiểm soát → `g_arm_request = 0` ngay.

### Bước 6 — Auto-arm (tuỳ chọn)

Sau khi dấu + gains đã đúng:
- `g_auto_arm = 1` → xe **tự ARM khi được giữ thẳng đứng ~1 s** (IMU ok, `|angle|<10°`, gyro nhỏ).
- Xe nằm ngang không tự ARM — an toàn.
- Sau fault (ngã) → phải giữ thẳng đứng lại để ARM lại.

---

## 6. Lỗi thường gặp và cách sửa

### 6.1 `g_imu.connected` nhảy `true/false` liên tục

**Nguyên nhân đã xác định (lịch sử):**
- Code cũ gọi `MPU6050_Reconnect` (recover + init + calibrate ~3 giây) **ngay trong chu kỳ điều khiển 4 ms** → trạng thái hỗn loạn, `connected` dao động theo chu kỳ.
- Đã sửa: `MPU6050_Update` chỉ đọc + báo `valid`; `connected` chỉ `false` sau **3 lỗi liên tiếp**; reconnect chỉ ở main loop khi `g_imu_reconnect=1` (mặc định tắt).

**Nếu vẫn gặp (sau khi đã dùng code hiện tại):**
- Kiểm tra `g_imu.last_hal_err`:
  - `0x04` (AF/NACK) → cảm biến không trả lời → kiểm tra địa chỉ/dây/nguồn/AD0.
  - `0x20` (TIMEOUT) → bus treo → kiểm tra SDA bị kéo thấp, dây dài, nhiễu.
  - `0x01` (BERR) → nhiễu bus.
- Kiểm tra **AD0 trên module MPU6050 phải nối GND** (địa chỉ 0x68) — **không để trống**.
- Đo `VCC` tại chân module = 3.3 V ổn định.
- Lay nhẹ dây SDA/SCL → `connected` nhảy theo = tiếp xúc kém.

### 6.2 IMU đọc được nhưng `angle` không về ~0 khi thẳng đứng

- Chạy lại `g_calib_request = 1` khi xe **thẳng đứng, đứng yên**.
- `g_imu.angle_offset_deg` giữ offset cơ khí — nếu `angle` lệch cố định → vấn đề lắp/trọng tâm, chỉnh `g_balance.target_angle`.

### 6.3 Motor quay ngược chiều mong muốn

- `g_motor.invert_left = true` hoặc `g_motor.invert_right = true` — độc lập từng motor, không cần đấu lại dây.

### 6.4 Encoder đếm âm khi quay tiến

- `g_enc_left.invert` / `g_enc_right.invert` trong `main.c` (hoặc báo để thêm biến debugger).
- Encoder trái hiện đã `invert=true` theo kết quả test trước (đếm âm khi tiến).

### 6.5 Xe ngã nhanh hơn khi ARM (dấu phản hồi sai)

- `g_balance.out_sign = -1` — đảo dấu lệnh motor.
- Kiểm tra lại `g_balance.angle_sign` / `gyro_sign` nếu góc/gyro cũng ngược.

### 6.6 `g_balance.status` = `BAL_FAULT_*`

- `BAL_FAULT_IMU` → IMU mất/không hợp lệ — xem 6.1.
- `BAL_FAULT_ANGLE` → `|angle|` vượt ngưỡng an toàn (mặc định 30–35°).
- `BAL_FAULT_RATE` → vận tốc góc quá lớn (rung/mất kiểm soát).
- `BAL_FAULT_TIMING` → nhiều chu kỳ điều khiển bị lỡ.
- Sau fault: **không tự ARM lại** — đưa `g_arm_request` về 0 rồi lên 1 (hoặc giữ thẳng đứng nếu `g_auto_arm=1`).

### 6.7 `g_ctrl_overrun` / `missed_cycles` tăng

- Vòng điều khiển 4 ms bị chậm — thường do `MPU6050_Update` blocking lâu khi bus lỗi (timeout).
- Firmware hiện đã tách `Reconnect` ra khỏi chu kỳ điều khiển — nếu còn tăng, kiểm tra `total_err`/`last_hal_err`.

### 6.8 Xe giật mạnh/dao động lớn

- `g_kp` quá cao → giảm.
- `g_kd` quá thấp → tăng dần.
- `g_balance.pwm_limit` quá thấp → xe thiếu lực, đổ chậm.
- Motor saturate (`control_out` chạm `pwm_limit`) → tăng `pwm_limit` nếu dòng cho phép.

---

## 7. Cấu trúc mã nguồn

```
Core/Inc/    — mpu6050.h, motor.h, encoder.h, balance_control.h, main.h, stm32f4xx_it.h
Core/Src/    — main.c, mpu6050.c, motor.c, encoder.c, balance_control.c,
               stm32f4xx_hal_msp.c, stm32f4xx_it.c, syscalls.c, sysmem.c, system_stm32f4xx.c
Drivers/     — STM32F4xx_HAL_Driver, CMSIS
XECANBANG.ioc — cấu hình CubeMX (pinout, clock, timer)
IMPLEMENTATION_REPORT.md — báo cáo triển khai + changelog chi tiết
```

**Các module:**
- `mpu6050` — driver IMU, Complementary Filter, offset góc, `connected`/`valid`/`last_hal_err`.
- `motor` — TB6612: Enable/Disable/Stop (short-brake), SetLeft/Right có dấu, `invert` độc lập, `pwm_max`.
- `encoder` — TIM3 16-bit + TIM2 32-bit, delta chu kỳ, `invert` dấu.
- `balance_control` — PD `out = Kp·err − Kd·rate` (+Ki), fault angle/rate/IMU/timing → cắt motor.

---

## 8. Biến điều khiển qua debugger (Live Expressions)

| Biến | Tác dụng |
|---|---|
| `g_arm_request` | `1`=ARM (edge-trigger), `0`=DISARM |
| `g_auto_arm` | `1`=tự ARM khi giữ thẳng đứng ~1 s |
| `g_calib_request` | `1`=hiệu chỉnh lại IMU (xe thẳng đứng, yên) |
| `g_motor_test` | `−3000..+3000` chạy motor trực tiếp (bánh nhấc khỏi đất) |
| `g_kp` / `g_kd` | hệ số P/D — chỉnh trực tiếp khi đang chạy |
| `g_balance.pwm_limit` | giới hạn PWM đầu ra (mặc định 3000/8399) |
| `g_balance.out_sign` | `+1`/`−1` đảo dấu lệnh motor |
| `g_balance.angle_sign` / `gyro_sign` | chuẩn hoá dấu góc/gyro |
| `g_motor.invert_left` / `invert_right` | đảo chiều từng motor |
| `g_enc_left.invert` / `g_enc_right.invert` | đảo dấu encoder |

**Biến quan sát:** `g_imu.*`, `g_enc_*`, `g_balance.*`, `g_state`, `g_ctrl_overrun`, `g_ctrl_tick`.

---

## 9. Giới hạn hiện tại / roadmap

**Đã có:**
- Cân bằng góc PD tại chỗ.
- An toàn đầy đủ: DISARM mặc định, fault cắt motor, không tự re-ARM, IMU mất → ngắt.
- Motor test riêng khi chưa ARM.
- Encoder đọc chuẩn hoá dấu.

**Chưa có (ngoài phạm vi):**
- Điều khiển di chuyển tiến/lùi theo lệnh.
- Điều khiển từ xa / Bluetooth / Wi-Fi / app điện thoại.
- Định hướng, quay đầu, tránh vật cản.
- Vòng điều khiển giữ vị trí/tốc độ phức tạp (chỉ thêm nếu thực nghiệm chứng minh cần).

---

## 10. An toàn bắt buộc khi thử nghiệm

- Luôn giữ **bánh nhấc khỏi đất** khi test chiều quay/dấu phản hồi lần đầu.
- `pwm_limit` mặc định 3000 (~36%) — **không tăng** trước khi biết dòng motor và giới hạn TB6612 (1.2 A liên tục / 3.2 A đỉnh).
- Khi ARM trên đất: tay đỡ sẵn hai bên, `g_arm_request = 0` để dừng ngay.
- Khuyến nghị công tắc ngắt nguồn `VM` motor bằng phần cứng khi thử.
- Nếu motor nóng/quá tải → dừng, kiểm tra dòng.

---

*Firmware dùng STM32 HAL, không RTOS, không dynamic memory trong vòng điều khiển.*

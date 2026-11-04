# BÁO CÁO TRIỂN KHAI FIRMWARE — XE TỰ CÂN BẰNG 2 BÁNH

**Project:** XECANBANG | **MCU:** STM32F407VET6 | **Toolchain:** STM32CubeIDE 2.2.0 (arm-none-eabi-gcc 14.3.1)
**Ngày:** 2026 | **Trạng thái:** Mức 1 — Build OK sau sửa đợt 3 (bus recovery + IMU retry + mã lỗi HAL); chờ kiểm chứng IMU ổn định

> **CHANGELOG (đợt 3 — sau khi IMU mất kết nối khi chỉ cắm ST-LINK):**
> | # | Lỗi / rủi ro đã sửa | File | Lý do |
> |---|---|---|---|
> | 13 | **Không có cơ chế bus recovery** — I2C lockup (slave giữ SDA thấp) → reset MCU không cứu, IMU chết vĩnh viễn | `mpu6050.c/.h` | Thêm `MPU6050_BusRecover()`: phát 16 xung SCL + STOP + `HAL_I2C_Init` lại; tự gọi sau 30 lỗi liên tiếp |
> | 14 | `MPU6050_Init` trả `false` khi **1 lần đọc WHO_AM_I glitch** → IMU chết từ boot, không retry | `mpu6050.c`, `main.c` | Init retry 5× khi boot + WHO_AM_I retry 3×; `who_am_i != 0x68` chỉ đặt `whoami_ok=false` (vẫn cho chạy nếu có ACK) |
> | 15 | `who_am_i != 0x68` → `Init=false` (do tôi thêm trước đó) — **quá khắt khe**, một glitch làm IMU chết | `mpu6050.c` | Đảo về: ghi `whoami_ok` để debug, không kill Init |
> | 16 | **Không có mã lỗi HAL I2C** → không chẩn đoán được NACK/BUSY/timeout | `mpu6050.c/.h` | Thêm `last_hal_err` (0x04 AF/NACK, 0x20 TIMEOUT, …) xem trong debugger |
> | 17 | IMU mất → **không bao giờ thử kết nối lại** → người dùng phải reset | `main.c` | Tự `MPU6050_Init` + `Calibrate` lại mỗi ~1s khi `!connected` (chỉ khi DISARM) |

---

> **CHANGELOG (đợt 2 — sau khi có kết quả test phần cứng):**
> | # | Lỗi / rủi ro đã sửa | File | Lý do |
> |---|---|---|---|
> | 1 | `g_arm_request` giữ 1 → **tự ARM lại sau fault** (nguy hiểm) | `main.c` | Đổi sang **edge-trigger**: chỉ ARM khi 0→1; sau fault phải đưa về 0 rồi lên 1 mới ARM lại |
> | 2 | Lỗi I2C chỉ `valid=false` sau **10 lần** → motor chạy trên dữ liệu cũ tới ~500ms | `mpu6050.c` | Một mẫu đọc lỗi → `connected=false, valid=false` **ngay** → `Balance_Update` cắt motor cùng chu kỳ |
> | 3 | Timeout I2C 50ms → một lỗi treo ~12 chu kỳ → `g_ctrl_overrun` bùng lên hàng nghìn | `mpu6050.c/.h` | Giảm timeout xuống **5ms** (`MPU6050_I2C_TIMEOUT_MS`) — đủ cho 14 byte @100kHz, không để bus treo cản vòng 4ms |
> | 4 | `g_ctrl_tick` đọc-ghi **không atomic** → mất tick nếu ISR đến giữa | `main.c` | Bọc `__disable_irq/__enable_irq` khi lấy-xóa tick |
> | 5 | `g_ctrl_overrun` chỉ đếm, **không đưa về trạng thái an toàn** | `main.c`, `balance_control.c/.h` | Thêm `BAL_FAULT_TIMING` + `Balance_NotifyMissed()`: >5 chu kỳ lỡ → disarm + cắt motor |
> | 6 | Hiệu chỉnh góc **trừ bias Ay tuyến tính** → góc filter ≠ `atan2(Ay,Az)` khi thẳng đứng (đúng hiện tượng đã đo: góc +3.09° trong khi accel cho −2.56°) | `mpu6050.c/.h` | Đổi sang **offset góc**: `angle_offset_deg = atan2(Ay_mean,Az_mean)` trừ trong `angle_acc`; thêm `angle_acc_raw` để so sánh trực tiếp |
> | 7 | Encoder trái đếm **âm khi xe tiến** nhưng code để `invert=false` | `main.c` | Đặt `g_enc_left.invert=true` — chuẩn hóa: tiến → cả hai encoder **dương** |
> | 8 | `Ki` tích lũy `error` **không nhân dt** | `balance_control.c` | `integral += error*0.004f` — đúng thứ nguyên; `ki` vẫn mặc định 0 |
> | 9 | `g_calib_request` chạy được **khi đang ARM** → nghẽn vòng điều khiển ~2s | `main.c` | Chỉ cho calibrate khi `!g_balance.armed` |
> | 10 | ARM chỉ cần `valid` — có thể ARM khi **chưa calibrate** | `main.c`, `balance_control.c`, `mpu6050.h` | Thêm `calibrated` flag; yêu cầu `connected && valid && calibrated` để ARM và để `Balance_Update` xuất |
> | 11 | `pwm_limit` (balance) vượt `pwm_max` (motor) không bị chặn sớm | `balance_control.c` | Giới hạn đầu ra theo `min(pwm_limit, pwm_max)` |
> | 12 | Chấp nhận `who_am_i` ≠ 0x68 mà vẫn cho connected | `mpu6050.c` | `who_am_i != 0x68` → `Init` trả `false`, `connected=false` *(đã đảo lại ở đợt 3, mục 15)* |

---

## 11. CHẨN ĐOÁN IMU MẤT KẾT NỐI (tham khảo nhanh)

Khi `connected=false` hoặc IMU chập chờn, xem các biến trong Live Expressions:

| Biến | Ý nghĩa chẩn đoán |
|---|---|
| `g_imu.last_hal_err` | **Mã lỗi HAL gốc:** `0x04`=AF/NACK (cảm biến không trả lời — dây/AD0/nguồn), `0x20`=TIMEOUT (bus treo/chậm), `0x01`=BERR (nhiễu bus) |
| `g_imu.err_count` | Lỗi liên tiếp — tăng nhanh = mất hoàn toàn; nhảy loạn = chập chờn |
| `g_imu.recoveries` | >0 và tăng = bus lockup lặp lại (slave giữ SDA) — firmware đang tự cứu |
| `g_imu.who_am_i` | `0` = chưa đọc được; `104`=0x68 đúng; khác = thiết bị lạ/đọc rác |
| `g_imu.whoami_ok` | `false` + `who_am_i≠0` = có ACK nhưng dữ liệu sai → nhiễu trên bus |

Phân biệt nhanh:
- **`last_hal_err = 0x04` (NACK)** → vấn đề địa chỉ/dây/AD0 trôi/nguồn MPU.
- **`last_hal_err = 0x20` (TIMEOUT)** → bus bị treo hoặc quá chậm — kiểm tra SCL/SDA có bị kéo thấp không.
- **`recoveries` tăng liên tục** → bus lockup lặp lại — thường do một giao dịch bị hủy giữa chừng; code đã tự cứu nhưng cần tìm nguyên nhân gốc (dây/nguồn/nhiễu).

---

---

## 1. KẾT QUẢ KIỂM TRA PROJECT (Giai đoạn A)

### Cấu hình đã có và ĐÚNG với thiết kế

| Thành phần | Cấu hình trong .ioc | Trạng thái |
|---|---|---|
| System Clock | HSI→PLL: PLLM=16, PLLN=336, PLLP=2 → SYSCLK=168MHz, APB1=42MHz, APB2=84MHz, APB1 Tim=84MHz, APB2 Tim=168MHz | ✅ Đúng |
| TIM9 (PWM motor) | PSC=0, ARR=8399 → **20kHz**, CH1=PE5, CH2=PE6, PWM Mode 1, Polarity High, Pulse=0 | ✅ Đúng |
| TIM6 (chu kỳ điều khiển) | PSC=83, ARR=3999 → **4ms / 250Hz**, NVIC TIM6_DAC_IRQn đã bật | ✅ Đúng |
| TIM3 (encoder trái) | Encoder Mode TI1&TI2, ARR=65535 (16-bit), IC Filter=4 | ✅ Đúng |
| TIM2 (encoder phải) | Encoder Mode TI1&TI2, ARR=0xFFFFFFFF (32-bit), IC Filter=4 | ✅ Đúng |
| I2C2 (MPU6050) | 100kHz, SCL=PB10, SDA=PB11, 7-bit addressing | ✅ Đúng |
| GPIO | STBY=PC4, AIN1=PE0, AIN2=PE1, BIN1=PE2, BIN2=PC1 — Output PP, init LOW | ✅ Đúng |
| Debug | SWD (PA13/PA14) | ✅ Đúng |

### Cấu hình encoder đã được sửa theo sơ đồ phần cứng

| Encoder | Chân phần cứng | Peripheral / AF |
|---|---|---|
| Trái | PC6 = TIM3_CH1, PC7 = TIM3_CH2 | TIM3 Encoder Mode, AF2 |
| Phải | PA5 = TIM2_CH1, PA1 = TIM2_CH2 | TIM2 Encoder Mode, AF1 |

`.ioc` và `Core/Src/stm32f4xx_hal_msp.c` đã được đồng bộ theo cấu hình này. Sau khi mở project trong CubeMX/CubeIDE, nếu tái sinh code, kiểm tra lại phần pinout trước khi chấp nhận thay đổi.

### Điểm còn thiếu trước khi triển khai

- Chưa có bất kỳ module ứng dụng nào (main.c chỉ có skeleton CubeMX).
- Không có UART (theo đúng yêu cầu — debug qua Live Expressions).

---

## 2. FILES ĐÃ TẠO / CHỈNH SỬA

| File | Vai trò |
|---|---|
| `Core/Inc/mpu6050.h` | Header driver MPU6050 + Complementary Filter |
| `Core/Src/mpu6050.c` | Init (tự probe 0x68/0x69), WHO_AM_I, ±4g / ±500dps, DLPF 44Hz, calib bias gyro-X + accel-Y, góc = `atan2(Ay−bias, Az)` + tích phân gyroX |
| `Core/Inc/motor.h` | Header driver TB6612 |
| `Core/Src/motor.c` | Enable/Disable/Stop (short-brake), SetLeft/Right/Both có dấu, `invert_left/right` độc lập, `pwm_max` runtime |
| `Core/Inc/encoder.h` | Header đọc encoder |
| `Core/Src/encoder.c` | TIM3 16-bit + TIM2 32-bit, delta xử lý wrap đúng, `count_total`, invert dấu |
| `Core/Inc/balance_control.h` | Header bộ điều khiển PD |
| `Core/Src/balance_control.c` | `out = Kp·err − Kd·rate` (+Ki tùy chọn, mặc định 0), fault angle/rate/IMU → auto-disarm + cắt motor |
| `Core/Src/main.c` | Tích hợp toàn bộ trong **USER CODE sections** (an toàn khi tái sinh code từ .ioc) |

---

## 3. KIẾN TRÚC THỜI GIAN THỰC

```
TIM6 (4ms) ──ISR──> g_ctrl_tick++        (chỉ set cờ, không làm gì nặng)
                       │
main while(1) <────────┘
    ├── xử lý g_calib_request / g_arm_request / g_motor_test (từ debugger)
    ├── chờ g_ctrl_tick (WFI khi rảnh)
    ├── MPU6050_Update()     — đọc I2C 14 byte (~0.6ms @100kHz) + filter
    ├── Encoder_Update() ×2  — delta count chu kỳ này
    └── Balance_Update()     — PD control → Motor_SetBoth()
```

- `g_ctrl_overrun` đếm số chu kỳ bị lỡ — **phải ≈ 0** trong debugger.
- Không `HAL_Delay`, không dynamic allocation trong chu kỳ điều khiển.

---

## 4. AN TOÀN ĐÃ TRIỂN KHAI

| Cơ chế | Chi tiết |
|---|---|
| Khởi động an toàn | STBY=LOW, PWM=0, `armed=false`, `kp=kd=0` — flash xong xe **không chuyển động** |
| ARM chủ động | Chỉ qua `g_arm_request=1` trong debugger (hoặc code) |
| Fault → cắt motor | `\|angle\| > 30°`, `\|rate\| > 800°/s`, IMU mất (`err_count ≥ 10`) → `Motor_Disable` ngay |
| Không tự ARM lại | Sau fault phải ARM lại có ý thức |
| Short-brake khi dừng | `Motor_Stop()` đặt IN1=IN2=HIGH (brake), không để trôi tự do |
| Giới hạn PWM | `pwm_limit` runtime (mặc định **3000/8399 ≈ 36%**) — chưa tăng khi chưa biết dòng motor |

---

## 5. BIẾN ĐIỀU KHIỂN QUA DEBUGGER (Live Expressions)

| Biến | Kiểu | Tác dụng |
|---|---|---|
| `g_arm_request` | int32 | `1`=ARM (xuất motor), `0`=DISARM |
| `g_calib_request` | int32 | `1`=hiệu chỉnh lại IMU — giữ xe thẳng đứng, yên khi set (chỉ khi DISARM) |
| `g_auto_arm` | int32 | `1`=**tự ARM khi xe được giữ thẳng đứng ~1s** (IMU ok, \|angle\|<10°, gyro<30°/s). `0`=chỉ ARM thủ công. Sau fault: phải giữ thẳng đứng lại — xe nằm ngang không tự hồi sinh |
| `g_motor_test` | int32 | `−3000..+3000`: chạy motor trực tiếp cả 2 kênh — **chỉ khi chưa ARM + `g_auto_arm=0`, bánh nhấc khỏi đất** |
| `g_imu.last_hal_err` | uint32 | **Mã lỗi HAL I2C lần gần nhất** — 0x01 BERR, 0x04 AF/NACK (cảm biến không trả lời), 0x20 TIMEOUT. Dùng để chẩn đoán nguyên nhân mất kết nối |
| `g_imu.recoveries` | uint32 | Số lần firmware **phục hồi bus I2C** (clock SCL + re-init) — nếu tăng liên tục = bus lockup lặp lại |
| `g_imu.whoami_ok` | bool | `true` nếu WHO_AM_I=0x68; `false`=đọc được ACK nhưng giá trị khác (thiết bị lạ/rác) |
| `g_balance.kp` / `.kd` / `.ki` | float | Hệ số PID — chỉnh trực tiếp khi đang chạy |
| `g_balance.pwm_limit` | int16 | Giới hạn PWM đầu ra |
| `g_balance.angle_sign` / `.gyro_sign` / `.out_sign` | float | `+1`/`−1` — chuẩn hóa dấu phản hồi |
| `g_balance.angle_limit_deg` | float | Ngưỡng góc ngắt an toàn (mặc định 30°) |
| `g_motor.invert_left` / `.invert_right` | bool | Đảo chiều logic từng motor |
| `g_motor.pwm_max` | uint16 | Giới hạn PWM phần cứng |

**Biến quan sát:**

| Biến | Ý nghĩa |
|---|---|
| `g_imu.who_am_i` | Phải = 104 (0x68) |
| `g_imu.connected` / `.valid` / `.total_err` | Sức khỏe I2C |
| `g_imu.angle` | Góc nghiêng sau filter (độ) |
| `g_imu.gyro_x_dps` | Vận tốc góc trục cân bằng (độ/s) |
| `g_imu.ax_g/ay_g/az_g` | Gia tốc từng trục — xác minh hướng lắp |
| `g_enc_left/right.count_total` / `.delta` | Xung encoder tích lũy / theo chu kỳ |
| `g_balance.control_out` | Lệnh PWM đang xuất |
| `g_balance.status` | `BAL_OK` / `BAL_FAULT_*` |
| `g_ctrl_overrun` | Số chu kỳ 4ms bị lỡ — phải ≈ 0 |

---

## 6. GIẢ ĐỊNH CHƯA XÁC MINH (bắt buộc test trên phần cứng)

1. **Dấu phản hồi** — `angle_sign`, `gyro_sign`, `out_sign`, `invert_left/right`. Sai dấu = xe tăng tốc ngã.
2. **Công thức góc** — đang dùng `atan2(Ay−bias, Az)` + gyro X. Kiểm chứng: xe thẳng đứng → `az_g ≈ ±1`, nghiêng về trước/sau → `angle` đổi mượt.
3. **Chân encoder thực tế** — xem mục 1.
4. **Điện áp VM + dòng stall JGA25-370** — so với giới hạn TB6612 (1.2A liên tục / 3.2A đỉnh) trước khi nâng `pwm_max`.
5. **Trọng tâm cơ khí** — `target_angle` có thể cần offset nhỏ nếu xe luôn nghiêng một phía.

---

## 7. QUY TRÌNH TEST PHẦN CỨNG (theo thứ tự, không bỏ bước)

### B1 — Kiểm tra IMU (không cần nguồn motor)
1. Flash firmware, đặt xe **thẳng đứng, giữ yên** khi reset (calibrate ~2s chạy trong init).
2. Debug → xem `g_imu.who_am_i` = 104, `connected` = true, `total_err` không tăng.
3. Nghiêng xe về trước/sau → `g_imu.angle` đổi mượt, về ≈0 khi thẳng đứng.
4. Nếu `angle` tăng khi nghiêng "sau" thay vì "trước" → `g_balance.angle_sign = -1`.
5. Lắc nhẹ xe → `g_imu.gyro_x_dps` đổi đúng hướng với `angle`. Ngược → `gyro_sign = -1`.

### B2 — Motor + encoder (bánh NHẤC KHỎI ĐẤT, `armed=false`)
1. `g_motor_test = 500` → cả 2 bánh quay.
2. Xác định chiều "tiến" của xe (chiều xe sẽ lăn về phía trước). Bánh nào quay ngược → `g_motor.invert_left` hoặc `invert_right = true`.
3. `g_motor_test = -500` → kiểm tra đảo chiều hoạt động.
4. Quay tay từng bánh về trước → `count_total` của encoder tương ứng **tăng**. Giảm → sửa `Encoder_SetInvert` trong `main.c` (g_enc_left/right.invert) hoặc báo để thêm biến debugger.
5. `g_motor_test = 0` → motor dừng (brake).

### B3 — Xác minh DẤU phản hồi (bánh vẫn nhấc khỏi đất) ⭐ BƯỚC QUYẾT ĐỊNH
1. `g_balance.kp = 50`, `g_balance.kd = 0`, `g_balance.pwm_limit = 1500`.
2. `g_arm_request = 1` → ARM (IMU phải valid).
3. Nghiêng nhẹ thân xe về trước → **bánh phải quay theo hướng xe sắp ngã** (bánh lăn về phía dưới trọng tâm).
   - ✅ Đúng → sang B4.
   - ❌ Bánh quay ngược (xe sẽ ngã nhanh hơn) → `g_balance.out_sign = -1`, test lại.
4. `g_arm_request = 0` để dừng ngay khi cần.

### B4 — Cân bằng trên mặt đất
1. Mặt phẳng nhẵn, hai tay đỡ sẵn hai bên xe.
2. Tăng `kp` từ ~50 → 200+ (PWM/độ): xe bắt đầu có xu hướng tự đứng nhưng dao động.
3. Thêm `kd` từ ~1 → tăng dần đến khi dao động tắt dần.
4. Quan sát `g_ctrl_overrun` ≈ 0 và `g_imu.total_err` không tăng.
5. Xe trôi mạn một phía → cân nhắc `ki` nhỏ (~0.1) hoặc điều chỉnh `target_angle`.
6. Nếu xe giật mạnh / mất kiểm soát → `g_arm_request = 0` ngay.

### Nguyên tắc tune
- Mỗi lần thử chỉ đổi MỘT thông số.
- Dao động biên độ lớn, tần số thấp → `kp` quá cao hoặc sai dấu.
- Xe rung nhanh (buzz) → `kd` quá cao hoặc motor đang saturate (`control_out` chạm `pwm_limit`).
- Xe đổ chậm một phía không cứu được → `kp` thiếu hoặc `pwm_limit` quá thấp.

---

## 8. KẾT QUẢ BUILD

```
arm-none-eabi-gcc 14.3.1, -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
XECANBANG.elf : 0 errors, 0 warnings
text=27460  data=16  bss=2248  →  ~29.7KB flash, ~2.2KB RAM
```

Build qua: `stm32cubeidec.exe` headless, config Debug. **Build sau sửa đợt 2: 0 errors, 0 warnings.**

---

## 10. BÀI KIỂM TRA PHẦN CỨNG TIẾP THEO (sau sửa)

Thực hiện đúng thứ tự, dừng ngay khi không đạt.

### TEST 1 — Ổn định IMU khi motor không quay (đã từng đạt, cần xác nhận lại sau sửa)
- **Chuẩn bị:** chỉ cấp nguồn MCU+MPU, bỏ/chưa cấp VM; xe đặt thẳng đứng; `g_arm_request=0`, `g_motor_test=0`.
- **Quan sát:** `g_imu.who_am_i` (phải 104), `connected`, `valid`, `calibrated`, `total_err`, `angle`, `angle_acc_raw`.
- **Kỳ vọng:** `connected=true`, `calibrated=true`, `total_err` không tăng, `angle ≈ 0±1°` khi thẳng đứng (sau sửa, offset góc trừ đúng).
- **Đạt:** `angle` mượt, về ~0 khi thẳng đứng; `angle ≈ angle_acc_raw − offset` hợp lý.
- **Dừng/xử lý khi không đạt:** nếu `total_err` tăng hoặc `connected=false` ngay khi chưa có motor → vấn đề bus I2C/nguồn, không liên quan motor. Kiểm tra dây SDA/SCL, pull-up, GND chung, nguồn 3.3V.

### TEST 2 — IMU khi cấp VM nhưng PWM=0
- **Chuẩn bị:** cấp VM (nguồn motor) nhưng `g_motor_test=0`, `g_arm_request=0` → STBY vẫn LOW, motor không quay.
- **Quan sát:** `g_imu.connected`, `total_err`, `angle`.
- **Kỳ vọng:** giống TEST 1, không lỗi.
- **Đạt:** `total_err` không tăng khi có VM.
- **Không đạt:** nhiễu từ nguồn VM sang logic → kiểm tra tụ lọc VM, GND chung chắc, cách ly nguồn.

### TEST 3 — IMU khi motor quay PWM thấp (bánh nhấc khỏi đất)
- **Chuẩn bị:** bánh nhấc khỏi đất, `g_arm_request=0`. Set `g_motor_test=500`.
- **Quan sát:** `g_imu.connected`, `total_err`, `g_ctrl_overrun`.
- **Kỳ vọng:** IMU vẫn `connected`, `total_err` không tăng, `overrun` ≈ 0.
- **Đạt:** I2C không nhiễu ở PWM thấp.
- **Không đạt:** nếu `total_err`/`overrun` tăng → nhiễu EMI từ motor/dây encoder → tụ lọc motor, xoắn dây encoder, tách xa dây I2C.

### TEST 4 — Tăng dần PWM (chỉ khi TEST 3 đạt)
- Tăng `g_motor_test`: 1000 → 1500 → 2500 (chỉ tới mức đã an toàn trước đây).
- **Dừng ngay** khi `connected=false` hoặc `total_err` tăng.
- **Mục tiêu:** xác định ngưỡng PWM bắt đầu gây lỗi I2C → đặt `pwm_max` dưới ngưỡng đó.

### TEST 5 — Xác nhận dấu encoder sau chuẩn hóa
- `g_motor_test=500`, quan sát `g_enc_left.count_total` và `g_enc_right.count_total`.
- **Kỳ vọng:** cả hai **tăng dương** khi lệnh dương đẩy xe tiến (encoder trái đã invert).
- **Không đạt:** nếu trái vẫn âm → kiểm tra lại `g_enc_left.invert` (đã đặt =true); nếu phải âm → báo để đảo `g_enc_right.invert`.

### TEST 6 — Dấu phản hồi P (bánh nhấc khỏi đất) ⭐
- `g_balance.kp=50`, `kd=0`, `pwm_limit=1500`. `g_arm_request` đặt 0→1 (edge) để ARM.
- Nghiêng xe về trước → bánh quay theo hướng xe sắp ngã.
- **Không đạt:** `g_balance.out_sign = -1` rồi thử lại.
- **Kiểm tra an toàn mới:** sau khi disarm (g_arm_request về 0), nếu giữ `g_arm_request=1` và cố tình đẩy `angle` vượt 30° → phải fault `BAL_FAULT_ANGLE` và **không tự ARM lại** (vì edge-trigger).

### TEST 7 — Overrun fault (kiểm chứng cơ chế mới)
- Cố tình gây lỗi I2C (rút SDA/SDA chạm) hoặc để IMU mất → quan sát `g_balance.status`.
- **Kỳ vọng:** `BAL_FAULT_IMU` ngay, motor cắt, `armed=false`, **không tự ARM lại** khi `g_arm_request` vẫn 1.
- Muốn ARM lại: đưa `g_arm_request` về 0 rồi lên 1 sau khi IMU phục hồi.

> **Chưa kiểm chứng sau sửa:** toàn bộ các thay đổi trên mới ở mức source+build. Cần chạy TEST 1–7 trên board để xác nhận trước khi nói firmware sẵn sàng cân bằng (mức 3).

---

## 9. TRẠNG THÁI THEO TIÊU CHÍ

| Mức | Trạng thái |
|---|---|
| 1 — Firmware build | ✅ ĐẠT (0 errors, 0 warnings) |
| 2 — Ngoại vi hoạt động | ⏳ Chờ test phần cứng |
| 3 — Dấu phản hồi + chu kỳ điều khiển | ⏳ Chờ test phần cứng |
| 4 — Cân bằng thực tế | ⏳ Chờ test phần cứng |

**Không được kết luận mức 2–4 cho tới khi có kết quả đo trên board thật.**

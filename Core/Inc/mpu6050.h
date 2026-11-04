/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   Driver MPU6050 (I2C) + Complementary Filter tinh goc nghieng can bang.
  *
  * He truc lap dat THUC TE (can xac minh bang thi nghiem):
  *   - Xe tien/lui theo chieu rong 18cm.
  *   - Truc X cua MPU6050 song song chieu DAI 8cm -> truc quay NGHIENG (pitch axis).
  *   - Truc Y cua MPU6050 song song chieu RONG 18cm -> truc chuyen dong tien/lui.
  *   - Truc Z huong len khi xe thang dung (gia dinh, can xac minh).
  *
  *   => Goc nghieng can bang dung Accel Y & Accel Z:  theta_acc = atan2(Ay, Az)
  *   => Van toc goc nghieng dung Gyro X.
  *
  * DAU (sign) cua goc va gyro PHAI xac minh bang thi nghiem truoc khi ARM
  * (xem balance_control.h va huong dan kiem tra). Khong co dinh dau o day.
  ******************************************************************************
  */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================== CAU HINH (chinh trong mpu6050.c) ===================== */

/* So mau lay trung binh khi hieu chinh bias luc cam bien dung yen */
#define MPU6050_CALIB_SAMPLES       500u

/* He so Complementary Filter: alpha = trong so gyro (0..1).
 * theta = alpha*(theta + gyro*dt) + (1-alpha)*theta_acc
 * alpha cang cao -> goc muot hon nhung drift lau hon bi gyro keo di.
 * 0.98 tai 250Hz la diem khoi dau hop ly.                                */
#define MPU6050_CF_ALPHA            0.98f

/* So lan loi I2C lien tiep toi thieu truoc khi bao mat ket noi (connected).
 * Giong imu_fail>=3 trong code da chay duoc — chi bao mat ket noi sau
 * vai loi lien tiep, khong nhay trang thai vi loi le.                 */
#define MPU6050_MAX_ERR_RETRY       3u

/* Reconnect day du (recover + init + calibrate ~3s) KHONG duoc goi tu
 * MPU6050_Update — no blocking qua lau trong chu ky dieu khien 4ms va
 * tu sinh dao dong. Chi goi o main loop (ngoai chu ky dieu khien) khi
 * g_imu_reconnect=1. */

/* Timeout moi giao dich I2C.
 * QUAN TRONG: timeout qua ngan (<50ms @100kHz) lam HAL huy giao dich
 * giua chung -> MPU6050 ket o giua byte, giu SDA o muc thap -> BUS LOCKUP
 * vinh vien (chi het khi phat xung SCL phuc hoi).
 * Code cu dung 50ms -> IMU on dinh. Timeout 5-10ms -> mat ket noi lien tuc.
 * Dung 50ms: du cho bus cham/nhieu, it khi timeout that.
 * Mot giao dich timeout chi tri hoan 1-2 chu ky 4ms — chap nhan duoc.     */
#define MPU6050_I2C_TIMEOUT_MS      50u

/* ===================== THANH GHI MPU6050 ===================== */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_WHOAMI_EXPECTED     0x68   /* WHO_AM_I luon tra ve 0x68 (AD0 khong anh huong) */

/* Dia chi I2C 7-bit */
#define MPU6050_ADDR_AD0_LOW        0x68
#define MPU6050_ADDR_AD0_HIGH       0x69

/* Dai do (phai khop voi cau hinh trong Init — theo code da chay duoc):
 *  Accel +-2g  -> 16384 LSB/g
 *  Gyro  +-500dps -> 65.5 LSB/(deg/s)                                     */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f
#define MPU6050_GYRO_LSB_PER_DPS    65.5f

/* ===================== KIEU DU LIEU ===================== */
typedef struct {
  /* Raw */
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t temp_raw;

  /* Don vi vat ly (da tru bias voi gyro) */
  float ax_g, ay_g, az_g;          /* gia toc, don vi g  */
  float gx_dps, gy_dps, gz_dps;    /* van toc goc, don vi deg/s */

  /* Goc nghieng */
  float angle_acc_raw;             /* goc truc tiep tu Accel Y/Z, chua tru offset (deg) */
  float angle_offset_deg;          /* offset goc do khi xe o tu the can bang (deg) */
  float angle_acc;                 /* goc tu gia toc sau tru offset (deg), chua loc */
  float angle;                     /* goc nghieng sau Complementary Filter (deg) */
  float gyro_x_dps;                /* van toc goc truc can bang (deg/s), da tru bias */

  /* Bias do duoc luc dung yen.
   * Chi ap dung cho gyro. Goc accel duoc hieu chinh bang angle_offset_deg,
   * khong phai bias gia tuyen tinh len Ay. */
  float gyro_x_bias;
  float accel_y_bias;

  /* Trang thai */
  bool    valid;                   /* true khi doc du lieu hop le o mau gan nhat */
  bool    calibrated;              /* true sau khi bias gyro + offset goc thanh cong */
  uint32_t err_count;              /* dem so lan I2C loi lien tiep */
  uint32_t total_err;              /* tong so loi tu luc khoi dong */
  bool    connected;               /* false sau MPU6050_MAX_ERR_RETRY loi lien tiep */
  uint8_t who_am_i;                /* gia tri WHO_AM_I doc duoc */
  bool    whoami_ok;               /* true neu WHO_AM_I = 0x68; false = doc khac/garbage */

  /* Debug loi I2C (xem trong Live Expressions de chan doan nguyen nhan) */
  uint32_t last_hal_err;           /* HAL_I2C_GetError() lan loi gan nhat:
                                    *  0x01 BERR bus error, 0x02 ARLO, 0x04 AF/NACK
                                    *  (cam bien khong tra loi), 0x20 TIMEOUT      */
  uint32_t recoveries;             /* so lan phuc hoi bus (clock SCL + re-init) */
  uint32_t retry_saved;            /* so giao dich duoc cuu bang retry (tang = bus margin) */
} MPU6050_Data_t;

/* ===================== API ===================== */

/**
 * @brief Khoi tao MPU6050: quet dia chi, doc WHO_AM_I, cau hinh dai do + DLPF.
 * @param hi2c  handle I2C (I2C2)
 * @param d     struct du lieu (se duoc reset)
 * @retval true neu tim thay cam bien va WHO_AM_I hop le.
 */
bool MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *d);

/**
 * @brief Hieu chinh bias gyro-x (va accel-y) khi cam bien DUNG YEN o tu the can bang.
 *        Goi MOT LAN truoc khi can bang; dat xe thang dung, giu yen.
 * @param d        struct du lieu
 * @param samples  so mau lay trung binh (MPU6050_CALIB_SAMPLES)
 * @param delay_ms ham delay ms (truyen HAL_Delay hoac tu viet)
 * @retval true neu hieu chinh thanh cong.
 * @note  Hieu chinh nay GIA DINH xe dang o TU THE CAN BANG (thang dung) va DUNG YEN.
 *        Goc sau hieu chinh se la goc lech so voi tu the nay.
 */
bool MPU6050_Calibrate(MPU6050_Data_t *d, uint16_t samples, void (*delay_ms)(uint32_t));

/**
 * @brief Doc 14 byte du lieu (accel+temp+gyro), chuyen doi va cap nhat Complementary Filter.
 * @param d   struct du lieu
 * @param dt  chu ky goi (giay), phai bang chu ky dieu khien (0.004s o 250Hz)
 * @retval true neu doc thanh cong va cap nhat d->angle.
 * @note  Goi o tan so dieu khien (250Hz). Neu I2C loi lien tiep -> connected=false.
 */
bool MPU6050_Update(MPU6050_Data_t *d, float dt);

/**
 * @brief Dat lai goc ve 0 (dung khi vua hieu chinh xong hoac reset).
 */
void MPU6050_ResetAngle(MPU6050_Data_t *d);

/**
 * @brief Thu phuc hoi bus I2C khi slave (MPU6050) giu SDA o muc thap
 *        (bus lockup — mot giao dich bi huy giua chung khien cam bien
 *        keo SDA xuong va khong tra loi nua; reset MCU KHONG cuu duoc).
 *
 *        Cach lam: tam chuyen PB10 (SCL) thanh GPIO output, phat toi da
 *        16 xung SCL de day slave het byte con do, sau do tao STOP,
 *        roi khoi phuc alternate function + HAL_I2C_Init lai.
 *
 * @param d  struct du lieu (tang d->recoveries)
 * @retval true neu sau phuc hoi cam sensor tra loi lai (probe_address ok).
 * @note   Chi goi khi gap chuoi loi I2C lien tiep — khong goi thuong xuyen.
 */
bool MPU6050_BusRecover(MPU6050_Data_t *d);

/**
 * @brief Thu khoi tao lai hoan toan MPU6050 (Init + Calibrate neu can).
 *        Goi khi IMU mat ket noi de tu dong reconnect (chi khi DISARM).
 * @retval true neu khoi tao + hieu chinh thanh cong.
 */
bool MPU6050_Reconnect(MPU6050_Data_t *d);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

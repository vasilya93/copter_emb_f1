#include <stdint.h>
#include "sensors_fusion.h"
#include "MPU6050.h"
#include "Messenger.h"
#include "linmath.h"

#define SENS_FUS_ACC_RENEWED 0x01
#define SENS_FUS_GYRO_RENEWED 0x02

static vec3_int vec_accel = {0, 0, 0};
static vec3_int vec_gyro = {0, 0, 0};
static uint16_t fus_renew_stat = 0;

void accel_received_callback(int16_t accelx, int16_t accely, int16_t accelz);
void gyro_received_callback(int16_t gyrox, int16_t gyroy, int16_t gyroz);
inline void check_renew_state();
void perform_fusion();
inline void normalize_acc();

void sensors_fusion_init(void)
{
	MPU6050_attach_accel_handler(&accel_received_callback);
	MPU6050_attach_gyro_handler(&gyro_received_callback);
}

void accel_received_callback(int16_t accelx, int16_t accely, int16_t accelz)
{
	Messenger_SendByte(SENSFUS_MSG_ACC_INSIDE);
        vec_accel[0] = accelx;
        vec_accel[1] = accely;
        vec_accel[2] = accelz;
        fus_renew_stat |= SENS_FUS_ACC_RENEWED;
        check_renew_state();
}

void gyro_received_callback(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
	Messenger_SendByte(SENSFUS_MSG_GYRO_INSIDE);
        vec_gyro[0] = gyrox;
        vec_gyro[1] = gyroy;
        vec_gyro[2] = gyroz;
        fus_renew_stat |= SENS_FUS_GYRO_RENEWED;
        check_renew_state();
}

void check_renew_state()
{
  if ((fus_renew_stat & SENS_FUS_GYRO_RENEWED) &&
      (fus_renew_stat & SENS_FUS_ACC_RENEWED)) {
        fus_renew_stat &= ~(SENS_FUS_GYRO_RENEWED | SENS_FUS_ACC_RENEWED);
        perform_fusion();
  }
}

void perform_fusion()
{
  vec3_int_norm(vec_accel, vec_accel, MPU6050_LSB_PER_G);
  Messenger_SendDWord(vec_accel[0], MSNR_DD_SENSFUS_ACCXNORM);
  Messenger_SendDWord(vec_accel[1], MSNR_DD_SENSFUS_ACCYNORM);
  Messenger_SendDWord(vec_accel[2], MSNR_DD_SENSFUS_ACCZNORM);
}
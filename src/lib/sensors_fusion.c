#include <stdint.h>
#include <math.h>
#include "sensors_fusion.h"
#include "MPU6050.h"
#include "Messenger.h"
#include "vector.h"
#include "helper.h"

static float roll_current;
static float pitch_current;
static float yaw_current;

static vector_t vector_I_tied_previous = {1, 0, 0};
static vector_t vector_J_tied_previous = {0, 1, 0};
static vector_t vector_K_tied_previous = {0, 0, 1};

static vector_t vector_K_tied_current = {0, 0, 0};
static vector_t vector_gyro = {0, 0, 0};

void accel_received_callback(int16_t accelx, int16_t accely, int16_t accelz);
void gyro_received_callback(int16_t gyrox, int16_t gyroy, int16_t gyroz);
inline void check_renew_state();
void perform_fusion();
inline void normalize_acc();

void sensors_fusion_init(uint16_t stat)
{
  MPU6050_attach_accel_handler(&accel_received_callback);
  MPU6050_attach_gyro_handler(&gyro_received_callback);
  
  if (stat & SENSFUS_ST_CALIBRATE_GYRO)
    sensfus_stat |= SENSFUS_ST_CALIBRATE_GYRO;

  if (stat & SENSFUS_ST_CALIBRATE_ACCEL)
    sensfus_stat |= SENSFUS_ST_CALIBRATE_ACCEL;
}

void sensfus_attach_handler(void(*new_handler)(float, float, float))
{
  new_pos_handler = new_handler;
}

void accel_received_callback(int16_t accelx, int16_t accely, int16_t accelz)
{
  if ((sensfus_stat & SENSFUS_ST_ACCCBTD) ||
      (~sensfus_stat & SENSFUS_ST_CALIBRATE_ACCEL)) {
     sensfus_setacc(accelx, accely, accelz);
     return;
  }

  sensfus_calibrateacc(accelx, accely, accelz);
}

static void sensfus_setacc(int16_t accx, int16_t accy, int16_t accz)
{
  vector_K_tied_current.x = (float)(accx + accelx_offset);
  vector_K_tied_current.y = (float)(accy + accely_offset);
  vector_K_tied_current.z = (float)(accz + accelz_offset);
  sensfus_stat |= SENSFUS_ACC_RENEWED;
  check_renew_state();
}

static void sensfus_calibrateacc(int16_t accx, int16_t accy, int16_t accz)
{  
  accelx_accum += accx;
  accely_accum += accy;
  accelz_accum += accz;

  if (++accel_counter >= SENSFUS_CALIBRATION_CYCLES) {
    sensfus_stat |= SENSFUS_ST_ACCCBTD;
    accelx_offset = -accelx_accum / accel_counter;
    accely_offset = -accely_accum / accel_counter;
    accelz_offset = SENSFUS_ACC_UNIT - accelz_accum / accel_counter;
  }
}

void gyro_received_callback(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
  if ((sensfus_stat & SENSFUS_ST_GRCBTD) ||
      (~sensfus_stat & SENSFUS_ST_CALIBRATE_GYRO)) {
     sensfus_setgyro(gyrox, gyroy, gyroz);
     return;
  }
  
  sensfus_calibrategyro(gyrox, gyroy, gyroz);
}

static void sensfus_setgyro(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
  vector_gyro.x = gyrox + gyrox_offset;
  vector_gyro.y = gyroy + gyroy_offset;
  vector_gyro.z = gyroz + gyroz_offset;
  sensfus_stat |= SENSFUS_GYRO_RENEWED;
  check_renew_state();
}

static void sensfus_calibrategyro(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
  gyrox_accum += gyrox;
  gyroy_accum += gyroy;
  gyroz_accum += gyroz;

  if (++gyro_counter >= SENSFUS_CALIBRATION_CYCLES) {
    sensfus_stat |= SENSFUS_ST_GRCBTD;
    gyrox_offset = -gyrox_accum / gyro_counter;
    gyroy_offset = -gyroy_accum / gyro_counter;
    gyroz_offset = -gyroz_accum / gyro_counter;
  }
}

void check_renew_state()
{
  if ((sensfus_stat & SENSFUS_GYRO_RENEWED) &&
      (sensfus_stat & SENSFUS_ACC_RENEWED)) {
        sensfus_stat &= ~(SENSFUS_GYRO_RENEWED | SENSFUS_ACC_RENEWED);
        perform_fusion();
  }
}

void perform_fusion()
{
  vector_t vector_dTheta,
           vector_I_tied_current,
           vector_J_tied_current,
           vector_aux;
  float correction_len;
  
  vector_normalize(&vector_K_tied_current);
  
  vector_aux = vector_subtract(vector_K_tied_current, vector_K_tied_previous);
  vector_dTheta = vector_cross(vector_K_tied_previous, vector_aux);
  
  //find new vectors from angle and normalize them
  vector_aux = vector_cross(vector_dTheta, vector_I_tied_previous);
  vector_I_tied_current = vector_add(vector_I_tied_previous, vector_aux);
  vector_normalize(&vector_I_tied_current);  
  vector_aux = vector_cross(vector_dTheta, vector_J_tied_previous);
  vector_J_tied_current = vector_add(vector_J_tied_previous, vector_aux);
  vector_normalize(&vector_J_tied_current);
  
  //correct new vectors so that they are perpendicular to K vector
  vector_aux = vector_cross(vector_K_tied_current, vector_I_tied_current);
  vector_I_tied_current = vector_cross(vector_aux, vector_K_tied_current);
  vector_aux = vector_cross(vector_K_tied_current, vector_J_tied_current);
  vector_J_tied_current = vector_cross(vector_aux, vector_K_tied_current);
  
  //correct new vectors so that they are perpendicular to each other
  vector_aux = vector_I_tied_current;
  correction_len = vector_dot(vector_J_tied_current, vector_I_tied_current) / 2;
  vector_I_tied_current = vector_subtract(vector_I_tied_current,
                                          vector_multiply(vector_J_tied_current, correction_len));
  vector_J_tied_current = vector_subtract(vector_J_tied_current,
                                          vector_multiply(vector_aux, correction_len));
  
  roll_current = asin(vector_K_tied_current.y);
  pitch_current = atan2(-vector_K_tied_current.x, vector_K_tied_current.z);
  yaw_current = atan2(-vector_I_tied_current.y, vector_J_tied_current.y);
  
  vector_I_tied_previous = vector_I_tied_current;
  vector_J_tied_previous = vector_J_tied_current;
  vector_K_tied_previous = vector_K_tied_current;
  
  //Messenger_SendFloat(roll_current, MSNR_DD_SENSFUS_ROLL);
  //Messenger_SendFloat(pitch_current, MSNR_DD_SENSFUS_PITCH);
  //Messenger_SendFloat(yaw_current, MSNR_DD_SENSFUS_YAW);
  
  if (new_pos_handler != NULL) {
    new_pos_handler(roll_current, pitch_current, yaw_current);
  }
  
  helper_pulse();
}

/*void gyro_calibrate(int16_t value, coordinate_t coord)
{
  int16_t *gyro_zero;
  int32_t *accum;
  uint16_t *counter;
  uint8_t data_descr;
  uint8_t coord_is_calibrated;
  
  switch (coord) {
  case COORDINATE_X:
    gyro_zero = &MPU6050_Data.gyrox_zero;
    accum = &MPU6050_Data.gyrox_accum;
    counter = &MPU6050_Data.x_counter;
    data_descr = MSNR_DD_GYROXOFF;
    coord_is_calibrated = MPU6050_ST_GRXCBTD;
    break;
  case COORDINATE_Y:
    gyro_zero = &MPU6050_Data.gyroy_zero;
    accum = &MPU6050_Data.gyroy_accum;
    counter = &MPU6050_Data.y_counter;
    data_descr = MSNR_DD_GYROYOFF;
    coord_is_calibrated = MPU6050_ST_GRYCBTD;
    break;
  case COORDINATE_Z:
    gyro_zero = &MPU6050_Data.gyroz_zero;
    accum = &MPU6050_Data.gyroz_accum;
    counter = &MPU6050_Data.z_counter;
    data_descr = MSNR_DD_GYROZOFF;
    coord_is_calibrated = MPU6050_ST_GRZCBTD;
    break;
  }
  
  *accum += value;
  if (++(*counter) >= MPU6050_CALIBRATION_CYCLES) {
    MPU6050_Data.state |= coord_is_calibrated;
    *gyro_zero = *accum / *counter;
    Messenger_SendWord(*gyro_zero, data_descr);
  }
}*/
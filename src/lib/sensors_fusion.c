#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "sensors_fusion.h"
#include "MPU6050.h"
#include "Messenger.h"
#include "vector.h"
#include "helper.h"
#include "settings.h"

void sensfus_init()
{
  MPU6050_attach_accel_handler(&accel_received_callback);
  MPU6050_attach_gyro_handler(&gyro_received_callback);
  
  fifo_int16_initialize(&fifo_accx, SETTINGS_SENSFUS_SIZE_FILTER_ACCEL);
  fifo_int16_initialize(&fifo_accy, SETTINGS_SENSFUS_SIZE_FILTER_ACCEL);
  fifo_int16_initialize(&fifo_accz, SETTINGS_SENSFUS_SIZE_FILTER_ACCEL);

  fifo_int16_initialize(&fifo_gyrox, SETTINGS_SENSFUS_SIZE_FILTER_GYRO);
  fifo_int16_initialize(&fifo_gyroy, SETTINGS_SENSFUS_SIZE_FILTER_GYRO);
  fifo_int16_initialize(&fifo_gyroz, SETTINGS_SENSFUS_SIZE_FILTER_GYRO);
  
#ifdef SETTINGS_SENSFUS_FILTER_MEDIAN
  size_array_median = (SETTINGS_SENSFUS_SIZE_FILTER_ACCEL >
                       SETTINGS_SENSFUS_SIZE_FILTER_GYRO) ?
                       SETTINGS_SENSFUS_SIZE_FILTER_ACCEL :
                       SETTINGS_SENSFUS_SIZE_FILTER_GYRO;

  array_median = calloc(size_array_median, sizeof(int16_t));
  if (array_median == NULL) {
    size_array_median = 0;
  }
#endif
}

void sensfus_attach_handler(void(*new_handler)(float, float, float))
{
  new_pos_handler = new_handler;
}

void accel_received_callback(int16_t accelx, int16_t accely, int16_t accelz)
{
#ifdef SETTINGS_SENSFUS_CALIBRATE_ACC
  if (~sensfus_stat & SENSFUS_ST_ACCCBTD) {
     calibrateacc(accelx, accely, accelz);
     return;
  }
#endif

  setacc(accelx, accely, accelz);
}

static void setacc(int16_t accx, int16_t accy, int16_t accz)
{
  m_acc_x = perform_filtration(accx + accelx_offset, &fifo_accx);
  m_acc_y = perform_filtration(accy + accely_offset, &fifo_accy);
  m_acc_z = perform_filtration(accz + accelz_offset, &fifo_accz);
  sensfus_stat |= SENSFUS_ACC_RENEWED;
#ifdef SETTINGS_SENSFUS_SEND_FILTERED_ACC
  Messenger_SendFloat(m_acc_x, MSNR_DD_ACCELX);
  Messenger_SendFloat(m_acc_y, MSNR_DD_ACCELY);
  Messenger_SendFloat(m_acc_z, MSNR_DD_ACCELZ);
#endif
  check_renew_state();
}

#ifdef SETTINGS_SENSFUS_CALIBRATE_GYRO
static void calibrateacc(int16_t accx, int16_t accy, int16_t accz)
{  
  accelx_accum += accx;
  accely_accum += accy;
  accelz_accum += accz;

  if (++accel_counter >= SETTINGS_SENSFUS_CALIBRATION_CYCLES) {
    sensfus_stat |= SENSFUS_ST_ACCCBTD;
    accelx_offset = -accelx_accum / accel_counter;
    accely_offset = -accely_accum / accel_counter;
    accelz_offset = SENSFUS_ACC_UNIT - accelz_accum / accel_counter;
  }
}
#endif

void gyro_received_callback(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
#ifdef SETTINGS_SENSFUS_CALIBRATE_GYRO
  if (~sensfus_stat & SENSFUS_ST_GRCBTD) {
     calibrategyro(gyrox, gyroy, gyroz);
     return;
  }
#endif

  setgyro(gyrox, gyroy, gyroz);
}

static void setgyro(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
  m_gyro_x = perform_filtration(gyrox + gyrox_offset, &fifo_gyrox);
  m_gyro_y = perform_filtration(gyroy + gyroy_offset, &fifo_gyroy);
  m_gyro_z = perform_filtration(gyroz + gyroz_offset, &fifo_gyroz);
  sensfus_stat |= SENSFUS_GYRO_RENEWED;
#ifdef SETTINGS_SENSFUS_SEND_FILTERED_GYRO
  Messenger_SendFloat(m_gyro_x, MSNR_DD_ANGSPEEDX);
  Messenger_SendFloat(m_gyro_y, MSNR_DD_ANGSPEEDY);
  Messenger_SendFloat(m_gyro_z, MSNR_DD_ANGSPEEDZ);
#endif
  check_renew_state();
}

#ifdef SETTINGS_SENSFUS_CALIBRATE_GYRO
static void calibrategyro(int16_t gyrox, int16_t gyroy, int16_t gyroz)
{
  gyrox_accum += gyrox;
  gyroy_accum += gyroy;
  gyroz_accum += gyroz;

  if (++gyro_counter >= SETTINGS_SENSFUS_CALIBRATION_CYCLES) {
    sensfus_stat |= SENSFUS_ST_GRCBTD;
    gyrox_offset = -gyrox_accum / gyro_counter;
    gyroy_offset = -gyroy_accum / gyro_counter;
    gyroz_offset = -gyroz_accum / gyro_counter;
  }
}
#endif

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
  calculate_acc_angles();
  
  transform_gyro_units();
  calculate_gyro_speeds();
  calculate_gyro_yaw();
  filter_kalman();

#ifdef SETTINGS_SENSFUS_SEND_ROLL
  Messenger_SendFloat(helper_rad_to_deg(m_roll), MSNR_DD_SENSFUS_ROLL);
#endif
#ifdef SETTINGS_SENSFUS_SEND_PITCH
  Messenger_SendFloat(helper_rad_to_deg(m_pitch), MSNR_DD_SENSFUS_PITCH);
#endif
#ifdef SETTINGS_SENSFUS_SEND_YAW
  Messenger_SendFloat(helper_rad_to_deg(m_yaw_gyro), MSNR_DD_SENSFUS_YAW);
#endif

  if (new_pos_handler != NULL) {
    new_pos_handler(m_roll, m_pitch, m_yaw_gyro);
  }
  
  //helper_pulse();
}

static void calculate_acc_angles(void)
{
  float acc_len,
        acc_x_norm,
        acc_y_norm,
        acc_z_norm;

  acc_len = sqrtf(m_acc_x * m_acc_x + m_acc_y * m_acc_y + m_acc_z * m_acc_z);
  acc_x_norm = m_acc_x / acc_len;
  acc_y_norm = m_acc_y / acc_len;
  acc_z_norm = m_acc_z / acc_len;

  m_pitch_acc = asin(-acc_x_norm);
  m_roll_acc = atan2(acc_y_norm, acc_z_norm);
}

static void transform_gyro_units(void)
{
  m_gyro_x = mpu6050_gyro_to_radpersec(m_gyro_x);
  m_gyro_y = mpu6050_gyro_to_radpersec(m_gyro_y);
  m_gyro_z = mpu6050_gyro_to_radpersec(m_gyro_z);
}

static void calculate_gyro_speeds()
{
  float tan_pitch,
        cos_pitch,
        cos_roll,
        sin_roll;

  tan_pitch = tan(m_pitch);
  cos_pitch = cos(m_pitch);
  cos_roll = cos(m_roll);
  sin_roll = sin(m_roll);

  m_d_roll_gyro = m_gyro_x + sin_roll * tan_pitch * m_gyro_y + cos_roll * tan_pitch * m_gyro_z; //in radians
  m_d_pitch_gyro = cos_roll * m_gyro_y - sin_roll * m_gyro_z; //in radians
  m_d_yaw_gyro = sin_roll / cos_pitch * m_gyro_y + cos_roll / cos_pitch * m_gyro_z; //in radians
}

static void calculate_gyro_yaw(void)
{
  m_yaw_gyro += DT * m_d_yaw_gyro;      
}

static void filter_kalman(void)
{
  static float d_roll_bias = 0, d_pitch_bias = 0;
  static float p11 = 0, p12 = 0, p13 = 0,
               p21 = 0, p22 = 0, p23 = 0,
               p31 = 0, p32 = 0, p33 = 0;
  
  static float roll_diff = 0, d_roll_diff = 0;
  static float pitch_diff = 0, d_pitch_diff = 0;
  
  static float p_new11 = 0, p_new12 = 0, p_new13 = 0,
               p_new21 = 0, p_new22 = 0, p_new23 = 0,
               p_new31 = 0, p_new32 = 0, p_new33 = 0;

  static float s11 = 0, s12 = 0,
               s21 = 0, s22 = 0;
  
  static float s_det = 0;
  
  static float s_inv11 = 0, s_inv12 = 0,
               s_inv21 = 0, s_inv22 = 0;

  static float k11 = 0, k12 = 0,
               k21 = 0, k22 = 0,
               k31 = 0, k32 = 0;

  m_roll = m_roll + m_d_roll * DT - d_roll_bias * DT;
  m_pitch = m_pitch + m_d_pitch * DT - d_pitch_bias * DT;

  p11 = p11 + DT * p21 - DT * p31 + DT * (p12 + DT * p22 - DT * p32) - DT * (p13 + DT * p23 - DT * p33) + Q1;
  p12 = p12 + DT * p22 - DT * p32;
  p13 = p13 + DT * p23 - DT * p33;
  p21 = p21 + DT * p22 - DT * p23;
  p22 = p22 + Q2;
  p31 = p31 + DT * p32 - DT * p33;
  p33 = p33 + Q3;

  roll_diff = m_roll_acc - m_roll;
  d_roll_diff = m_d_roll_gyro - m_d_roll;

  pitch_diff = m_pitch_acc - m_pitch;
  d_pitch_diff = m_d_pitch_gyro - m_d_pitch;

  //S = H * P * H' + R;
  s11 = p11 + R1;
  s12 = p12;
  s21 = p21;
  s22 = p22 + R2;

  //K = P * H' / S;
  s_det = s11 * s22 - s12 * s21;

  s_inv11 = s22 / s_det;
  s_inv12 = -s12 / s_det;
  s_inv21 = -s21 / s_det;
  s_inv22 = s11 / s_det;

  k11 = p11*s_inv11 + p12*s_inv21;
  k21 = p21*s_inv11 + p22*s_inv21;
  k31 = p31*s_inv11 + p32*s_inv21;
    
  k12 = p11*s_inv12 + p12*s_inv22;
  k22 = p21*s_inv12 + p22*s_inv22;
  k32 = p31*s_inv12 + p32*s_inv22;

  //x = x + K * y;
  m_roll = m_roll + k11 * roll_diff + k12 * d_roll_diff;
  m_d_roll = m_d_roll + k21 * roll_diff + k22 * d_roll_diff;
  d_roll_bias = d_roll_bias + k31 * roll_diff + k32 * d_roll_diff;

  m_pitch = m_pitch + k11 * pitch_diff + k12 * d_pitch_diff;
  m_d_pitch = m_d_pitch + k21 * pitch_diff + k22 * d_pitch_diff;
  d_pitch_bias = d_pitch_bias + k31 * pitch_diff + k32 * d_pitch_diff;

  //P = (I - K * H) * P;
  p_new11 = - k12*p21 - p11*(k11 - 1);
  p_new12 = - k12*p22 - p12*(k11 - 1);
  p_new13 = - k12*p23 - p13*(k11 - 1);
  p_new21 = - k21*p11 - p21*(k22 - 1);
  p_new22 = - k21*p12 - p22*(k22 - 1);
  p_new23 = - k21*p13 - p23*(k22 - 1);
  p_new31 = p31 - k31*p11 - k32*p21;
  p_new32 = p32 - k31*p12 - k32*p22;
  p_new33 = p33 - k31*p13 - k32*p23;

  p11 = p_new11; p12 = p_new12; p13 = p_new13;
  p21 = p_new21; p22 = p_new22; p23 = p_new23;
  p31 = p_new31; p32 = p_new32; p33 = p_new33;
}

static float perform_filtration(int16_t value_new,
                                fifo_int16_t *const fifo_values_last)
{
#ifdef SETTINGS_SENSFUS_PERFORM_FILTRATION
  fifo_int16_push(fifo_values_last, value_new);
#ifndef SETTINGS_SENSFUS_FILTER_MEDIAN
  return get_fifo_average(fifo_values_last);
#else
  return get_fifo_median(fifo_values_last);
#endif
#else
  return (float) value_new;
#endif
}

#ifndef SETTINGS_SENSFUS_FILTER_MEDIAN
static float get_fifo_average(const fifo_int16_t *const fifo)
{
  float result = 0;

  for (uint16_t i = 0; i < fifo->size; i++) {
    result += fifo->array[i];
  }

  result /= fifo->size;

  return result;
}
#else
static float get_fifo_median(const fifo_int16_t *const fifo)
{
  int result = 0;
  uint16_t current_array_size = size_array_median;
  result = fifo_int16_get_array(fifo, array_median, &current_array_size);

  if ((result != 0) || (current_array_size < SETTINGS_SENSFUS_MEDIAN_POSITION)) {
    return 0.0;
  }

  qsort(array_median, current_array_size, sizeof(int16_t), compare);
  
  return (float) array_median[SETTINGS_SENSFUS_MEDIAN_POSITION];
}

static int compare (const void *const p1, const void *const p2)
{
  if ( *(int16_t *)p1 <  *(int16_t *)p2 ) return -1;
  if ( *(int16_t *)p1 == *(int16_t *)p2 ) return 0;
  if ( *(int16_t *)p1 >  *(int16_t *)p2 ) return 1;

  return 0;
}
#endif
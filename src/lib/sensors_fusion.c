#include <stdint.h>
#include <math.h>
#include "sensors_fusion.h"
#include "MPU6050.h"
#include "Messenger.h"
#include "vector.h"
#include "helper.h"
#include "settings.h"

static float roll_current;
static float pitch_current;
static float yaw_current;

static vector_t vector_I_tied_previous = {1, 0, 0};
static vector_t vector_J_tied_previous = {0, 1, 0};
static vector_t vector_K_tied_previous = {0, 0, 1};

static vector_t vector_K_tied_current = {0, 0, 0};
static vector_t vector_gyro = {0, 0, 0};

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
  static uint16_t *array_median = NULL;
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
  vector_K_tied_current.x = perform_filtration(accx + accelx_offset, &fifo_accx);
  vector_K_tied_current.y = perform_filtration(accy + accely_offset, &fifo_accy);
  vector_K_tied_current.z = perform_filtration(accz + accelz_offset, &fifo_accz);
  sensfus_stat |= SENSFUS_ACC_RENEWED;
#ifdef SETTINGS_SENSFUS_SEND_FILTERED_ACC
  Messenger_SendFloat(vector_K_tied_current.x, MSNR_DD_ACCELX);
  Messenger_SendFloat(vector_K_tied_current.y, MSNR_DD_ACCELY);
  Messenger_SendFloat(vector_K_tied_current.z, MSNR_DD_ACCELZ);
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
    accelz_offset = /*SENSFUS_ACC_UNIT*/ - accelz_accum / accel_counter;
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
  vector_gyro.x = perform_filtration(gyrox + gyrox_offset, &fifo_gyrox);
  vector_gyro.y = perform_filtration(gyroy + gyroy_offset, &fifo_gyroy);
  vector_gyro.z = perform_filtration(gyroz + gyroz_offset, &fifo_gyroz);
  sensfus_stat |= SENSFUS_GYRO_RENEWED;
#ifdef SETTINGS_SENSFUS_SEND_FILTERED_GYRO
  Messenger_SendFloat(vector_gyro.x, MSNR_DD_ANGSPEEDX);
  Messenger_SendFloat(vector_gyro.y, MSNR_DD_ANGSPEEDY);
  Messenger_SendFloat(vector_gyro.z, MSNR_DD_ANGSPEEDZ);
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

static float perform_filtration(int16_t value_new,
                                fifo_int16_t *const fifo_values_last)
{
#ifdef SETTINGS_SENSFUS_PERFORM_FILTRATION
  fifo_int16_push(fifo_values_last, value_new);
  return get_fifo_average(fifo_values_last);
#else
  return (float) value_new;
#endif
}

static float get_fifo_average(const fifo_int16_t *const fifo)
{
  float result = 0;

  for (uint16_t i = 0; i < fifo->size; i++) {
    result += fifo->array[i];
  }

  result /= fifo->size;

  return result;
}
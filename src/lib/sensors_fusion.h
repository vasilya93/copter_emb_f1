#ifndef SENSORS_FUSION_H
#define SENSORS_FUSION_H

#include <stddef.h>
#include <stdint.h>
#include "fifo_int16.h"
#include "settings.h"

#define SENSFUS_MSG_ACC_INSIDE 51
#define SENSFUS_MSG_GYRO_INSIDE 52

#define SENSFUS_ACC_UNIT 16384

#define SENSFUS_ACC_RENEWED 0x01
#define SENSFUS_GYRO_RENEWED 0x02

#define SENSFUS_ST_GRCBTD 0x10 //gyro calibrated
#define SENSFUS_ST_ACCCBTD 0x20 //accel calibrated

//variables
static int16_t gyrox_offset = 0;
static int16_t gyroy_offset = 0;
static int16_t gyroz_offset = 0;

static int32_t gyrox_accum = 0;
static int32_t gyroy_accum = 0;
static int32_t gyroz_accum = 0;

static int16_t accelx_offset = 0;
static int16_t accely_offset = 0;
static int16_t accelz_offset = 0;

static int32_t accelx_accum = 0;
static int32_t accely_accum = 0;
static int32_t accelz_accum = 0;

static uint16_t gyro_counter = 0;
static uint16_t accel_counter = 0;

static uint16_t sensfus_stat = 0;

static fifo_int16_t fifo_accx,
                    fifo_accy,
                    fifo_accz;

static fifo_int16_t fifo_gyrox,
                    fifo_gyroy,
                    fifo_gyroz;

#ifdef SETTINGS_SENSFUS_FILTER_MEDIAN
static uint16_t *array_median = NULL;
#endif

static void (*new_pos_handler)(float, float, float) = NULL;

//functions
void sensfus_init();
void sensfus_attach_handler(void(*)(float, float, float));

static void accel_received_callback(int16_t accelx, int16_t accely, int16_t accelz);
static void gyro_received_callback(int16_t gyrox, int16_t gyroy, int16_t gyroz);

static inline void check_renew_state();
static void perform_fusion();
static inline void normalize_acc();

static void setgyro(int16_t gyrox, int16_t gyroy, int16_t gyroz);
static void setacc(int16_t accx, int16_t accy, int16_t accz);
#ifdef SETTINGS_SENSFUS_CALIBRATE_GYRO
static void calibrategyro(int16_t gyrox, int16_t gyroy, int16_t gyroz);
#endif

#ifdef SETTINGS_SENSFUS_CALIBRATE_ACC
static void calibrateacc(int16_t accx, int16_t accy, int16_t accz);
#endif

static float perform_filtration(int16_t value_new,
                                fifo_int16_t *const fifo_values_last);

#ifndef SETTINGS_SENSFUS_FILTER_MEDIAN
static float get_fifo_average(const fifo_int16_t *const fifo);
#else
static float get_fifo_median(const fifo_int16_t *const fifo);
#endif

#endif /*SENSORS_FUSION_H*/

#ifndef SENSORS_FUSION_H
#define SENSORS_FUSION_H

#include <stddef.h>
#include <stdint.h>

#define SENSFUS_MSG_ACC_INSIDE 51
#define SENSFUS_MSG_GYRO_INSIDE 52

#define SENSFUS_CALIBRATION_CYCLES 10

#define SENSFUS_ACC_UNIT 16384

#define SENSFUS_ACC_RENEWED 0x01
#define SENSFUS_GYRO_RENEWED 0x02

#define SENSFUS_ST_USE_GYRO_PRESET 0x00
#define SENSFUS_ST_CALIBRATE_GYRO 0x04
#define SENSFUS_ST_USE_ACCEL_PRESET 0x00
#define SENSFUS_ST_CALIBRATE_ACCEL 0x08

#define SENSFUS_ST_GRCBTD 0x10 //gyro calibrated
#define SENSFUS_ST_ACCCBTD 0x20 //accel calibrated

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

static void(*new_pos_handler)(float, float, float) = NULL;

void sensors_fusion_init(uint16_t stat);
void sensfus_attach_handler(void(*)(float, float, float));
static void sensfus_setgyro(int16_t gyrox, int16_t gyroy, int16_t gyroz);
static void sensfus_setacc(int16_t accx, int16_t accy, int16_t accz);
static void sensfus_calibrategyro(int16_t gyrox, int16_t gyroy, int16_t gyroz);
static void sensfus_calibrateacc(int16_t accx, int16_t accy, int16_t accz);


#endif /*SENSORS_FUSION_H*/

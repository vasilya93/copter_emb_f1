#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "fifo_float.h"
#include "settings.h"

static float roll_desired = 0;
static float pitch_desired = 0;
static float yaw_desired = 0;

#ifdef SETTINGS_CONTROLLER_USE_INTEGRAL
static fifo_float_t fifo_roll_diff,
                    fifo_pitch_diff;

static float roll_int_coef = 1;
static float pitch_int_coef = 1;
static float yaw_int_coef = 0;
#endif

#ifdef SETTINGS_CONTROLLER_USE_PROPORTIONAL
static float roll_prop_coef = 30.00; //30 per cent for 90 deg
static float pitch_prop_coef = 30.00; //30 per cent for 90 deg
static float yaw_prop_coef = 0;
#endif

void controller_init(void);

void controller_set_roll(float roll);
void controller_set_pitch(float pitch);
void controller_set_yaw(float yaw);

#ifdef SETTINGS_CONTROLLER_USE_INTEGRAL
inline void controller_set_int_coef(float int_coef);
#endif

static void
controller_generate_control(float new_roll,
                            float new_pitch,
                            float new_yaw);

#endif /*CONTROLLER_H*/
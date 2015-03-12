#ifndef CONTROLLER_H
#define CONTROLLER_H

static float roll_desired = 0;
static float pitch_desired = 0;
static float yaw_desired = 0;

static float roll_prop_coef = 19.098593; //30 per cent for 90 deg
static float pitch_prop_coef = 19.098593; //30 per cent for 90 deg
static float yaw_prop_coef = 0;

void controller_init(void);

void controller_set_roll(float roll);
void controller_set_pitch(float pitch);
void controller_set_yaw(float yaw);

static void
controller_generate_control(float new_roll,
                            float new_pitch,
                            float new_yaw); 

#endif /*CONTROLLER_H*/
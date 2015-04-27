#include "controller.h"
#include "sensors_fusion.h"
#include "motors.h"
#include "Messenger.h"
#include "helper.h"

void controller_init(void)
{
#ifdef SETTINGS_CONTROLLER_USE_INTEGRAL
  fifo_float_initialize(&fifo_roll_diff, SETTINGS_CONTROLLER_INTEGRAL_LENGTH);
  fifo_float_initialize(&fifo_pitch_diff, SETTINGS_CONTROLLER_INTEGRAL_LENGTH);
#endif
  
  motors_set(0, MOTOR2);
  motors_set(0, MOTOR4);
  motors_set(0, MOTOR1);
  motors_set(0, MOTOR3);
  sensfus_attach_handler(controller_generate_control);
}

void controller_set_roll(float new_desired_roll)
{
  roll_desired = new_desired_roll;
}

void controller_set_pitch(float new_desired_pitch)
{
  pitch_desired = new_desired_pitch;
}

void controller_set_yaw(float new_desired_yaw)
{
  yaw_desired = new_desired_yaw;
}

static void
controller_generate_control(float new_roll,
                            float new_pitch,
                            float new_yaw)
{
  float roll_impact = 0,
        pitch_impact = 0;

  float roll_diff = roll_desired - new_roll,
        pitch_diff = pitch_desired - new_pitch;

#ifdef SETTINGS_CONTROLLER_USE_INTEGRAL
  fifo_float_push(&fifo_roll_diff, roll_diff);
  fifo_float_push(&fifo_pitch_diff, pitch_diff);
  
  roll_impact += helper_array_sum(fifo_roll_diff.array, fifo_roll_diff.size) * roll_int_coef;
  pitch_impact += helper_array_sum(fifo_pitch_diff.array, fifo_pitch_diff.size) * pitch_int_coef;
#endif  

#ifdef SETTINGS_CONTROLLER_USE_PROPORTIONAL
  roll_impact += roll_diff * roll_prop_coef;
  pitch_impact += pitch_diff * pitch_prop_coef;
#endif
  
#ifdef SETTINGS_CONTROLLER_SEND_IMPACT
  Messenger_SendFloat(roll_impact, MSNR_DD_CONTROLLER_IMPROLL);
  Messenger_SendFloat(pitch_impact, MSNR_DD_CONTROLLER_IMPPITCH);
#endif
  
  motors_set(roll_impact, MOTOR2);
  motors_set(-roll_impact, MOTOR4);
  motors_set(-pitch_impact, MOTOR1);
  motors_set(pitch_impact, MOTOR3);
}

#ifdef SETTINGS_CONTROLLER_USE_INTEGRAL
void controller_set_int_coef(float int_coef)
{
  roll_int_coef = int_coef;
  pitch_int_coef = int_coef;
  //yaw_int_coef = int_coef;
}
#endif
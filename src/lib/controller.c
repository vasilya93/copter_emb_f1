#include "controller.h"
#include "sensors_fusion.h"
#include "motors.h"
#include "Messenger.h"

void controller_init(void)
{
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
  new_roll = (roll_desired - new_roll) * roll_prop_coef;
  new_pitch = (pitch_desired - new_pitch) * pitch_prop_coef;
  
  Messenger_SendFloat(new_roll, MSNR_DD_DCM11);
  Messenger_SendFloat(new_pitch, MSNR_DD_DCM12);
  
  motors_set(new_roll, MOTOR2);
  motors_set(-new_roll, MOTOR4);
  motors_set(new_pitch, MOTOR1);
  motors_set(-new_pitch, MOTOR3);
}
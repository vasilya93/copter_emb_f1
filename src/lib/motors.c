#include "motors.h"
#include "PWM.h"

#define MOTORS_MAX 920
#define MOTORS_MIN 520

void motors_set(float value, motor_number motor)
{
  uint16_t pwm_value = 0;

  if (value > 100)
    value = 100;  
  else if (value < 0)
    value = 0;
  
  pwm_value = (uint16_t)(MOTORS_MIN + (MOTORS_MAX - MOTORS_MIN) / 100.0 * value);

  switch (motor) {
  case MOTOR1:
    pwm_set(PWM_CHANNEL1, pwm_value); 
    break;
  case MOTOR2:
    pwm_set(PWM_CHANNEL2, pwm_value);
    break;
  case MOTOR3:
    pwm_set(PWM_CHANNEL3, pwm_value);
    break;
  case MOTOR4:
    pwm_set(PWM_CHANNEL4, pwm_value);
    break;
  default:
    break;
  }
}
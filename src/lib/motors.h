#ifndef MOTORS_H
#define MOTORS_H

typedef enum motor_number{
  MOTOR1,
  MOTOR2,
  MOTOR3,
  MOTOR4
}motor_number;

void motors_set(float value, motor_number motor);

#endif /*MOTORS_H*/
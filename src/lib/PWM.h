#ifndef PWM_H
#define PWM_H

#include <stdlib.h>
#include <stdint.h>

#define PWM_DESCRETES_COUNT 10000
#define PWM_FREQUENCY 50

#define PWM_CHANNEL1 0x01 //TODO: implement check like IS_PWM_CHANNEL
#define PWM_CHANNEL2 0x02
#define PWM_CHANNEL3 0x03
#define PWM_CHANNEL4 0x04

void pwm_begin(); //50 Hz

void pwm_set(uint8_t channel, uint16_t value);

#endif /*PWM_H*/
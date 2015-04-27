#ifndef HELPER_H
#define HELPER_H

#define M_PI 3.14159265358979323846

void helper_pulse_init(void);

void helper_pulse(void);
static void helper_pulse_off(void);

inline float helper_rad_to_deg(float angle);
inline float helper_deg_to_rad(float angle);

float helper_array_sum(float *array, uint16_t size);

#endif /*HELPER_H*/
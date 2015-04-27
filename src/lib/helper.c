#include "stm32f10x.h"
#include "Timer.h"
#include "helper.h"

void helper_pulse_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
  GPIOA->CRH |= GPIO_CRH_MODE11_1;
  GPIOA->ODR &= ~GPIO_ODR_ODR11;

  Timer_init(TIMER4);
}

void helper_pulse(void)
{
  GPIOA->ODR |= GPIO_ODR_ODR11;
  Timer_start(TIMER4, helper_pulse_off, 50, false);
}

static void helper_pulse_off(void)
{
  GPIOA->ODR &= ~GPIO_ODR_ODR11;
}

float helper_rad_to_deg(float angle)
{
  return angle / M_PI * 180.0;
}

float helper_deg_to_rad(float angle)
{
  return angle / 180.0 * M_PI;
}

float helper_array_sum(float *array, uint16_t size)
{
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += array[i];
  }

  return sum;
}
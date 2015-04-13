#include "stm32f10x.h"
#include "Timer.h"
#include "helper.h"

void helper_pulse_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
  GPIOA->CRH |= GPIO_CRH_MODE11_1;
  GPIOA->ODR &= ~GPIO_ODR_ODR11;

  Timer_init(TIMER2);
}

void helper_pulse(void)
{
  GPIOA->ODR |= GPIO_ODR_ODR11;
  Timer_start(TIMER2, helper_pulse_off, 50, false);
}

static void helper_pulse_off(void)
{
  GPIOA->ODR &= ~GPIO_ODR_ODR11;
}
#include "stm32f10x.h"
#include "PWM.h"
#include "ClockControl.h"

void pwm_begin() //TODO: enabling particular channels
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  
  GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_MODE9 |
                  GPIO_CRH_MODE10 | GPIO_CRH_MODE11);
  GPIOA->CRH |= GPIO_CRH_MODE8 | GPIO_CRH_MODE9 |
                GPIO_CRH_MODE10 | GPIO_CRH_MODE11;
  GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_CNF9 |
                  GPIO_CRH_CNF10 | GPIO_CRH_CNF11);
  GPIOA->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1 |
                GPIO_CRH_CNF10_1 | GPIO_CRH_CNF11_1;
  
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                 TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                 TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  
  TIM1->PSC = (ClockControl.APB2_Frequency *2 / PWM_FREQUENCY / PWM_DESCRETES_COUNT) - 1;
  TIM1->ARR = PWM_DESCRETES_COUNT;
  
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  TIM1->CR1 |= TIM_CR1_ARPE;
  
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM1->BDTR |= TIM_BDTR_MOE;

  TIM1->CR1 |= TIM_CR1_CEN;
}

void pwm_set(uint8_t channel, uint16_t value)
{
  if (value > PWM_DESCRETES_COUNT)
    return;

  switch (channel) {
  case PWM_CHANNEL1:
    TIM1->CCR1 = value;
    break;
  case PWM_CHANNEL2:
    TIM1->CCR2 = value;
    break;
  case PWM_CHANNEL3:
    TIM1->CCR3 = value;
    break;
  case PWM_CHANNEL4:
    TIM1->CCR4 = value;
    break;
  }
}
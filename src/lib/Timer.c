#include "stddef.h"
#include "stm32f10x.h"
#include "Timer.h"
#include "ClockControl.h"

timer_t timer2 = {
  NULL,
  false,
  false,
  false
};

timer_t timer4  = {
  NULL,
  false,
  false,
  false
};

void Timer_init(uint8_t timer)
{
  switch (timer) {
  case TIMER4:
    timer4.is_initialized = true;
    NVIC_EnableIRQ(TIM4_IRQn);
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    break;
  case TIMER2:
    timer2.is_initialized = true;
    NVIC_EnableIRQ(TIM2_IRQn);
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    break;
  default:
    return;
  }
}

//doesn't consider the freq of APB1, but should
bool Timer_start(uint8_t timer, void (*handler)(), uint32_t useconds, bool do_repeat)
{
  timer_t *timer_ref;
  TIM_TypeDef *timer_settings;
  
  if (!useconds)
    return false;
  
  if (useconds > TIMER_USECONDS_MAX)
    return false;
  
  switch(timer) {
  case TIMER2:
    timer_ref = &timer2;
    timer_settings = TIM2;
    break;
  case TIMER4:
    timer_ref = &timer4;
    timer_settings = TIM4;
    break;
  default:
    return false;
  }
  
  if(!timer_ref->is_initialized) {
    return false;
  }
  
  timer_ref->is_busy = true;
  timer_ref->do_repeat = do_repeat;
  timer_ref->handler = handler;
  
  if (useconds <= 1000) {
    timer_settings->PSC = 15;
    timer_settings->ARR = useconds;
  } else if (useconds <= 10000) {
    timer_settings->PSC = 1600 - 1;
    timer_settings->ARR = useconds / 100;
  } else {
    timer_settings->PSC = 16000 - 1;
    timer_settings->ARR = useconds / 1000;
  }
  
  timer_settings->DIER |= TIM_DIER_UIE;
  timer_settings->CR1 |= TIM_CR1_CEN;
  
  return true;
}

//-----------------------------Handlers-----------------------------------------

void TIM2_IRQHandler(void)
{  
  TIM2->SR &= ~TIM_SR_UIF;
  if (!timer2.do_repeat) {
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT = 0;
    timer2.is_busy = false;
  }
  if (timer2.handler != NULL) {
    timer2.handler();
  }
}

void TIM4_IRQHandler(void)
{  
  TIM4->SR &= ~TIM_SR_UIF;
  if (!timer4.do_repeat) {
    TIM4->DIER &= ~TIM_DIER_UIE;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CNT = 0;
    timer4.is_busy = false;
  }
  if (timer4.handler != NULL) {
    timer4.handler();
  }
}

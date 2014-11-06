#include "stddef.h"
#include "stm32f10x.h"
#include "Timer.h"
#include "ClockControl.h"

timer_t timer6 = {
	NULL,
	false,
	false,
	false
};

timer_t timer7  = {
	NULL,
	false,
	false,
	false
};

void Timer_init(uint8_t timer)
{
	switch (timer) {
	case TIMER6:
		timer6.is_initialized = true;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
		break;
	case TIMER7:
		timer7.is_initialized = true;
		NVIC_EnableIRQ(TIM7_IRQn);
		RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
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
  case TIMER6:
		timer_ref = &timer6;
		timer_settings = TIM6;
    break;
  case 2:
    timer_ref = &timer7;
		timer_settings = TIM7;
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
		timer_settings->PSC = (8 * useconds) - 1;
		timer_settings->ARR = 1;
	} else if (useconds <= 10000) {
		timer_settings->PSC = 800 - 1;
		timer_settings->ARR = useconds / 100;
	} else {
		timer_settings->PSC = 8000 - 1;
		timer_settings->ARR = useconds / 1000;
	}
	
	timer_settings->DIER |= TIM_DIER_UIE;
	timer_settings->CR1 |= TIM_CR1_CEN;
	
  return true;
}

//-----------------------------Handlers-----------------------------------------

void TIM6_DAC_IRQHandler(void)
{  
  TIM6->SR &= ~TIM_SR_UIF;
	if (!timer6.do_repeat) {
		TIM6->DIER &= ~TIM_DIER_UIE;
		TIM6->CR1 &= ~TIM_CR1_CEN;
		TIM6->CNT = 0;
		timer6.is_busy = false;
	}
	if (timer6.handler != NULL) {
		timer6.handler();
	}
}

void TIM7_IRQHandler(void)
{  
  TIM7->SR &= ~TIM_SR_UIF;
	if (!timer7.do_repeat) {
		TIM7->DIER &= ~TIM_DIER_UIE;
		TIM7->CR1 &= ~TIM_CR1_CEN;
		TIM7->CNT = 0;
		timer7.is_busy = false;
	}
	if (timer7.handler != NULL) {
		timer7.handler();
	}
}

#include "stm32f10x.h"
#include "ClockControl.h"

ClockControlType ClockControl;

void ClockControl_Initialize()
{  
  //enable MSI
  RCC->CR |= RCC_CR_HSION;
  while(~RCC->CR & RCC_CR_HSIRDY);
  
  //run MSI as SYSCLK
  RCC->CFGR &= ~(RCC_CFGR_SW);
  while(RCC->CFGR & RCC_CFGR_SWS);
  
  //disable PLL to configure it
  RCC->CR &= ~RCC_CR_PLLON;
  while(RCC->CR & RCC_CR_PLLRDY);

  RCC->CFGR &= ~RCC_CFGR_PLLSRC; //HSI
  RCC->CFGR &= ~RCC_CFGR_PLLMULL;
  RCC->CFGR |= RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2; //multiply by 8
  RCC->CFGR &= ~RCC_CFGR_HPRE; //no division
  RCC->CFGR &= ~RCC_CFGR_PPRE1;
  RCC->CFGR |= RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_2; //devide by 4
  RCC->CFGR &= ~RCC_CFGR_PPRE2;
  RCC->CFGR |= RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_2; //devide by 4

  //enable PLL
  RCC->CR |= RCC_CR_PLLON;
  while(~RCC->CR & RCC_CR_PLLRDY);
  
  //run PLL as SYSCLK
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while(~RCC->CFGR & RCC_CFGR_SWS_PLL);

  ClockControl.APB1_Frequency = 8000000; // if changes, clock tuning in I2C should be changed 
  ClockControl.APB2_Frequency = 8000000;
  ClockControl.AHB_Frequency = 32000000;
  ClockControl.SYSCLK_Frequency = 32000000;
}

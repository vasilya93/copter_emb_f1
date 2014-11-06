#include "stm32f10x.h"
#include "ClockControl.h"
#include "Timer.h"
#include "USART.h"

int led_state = 0;

void init_all(void);
void delay(unsigned int cycles_num);
void toggle_led(void);
void check_USART(void);

int main(void)
{
  init_all();

  while(1);
}

void init_all(void)
{
  ClockControl_Initialize();
  USART_Begin(115200);
  Timer_init(TIMER6);
  Timer_start(TIMER6, &check_USART, 1000000, true);
  
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE2_1 | GPIO_CRL_MODE3_1;
  GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF2 | GPIO_CRL_CNF3);
  GPIOD->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR2 | GPIO_ODR_ODR3);
}

void delay(unsigned int cycles_num)
{
	while (cycles_num--);
}

void toggle_led(void)
{
	if (led_state) {
		led_state = 0;
		GPIOA->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR2 | GPIO_ODR_ODR3);
	} else {
		led_state = 1;
		GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR2 | GPIO_ODR_ODR3;
	}
}

void check_USART(void)
{
  USART_WriteByte('h');
}

#include "stm32f10x.h"
#include "ClockControl.h"
#include "Timer.h"
#include "USART.h"
#include "Serial.h"
#include "Messenger.h"
#include "Wire.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "PWM.h"
#include "sensors_fusion.h"
#include "controller.h"

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))

int led_state = 0;
bool are_devs_init = false;
/*uint32_t pwm_arr[]={250, 281, 313, 344, 375, 406, 437, 468, 499, 529, 559, 589, 618, 647, 676, 704,
732, 759, 786, 812, 838, 863, 887, 911, 935, 957, 979, 1000, 1020, 1040, 1059, 1077,
1094, 1111, 1126, 1141, 1155, 1168, 1180, 1191, 1201, 1210, 1219, 1226, 1232,
1238, 1242, 1246, 1248, 1250, 1250, 1250, 1248, 1246, 1242, 1238, 1232, 1226,
1219, 1210, 1201, 1191, 1180, 1168, 1155, 1141, 1126, 1111, 1094, 1077, 1059,
1040, 1021, 1000, 979, 957, 935, 911, 887, 863, 838, 812, 786, 759, 732, 704, 676,
647, 618, 589, 559, 529, 499, 468, 437, 407, 375, 344, 313, 282, 250};
uint8_t pwm_counter = 0;*/

void init_all(void);
void delay(unsigned int cycles_num);
void toggle_led(void);
void check_USART(void);
void check_serial(void);
void start_operation(void);
void begin_wire(void);
void init_pwm(void);
/*void switch_pwm(void);*/

int main(void)
{
  init_all();
  
  while(1);
}

void init_all(void)
{
  ClockControl_Initialize();
  Serial_Begin(57600);
  pwm_begin();
  Messenger_Initialize(&start_operation, MSNR_MODE_5BYTE);
  messenger_attach_pwm(&pwm_set);
  
  Wire_Initialize();
  MPU6050_Initialize();
  //HMC5883_initialize();
  sensors_fusion_init(SENSFUS_ST_CALIBRATE_GYRO | SENSFUS_ST_CALIBRATE_ACCEL);
  controller_init();
  
  
  //receiving first data from gyro doesn't reset i2c busy flag 
  //the last receiving doesn't end up logically
  //says nothing in the end; doesn't complete the operation; doesn't end with receiving two bytes or 
  
  //RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  //GPIOA->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE3_1 | GPIO_CRL_MODE4_1;
  //GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF3 | GPIO_CRL_CNF4);
  //GPIOA->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR3 | GPIO_ODR_ODR4);
  
  //Timer_init(TIMER6);
  //Timer_start(TIMER6, toggle_led, 1000000, false);
}

void start_operation(void)
{
  Timer_init(TIMER6);
  Timer_start(TIMER6, begin_wire, 10000, true);
}

void delay(unsigned int cycles_num)
{
  while (cycles_num--);
}

void toggle_led(void)
{
  if (led_state) {
    led_state = 0;
    GPIOA->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1 |
                    GPIO_ODR_ODR3 | GPIO_ODR_ODR4);
    /*Timer_start(TIMER6, toggle_led, 1000 * pwm_arr[pwm_counter], false);
    if (++pwm_counter >= NELEMS(pwm_arr))
      pwm_counter = 0;*/
  } else {
    led_state = 1;
    GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1 |
                  GPIO_ODR_ODR3 | GPIO_ODR_ODR4;
    Timer_start(TIMER6, toggle_led, 1000000, false);
  }
}

void check_USART(void)
{
  USART_WriteByte('h');
}

void check_serial(void)
{
  Serial_WriteLine("Hello world!\n");
}

void begin_wire(void)
{
  if (are_devs_init) {
    Wire_BeginCycle();
    return;
  }
  
  are_devs_init = true;
  Wire_InitDevices();
}

/*void switch_pwm(void)
{
  pwm_set(PWM_CHANNEL1, pwm_arr[pwm_counter]);
  pwm_set(PWM_CHANNEL2, pwm_arr[pwm_counter]);
  pwm_set(PWM_CHANNEL3, pwm_arr[pwm_counter]);
  pwm_set(PWM_CHANNEL4, pwm_arr[pwm_counter]);

  if (++pwm_counter >= NELEMS(pwm_arr))
    pwm_counter = 0;
}*/
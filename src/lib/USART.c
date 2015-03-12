#include "math.h"
#include "stddef.h"
#include "stdlib.h"
#include "stm32f10x.h"
#include "USART.h"
#include "ClockControl.h"

USART_Type USART = {
	NULL,
	0,
	NULL,
	0,
	NULL,
	0,
	false
};

void USART_SetupBasic(unsigned int speed);

void USART_Begin(unsigned int speed)
{
  USART_SetupBasic(speed);
  USART_CURRENT->CR1 |= USART_CR1_RE | USART_CR1_TE;
}

void USART_BeginConfigured(uint16_t speed, uint8_t conf)
{
  USART_SetupBasic(speed);
  
  //configuring data bits, parity control and stop bits
  if(conf & USART_9BYTE) {
    USART_CURRENT->CR1 |= USART_CR1_M;
  } else {
    USART_CURRENT->CR1 &= ~USART_CR1_M;
  }
  
  if(conf & USART_PCENABLE) {
    USART_CURRENT->CR1 |= USART_CR1_PCE;
    if(conf & USART_PARITY_ODD) {
      USART_CURRENT->CR1 |= USART_CR1_PS;
    } else {
      USART_CURRENT->CR1 &= ~USART_CR1_PS;
    }
  } else {
    USART_CURRENT->CR1 &= ~USART_CR1_PCE;
  }
  
  switch(conf & USART_SB)
  {
  case USART_SB1:
    USART_CURRENT->CR2 &= ~USART_CR2_STOP;
    break;
  case USART_SB05:
    USART_CURRENT->CR2 |= USART_CR2_STOP_0;
    break;
  case USART_SB2:
    USART_CURRENT->CR2 |= USART_CR2_STOP_1;
    break;
  case USART_SB15:
    USART_CURRENT->CR2 |= USART_CR2_STOP;
    break;
  }
  
  USART_CURRENT->CR1 |= USART_CR1_RE | USART_CR1_TE;
}

void USART_End()
{
#ifdef USE_USART3
  RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
  RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
#else
  RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
  RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
#endif
}

void USART_WriteByte(uint8_t writtenByte)
{
  USART_CURRENT->DR = writtenByte;
}

bool USART_ByteReceived_Attach(void (*handler)(uint8_t receicedByte))
{
  void (**newPointer)(uint8_t receivedByte);
	
  if(handler == NULL)
  {
    return false;
  }
  
  USART.BRH_Amount++;
  newPointer = (void (**)(uint8_t))realloc(USART.ByteReceivedHandlers,
                                           USART.BRH_Amount*sizeof(void(*)(uint8_t)));
  
  if(newPointer == NULL)
  {
    USART.BRH_Amount--;
    return false;
  }
  
  USART.ByteReceivedHandlers = newPointer;
  USART.ByteReceivedHandlers[USART.BRH_Amount - 1] = handler;
  
  return true;
}

void USART_ByteReceived_Detach()
{
  free(USART.ByteReceivedHandlers);
  USART.ByteReceivedHandlers = NULL;
  USART.BRH_Amount = 0;
}

bool USART_TransRegEmpty_Attach(void (*handler)())
{
  void (**newPointer)();

  if(handler == NULL) {
    return false;
  }
  
  USART.TREH_Amount++;
  newPointer = (void (**)())realloc(USART.TransRegEmptyHandlers, USART.TREH_Amount*sizeof(void(*)()));
  
  if(newPointer == NULL) {
    USART.TREH_Amount--;
    return false;
  }

  USART.TransRegEmptyHandlers = newPointer;
  USART.TransRegEmptyHandlers[USART.TREH_Amount - 1] = handler;
  
  return true;
}

void USART_TransRegEmpty_Detach()
{
  free(USART.TransRegEmptyHandlers);
  USART.TransRegEmptyHandlers = NULL;
  USART.TREH_Amount = 0;
}

bool USART_TransComplete_Attach(void (*handler)())
{
  void (**newPointer)();
	
  if(handler == NULL) {
    return false;
  }
  
  USART.TCH_Amount++;
  newPointer = (void (**)())realloc(USART.TransCompleteHandlers, USART.TCH_Amount*sizeof(void(*)()));
  
  if(newPointer == NULL) {
    USART.TCH_Amount--;
    return false;
  }
  
  USART.TransCompleteHandlers = newPointer;
  USART.TransCompleteHandlers[USART.TCH_Amount - 1] = handler;
  
  return true;
}

void USART_TransComplete_Detach(void)
{
  free(USART.TransCompleteHandlers);
  USART.TransCompleteHandlers = NULL;
  USART.TCH_Amount = 0;
}

void USART_InitStructure(void)
{
  free(USART.ByteReceivedHandlers);
  USART.ByteReceivedHandlers = NULL;
  USART.BRH_Amount = 0;
  
  free(USART.TransRegEmptyHandlers);
  USART.TransRegEmptyHandlers = NULL;
  USART.TREH_Amount = 0;
  
  free(USART.TransCompleteHandlers);
  USART.TransCompleteHandlers = NULL;
  USART.TCH_Amount = 0;
  
  USART.IsStructureInitialized = true;
}

void USART_EnableInterrupts(uint8_t interrupts)
{
  if(interrupts & USART_IT_TXE) {
    USART_CURRENT->CR1 |= USART_CR1_TXEIE;
  }

  if(interrupts & USART_IT_TC) {
    USART_CURRENT->CR1 |= USART_CR1_TCIE;
  }

  if(interrupts & USART_IT_RXNE) {
    USART_CURRENT->CR1 |= USART_CR1_RXNEIE;
  }

  if(interrupts & USART_IT_PE) {
    USART_CURRENT->CR1 |= USART_CR1_PEIE;
  }
}

void USART_DisableInterrupts(uint8_t interrupts)
{
  if(interrupts & USART_IT_TXE) {
    USART_CURRENT->CR1 &= ~USART_CR1_TXEIE;
  }

  if(interrupts & USART_IT_TC) {
    USART_CURRENT->CR1 &= ~USART_CR1_TCIE;
  }

  if(interrupts & USART_IT_RXNE) {
    USART_CURRENT->CR1 &= ~USART_CR1_RXNEIE;
  }

  if(interrupts & USART_IT_PE) {
    USART_CURRENT->CR1 &= ~USART_CR1_PEIE;
  }
}

//-----------------------------Handlers-----------------------------------------

#ifdef USE_USART3
void USART3_IRQHandler(void)
{
  unsigned int i;
  if((USART3->SR & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE))
  {
    //cleared by a write to DR
    for(i = 0; i < USART.TREH_Amount; i++)
    {
      (*USART.TransRegEmptyHandlers[i])();
    }
  }
  if((USART3->SR & USART_SR_TC) && (USART3->CR1 & USART_CR1_TCIE))
  {
    //cleared by a read from SR followed by a write to DR
    //can be cleared by writing '0' to it (recommended for multibuffer communication)
    USART3->SR &= ~USART_SR_TC;
    for(i = 0; i < USART.TCH_Amount; i++)
    {
      (*USART.TransCompleteHandlers[i])();
    }
  }
  if((USART3->SR & USART_SR_RXNE) && (USART3->CR1 & USART_CR1_RXNEIE))
  {
    //cleared by a read to DR
    //can be cleared by writing '0' to it (recommended for multibuffer communication)
    uint8_t receivedByte = USART3->DR;
    for(i = 0; i < USART.BRH_Amount; i++)
    {
      (*USART.ByteReceivedHandlers[i])(receivedByte);
    }
  }
  
  //PE cleared by a sequence of SR read and read/write access to DR
  //software must wait for RXNE before clearing PE
}

#else

void USART2_IRQHandler(void)
{
  unsigned int i;
  if((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
    //cleared by a write to DR
    for(i = 0; i < USART.TREH_Amount; i++) {
      (*USART.TransRegEmptyHandlers[i])();
    }
  }

  if((USART2->SR & USART_SR_TC) && (USART2->CR1 & USART_CR1_TCIE)) {
    //cleared by a read from SR followed by a write to DR
    //can be cleared by writing '0' to it (recommended for multibuffer communication)
    USART2->SR &= ~USART_SR_TC;
    for(i = 0; i < USART.TCH_Amount; i++) {
      (*USART.TransCompleteHandlers[i])();
    }
  }

  if((USART2->SR & USART_SR_RXNE) && (USART2->CR1 & USART_CR1_RXNEIE)) {
    //cleared by a read to DR
    //can be cleared by writing '0' to it (recommended for multibuffer communication)
    uint8_t receivedByte = USART2->DR;
    for(i = 0; i < USART.BRH_Amount; i++) {
      (*USART.ByteReceivedHandlers[i])(receivedByte);
    }
  }
  
  //PE cleared by a sequence of SR read and read/write access to DR
  //software must wait for RXNE before clearing PE
}
#endif

//-------------------------Hidden funtions--------------------------------------

void USART_SetupBasic(unsigned int baudrate)
{
  float Fraction;
  float USARTDIV;
  unsigned int DivMantissa;
  unsigned int DivFraction;

#ifdef USE_USART3 
  NVIC_EnableIRQ(USART3_IRQn);
  
  //enabling GPIOB clocking
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  //configuring output pins
  GPIOB->CRH &= ~(GPIO_CRH_MODE10 |
                 GPIO_CRH_CNF10 |
                 GPIO_CRH_MODE11 |
                 GPIO_CRH_CNF11);
  GPIOB->CRH |= (GPIO_CRH_MODE10_1 |
                 GPIO_CRH_CNF10_1 |
                 GPIO_CRH_CNF11_0);
  
  //USART clocking
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; 
#else
  NVIC_EnableIRQ(USART2_IRQn);

  //enabling GPIOB clocking
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  //configuring output pins
  GPIOA->CRL &= ~(GPIO_CRL_MODE2 |
                  GPIO_CRL_CNF2 |
                  GPIO_CRL_MODE3 |
                  GPIO_CRL_CNF3);
  GPIOA->CRL |= (GPIO_CRL_MODE2_1 |
                 GPIO_CRL_CNF2_1 |
                 GPIO_CRL_CNF3_0);
  
  //USART clocking
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#endif
 
  
  //configuring USART
  USART_CURRENT->SR &= ~USART_SR_TC;
  USART_CURRENT->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TCIE;

  //calculating Baudrate Register value
  USARTDIV = (float)ClockControl.APB1_Frequency / (float)(16 * baudrate);
  DivMantissa = (int)USARTDIV;
  while(DivMantissa < 1); //caution in case fck is too low or baudrate is too big

  Fraction = (USARTDIV - DivMantissa);
  if(Fraction >= (0.0625 * 15.5)) {
    DivFraction = 0;
    DivMantissa++;
  } else {
    DivFraction = (int) floor((Fraction / 0.0625) + 0.5);
  }

  USART_CURRENT->BRR = DivFraction | (DivMantissa << 4);
}

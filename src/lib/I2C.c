#include <stdlib.h>
#include "stm32f10x.h"
#include "I2C.h"
#include "Messenger.h"

I2C_Operation_Type I2C_Operation;
uint16_t addr_counter = 0;
uint16_t sb_counter = 0;
uint16_t txe_counter = 0;
uint16_t rxne_counter = 0;
uint16_t tot_counter = 0;

void I2C_SendAddress(void);
void I2C_OperateTransmitter(void);
void I2C_OperateReceiver(void);
void I2C_TransmitRegisterEmpty(void);
void I2C_ReceiveRegisterNotEmpty(void);

void I2C_Begin(void)
{
  I2C_Operation.State = 0;
  I2C_Operation.Bytes = calloc(I2C_DATA_SIZE, sizeof(uint8_t));
  I2C_Operation.Description.DataDescript = I2C_DD_NODATA;
  I2C_Operation.OCH_Amount = 0;
  
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
  
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  
  GPIOB->CRL &= ~(GPIO_CRL_MODE6 |
                  GPIO_CRL_MODE7);
  GPIOB->CRL |= (GPIO_CRL_MODE6_0 |
                 GPIO_CRL_CNF6 |
                   GPIO_CRL_MODE7_0 |
                     GPIO_CRL_CNF7);
  //GPIOB->MODER |=  /*I2C1*/ GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
  //GPIOB->OTYPER |= /*I2C1*/ GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7; //open drain
  //GPIOB->OSPEEDR |= /*I2C1*/ GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
  //GPIOB->PUPDR |= /*I2C1*/ GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0; //pull up
  //GPIOB->AFR[0] |= (0x4 << 24) | (0x4 << 28);
  
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
  
  I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_FREQ_3 /*changes if PCLK1 changes*/;
  I2C1->CCR |= 40; /*changes if PCLK1 changes*/
  I2C1->TRISE = 9; /*changes if PCLK1 changes*/
  I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
}

bool I2C_OperationComplete_Attach(void (*handler)(void* operation))
{
  void (**newPointer)(void* operation);
  
  if(handler == NULL)
  {
    return false;
  }
  
  I2C_Operation.OCH_Amount++;
  newPointer = (void (**)(void*))realloc(I2C_Operation.OperationCompleteHandlers, I2C_Operation.OCH_Amount*sizeof(void(*)(void*)));
  
  if(newPointer == NULL)
  {
    I2C_Operation.OCH_Amount--;
    return false;
  }
  
  I2C_Operation.OperationCompleteHandlers = newPointer;
  I2C_Operation.OperationCompleteHandlers[I2C_Operation.OCH_Amount - 1] = handler;
  
  return true;
}

bool I2C_StartOperation(I2C_OpDescript_Type description, uint8_t* bytes)
{
  unsigned int i;

  if((description.BytesNum > I2C_DATA_SIZE) || (I2C_Operation.State & I2C_ST_BUSY)) {
    Messenger_SendByte(I2C_MSG_STRTFL); 
    free(bytes);
    return false;
  }
  I2C_Operation.State |= I2C_ST_BUSY;
  I2C_Operation.Description = description;
  I2C_Operation.CurrentByte = 0;
  if(!description.IsRead) {
    for(i = 0; i < description.BytesNum; i++) {
      I2C_Operation.Bytes[i] = bytes[i];
    }
    free(bytes);
  }
  
  I2C1->CR1 |= I2C_CR1_ACK;
  
  while(I2C1->SR2 & I2C_SR2_BUSY) {
    asm("NOP");
  }
  
  I2C1->CR1 |= I2C_CR1_START;
  //Messenger_SendByte(I2C_MSG_STRTST); 
  
  return true;
}

void I2C_SetOpDescription(I2C_OpDescript_Type* descript, uint16_t dataDescript,
                          uint8_t deviceAddress, bool isRead, uint8_t bytesNum)
{
  descript->DataDescript = dataDescript;
  descript->DeviceAddress = deviceAddress;
  descript->IsRead = isRead;
  descript->BytesNum = bytesNum;
}

void I2C1_EV_IRQHandler(void)
{
  if(I2C1->SR1 & I2C_SR1_SB) {
    sb_counter++;
    I2C_SendAddress();
    //Messenger_SendByte(I2C_MSG_SBSND);
    return;
  }
  
  if(I2C1->SR1 & I2C_SR1_ADDR) {
    addr_counter++;
    //Messenger_SendByte(I2C_MSG_ADSND);
    if (I2C_Operation.Description.IsRead &&
        I2C_Operation.Description.BytesNum == 1) {
          I2C1->CR1 &= ~I2C_CR1_ACK; //This should be done during event6, before addr is cleared
          //Messenger_SendByte(I2C_MSG_NACKST);
        }
    if(I2C1->SR2 & I2C_SR2_TRA) {
      I2C_OperateTransmitter();
    } else {
      I2C_OperateReceiver();
    }
    return;
  }
  
  if(I2C1->SR1 & I2C_SR1_TXE) {
    txe_counter++;
    I2C_TransmitRegisterEmpty(); //it happens that new start is set from the interrupt handler
    //not very good. it's better to give to a main thread idea, that it should start the operation;
    return;
  }
  
  if(I2C1->SR1 & I2C_SR1_RXNE) {
    rxne_counter++;
    I2C_ReceiveRegisterNotEmpty();
    return;
  }
  
  tot_counter++;
}

void I2C1_ER_IRQHandler(void)
{
  if(I2C1->SR1 & I2C_SR1_AF)
  {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C_Operation.State &= ~I2C_ST_BUSY;
    Messenger_SendByte(I2C_MSG_AF);    
    
    I2C1->CR1 |= I2C_CR1_ACK;  
    while(I2C1->SR2 & I2C_SR2_BUSY);  
    I2C1->CR1 |= I2C_CR1_START;
    
    //I2C_SetStart(false);
  } else {
    Messenger_SendByte(I2C_MSG_ERR);
  }
}

//-------------------------Hidden functions-------------------------------------

void I2C_SendAddress()
{
  I2C1->DR = (I2C_Operation.Description.DeviceAddress << 1) |
    (I2C_Operation.Description.IsRead ? 1 : 0);
  //Messenger_SendByte(I2C_MSG_ADSND);
}

void I2C_OperateTransmitter()
{
  I2C1->DR = I2C_Operation.Bytes[0];
  I2C_Operation.CurrentByte++;
  //Messenger_SendByte(I2C_MSG_TRNSMTR);
}

void I2C_OperateReceiver()
{
  if(I2C_Operation.Description.BytesNum == 1) {
    I2C1->CR1 |= I2C_CR1_STOP;
  }
  //Messenger_SendByte(I2C_MSG_RCVR);
}

void I2C_TransmitRegisterEmpty()
{
  unsigned int i;
  if(I2C_Operation.CurrentByte < I2C_Operation.Description.BytesNum) {
    I2C1->DR = I2C_Operation.Bytes[I2C_Operation.CurrentByte++];
    //Messenger_SendByte(I2C_MSG_TXE);
  } else if(I2C1->SR1 & I2C_SR1_BTF) {
    I2C1->CR1 |= I2C_CR1_STOP;
    //Messenger_SendByte(I2C_MSG_TRSCPL);
    for (int i = 0; i < 20; i++)
      __NOP();
    I2C_Operation.State &= ~I2C_ST_BUSY; //danger
    
    for(i = 0; i < I2C_Operation.OCH_Amount; i++) {
      (*I2C_Operation.OperationCompleteHandlers[i])(&I2C_Operation);
    }
  }
}

void I2C_ReceiveRegisterNotEmpty()
{
  uint8_t bytesLeft;
  unsigned int i;
  
  I2C_Operation.CurrentByte++;
  I2C_Operation.Bytes[I2C_Operation.CurrentByte - 1] = I2C1->DR;
  bytesLeft = I2C_Operation.Description.BytesNum - I2C_Operation.CurrentByte;
  switch(bytesLeft) {
  case 1:
    //Messenger_SendByte(I2C_MSG_RXNE1);
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    break;
  case 0:
    //Messenger_SendByte(I2C_MSG_RXNE0);
    I2C_Operation.State &= ~I2C_ST_BUSY;
    for(i = 0; i < I2C_Operation.OCH_Amount; i++) {
      (*I2C_Operation.OperationCompleteHandlers[i])(&I2C_Operation);
    }
    break;
  default:
    //Messenger_SendByte(I2C_MSG_RXNE);
    break;
  }
}

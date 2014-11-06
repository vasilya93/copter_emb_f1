#include "stdbool.h"
#include "stdint.h"

#ifndef USART_H
#define USART_H

//apb2 for usart1
//apb1 usart2 - usart5, i2c1, i2c2, tim2 - tim7 (usart3 for me)
//way apb2 prescaler <- ahb prescaler
//now i'll make easier - with default frequency; when i add functions to change frequency -
//i'll add write functions to to change baudrate for USART;

#define USART_9BYTE 0x01
#define USART_8BYTE 0x00

#define USART_PCENABLE 0x02
#define USART_PCDISABLE 0x00

#define USART_PARITY_EVEN 0x00
#define USART_PARITY_ODD 0x04

#define USART_SB1 0x00
#define USART_SB05 0x08
#define USART_SB2 0x10
#define USART_SB15 0x18
#define USART_SB 0x18

#define USART_IT_TXE 0x01
#define USART_IT_TC 0x02
#define USART_IT_RXNE 0x04
#define USART_IT_PE 0x08

void USART_InitStructure(void);

void USART_Begin(unsigned int speed);

void USART_BeginConfigured(uint16_t speed, uint8_t conf);

void USART_End(void);

void USART_WriteByte(uint8_t writtenByte);

bool USART_ByteReceived_Attach(void (*handler)(uint8_t receicedByte));

void USART_ByteReceived_Detach(void);

bool USART_TransRegEmpty_Attach(void (*handler)());

void USART_TransRegEmpty_Detach(void);

bool USART_TransComplete_Attach(void (*handler)());

void USART_TransComplete_Detach(void);

void USART_EnableInterrupts(uint8_t);

void USART_DisableInterrupts(uint8_t);

void USART3_IRQHandler(void);

typedef struct USART_Type
{
  void (**ByteReceivedHandlers)(uint8_t receivedByte);
  uint8_t BRH_Amount;
  
  void (**TransRegEmptyHandlers)();
  uint8_t TREH_Amount;
  
  void (**TransCompleteHandlers)();
  uint8_t TCH_Amount;
  
  bool IsStructureInitialized;
}USART_Type;

#endif

#ifndef I2C_H
#define I2C_H

#define I2C_DATA_SIZE 10

#define I2C_DD_NODATA 0

#define I2C_ST_BUSY 1

//1-30 message codes are for I2C
#define I2C_MSG_STRTST 1
#define I2C_MSG_SBSND 2
#define I2C_MSG_ADSND 3
#define I2C_MSG_TRNSMTR 4
#define I2C_MSG_NACKST 5
#define I2C_MSG_RCVR 6
#define I2C_MSG_TRSCPL 7
#define I2C_MSG_TXE 8
#define I2C_MSG_RXNE 9
#define I2C_MSG_RXNE0 10
#define I2C_MSG_RXNE1 11
#define I2C_MSG_AF 12
#define I2C_MSG_STRTFL 13
#define I2C_MSG_ERR 14

#include "stdbool.h"
#include "stdint.h"

typedef struct I2C_OpDescript_Type
{
  uint16_t DataDescript; 
  uint8_t DeviceAddress;
  bool IsRead;
  uint8_t BytesNum;  
}I2C_OpDescript_Type;

typedef struct I2C_Operation_Type
{
  I2C_OpDescript_Type Description;
  uint8_t State;
  uint8_t* Bytes;  
  uint8_t CurrentByte;
  
  void (**OperationCompleteHandlers)(void* operation);
  uint8_t OCH_Amount;
}I2C_Operation_Type;

void I2C_Begin(void);

bool I2C_OperationComplete_Attach(void (*handler)(void* operation));

bool I2C_StartOperation(I2C_OpDescript_Type, uint8_t*);

void I2C_SetOpDescription(I2C_OpDescript_Type* description,
                         uint16_t dataDescript, uint8_t deviceAddress, 
                         bool isRead, uint8_t bytesNum);

#endif

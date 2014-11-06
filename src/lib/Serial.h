#include "stdint.h"
#include "stdbool.h"
#include "Buffer.h"

#ifndef SERIAL_H
#define SERIAL_H

#define SERIAL_STATE_INITIALIZED 0x01
#define SERIAL_STATE_BUSY 0x02
#define SERIAL_STATE_PATTERNED 0x04

#define NUMBER_AS_STRING_SIZE 10

typedef union WordsToBytes
{
  uint16_t Word;
  uint8_t Bytes[2];
}WordsToBytes;

typedef union _32ToBytes
{
  float RealNum;
  uint32_t Integer;
  uint8_t Bytes[4];
}_32ToBytes;

typedef struct Serial_Type
{
  Buffer_Type InputBuffer;
  Buffer_Type OutputBuffer;
  
  WordsToBytes WordConverter;
  _32ToBytes _32Converter;
  
  uint8_t State;
  
  char* Pattern;
  uint16_t PatternLength;
  
  void (**InBufOverHandlers)();
  int IBOH_Amount;
  
  void (**OutBufOverHandlers)();
  int OBOH_Amount;
  
  void (**ByteReceivedHandlers)(uint8_t);
  int BRH_Amount;
  
  void (**VarReceivedHandlers)(void);
  int VRH_Amount;
}Serial_Type;

void Serial_Begin(unsigned int speed);

void Serial_BeginConfigured(unsigned int speed, uint8_t conf);

void Serial_End(void);

void Serial_WriteByte(uint8_t byte);

void Serial_WriteInt32AsString(uint32_t number);

void Serial_WriteInt16AsString(uint16_t number);

void Serial_WriteInt8AsString(uint8_t number);

void Serial_WriteFloatAsString(float number);

void Serial_WriteInt32(uint32_t number);

void Serial_WriteInt16(uint16_t number);

void Serial_WriteFloat(float number);

void Serial_WriteLine(const char* line);

void Serial_SetReceivePattern(char* pattern);

bool Serial_ByteReceived_Attach(void(*)(uint8_t));

void Serial_ByteReceived_Detach(void);

bool Serial_VariableReceived_Attach(void(*)(void));

void Serial_VariableReceived_Detach(void);

bool Serial_InpBufOver_Attach(void(*)());

void Serial_InpBufOver_Detach(void);

bool Serial_OutBufOver_Attach(void(*)());

void Serial_OutBufOver_Detach(void);

#endif

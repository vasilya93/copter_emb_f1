#include "stdint.h" //define custom types like uint8_t, uint16_t 

#ifndef BUFFER_H
#define BUFFER_H

#define BUFFER_STATUS_FULL 0x01
#define BUFFER_STATUS_EMPTY 0x02
#define BUFFER_SIZE 256

typedef struct Buffer_Type
{
  uint8_t Array[BUFFER_SIZE];
  uint16_t IndexFirst; //index of the first element added into buffer
  uint16_t IndexLast; //index of the last element into buffer
  uint16_t FreeElements;
  uint8_t Status;
}Buffer_Type;

//returns 0 on success, other value if overflow happens
uint8_t Buffer_Push(Buffer_Type *buffer, uint8_t symbol);

uint8_t Buffer_Pop(Buffer_Type *buffer);

void Buffer_Init(Buffer_Type *buffer);

#endif

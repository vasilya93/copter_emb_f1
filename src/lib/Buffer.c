#include "Buffer.h"

void IncrementIndex(uint16_t *index);


uint8_t Buffer_Push(Buffer_Type *buffer, uint8_t symbol)
{
	uint8_t res = 0;
	
  buffer->Array[buffer->IndexLast] = symbol;
  buffer->Status &= ~BUFFER_STATUS_EMPTY;
  IncrementIndex(&buffer->IndexLast);
  if(buffer->Status & BUFFER_STATUS_FULL) {
		res = 1;
    IncrementIndex(&buffer->IndexFirst);
  } else {
    buffer->FreeElements--;
  }
  if(buffer->IndexFirst == buffer->IndexLast) {
    buffer->Status &= BUFFER_STATUS_FULL;
  }
	
	return res;
}

uint8_t Buffer_Pop(Buffer_Type *buffer)
{
	uint8_t symbol = '\0';
  buffer->Status &= ~BUFFER_STATUS_FULL;
  if(buffer->Status & BUFFER_STATUS_EMPTY)
  {
    return symbol;
  }
  else
  {
    buffer->FreeElements++;
    symbol = buffer->Array[buffer->IndexFirst];
    IncrementIndex(&buffer->IndexFirst);
    if(buffer->IndexFirst == buffer->IndexLast)
    {
      buffer->Status |= BUFFER_STATUS_EMPTY;
    }
    return symbol;
  }
}


void Buffer_Init(Buffer_Type *buffer)
{
  buffer->IndexFirst = 0;
  buffer->IndexLast = 0;
  buffer->FreeElements = BUFFER_SIZE;
  buffer->Status = BUFFER_STATUS_EMPTY;
}

//-------------------------Hidden funtions--------------------------------------

void IncrementIndex(uint16_t *index)
{
  *index += 1;
  if(*index == BUFFER_SIZE)
  {
    *index = 0;
  }
}

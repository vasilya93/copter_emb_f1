#include "Serial.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "USART.h"
#include "Buffer.h"
#include "assert.h"

Serial_Type Serial;

void SerialInit(void);
void SerialDeinit(void);
void RunTransmission(void);
void TC_Handler(void);
void TXE_Handler(void);
void RXNE_Handler(uint8_t symbol);

void Serial_Begin(unsigned int speed)
{
  //init buffers and attach events
  SerialInit();
    
  //configure registers
  USART_Begin(speed);
}

void Serial_BeginConfigured(unsigned int speed, uint8_t conf)
{
  //init buffers and attach events
  SerialInit();
    
  //configure registers
  USART_BeginConfigured(speed, conf);
}

void Serial_End()
{
  //there is no need to deinit buffers because deinitialization proc is similar
  //to init and init is accomplished during 'Begin' function
    
  USART_End();
  
  USART_ByteReceived_Detach();
  USART_TransRegEmpty_Detach();
  USART_TransComplete_Detach();  
  
  Serial.State = 0;
}

void Serial_WriteByte(uint8_t writtenByte)
{
	unsigned int i;
	
  if (Buffer_Push(&Serial.OutputBuffer, writtenByte)) {
		for (i = 0; i < Serial.OBOH_Amount; i++)
			Serial.OutBufOverHandlers[i]();
	}
  RunTransmission();
}

void Serial_WriteInt32AsString(uint32_t number)
{
  char* line = calloc(NUMBER_AS_STRING_SIZE + 1, sizeof(char));
  int charsWritten = sprintf(line, "%u", number);  
  //terminating null-char automatically appended
  //on success number of characters written is returned (not acc null-terminator)
  //on failure negative value is returned
  
  if((charsWritten < 0) || (charsWritten > NUMBER_AS_STRING_SIZE))
  {
    while(1);
  }
  
  Serial_WriteLine(line);
  free(line);
}

void Serial_WriteInt16AsString(uint16_t number)
{
  char* line = calloc(NUMBER_AS_STRING_SIZE + 1, sizeof(char));
  int charsWritten = sprintf(line, "%u", number);  
  //terminating null-char automatically appended
  //on success number of characters written is returned (not acc null-terminator)
  //on failure negative value is returned
  
  if((charsWritten < 0) || (charsWritten > NUMBER_AS_STRING_SIZE))
  {
    while(1);
  }
  
  Serial_WriteLine(line);
  free(line);
}

void Serial_WriteInt8AsString(uint8_t number)
{
  char* line = calloc(NUMBER_AS_STRING_SIZE + 1, sizeof(char));
  int charsWritten = sprintf(line, "%u", number);  
  //terminating null-char automatically appended
  //on success number of characters written is returned (not acc null-terminator)
  //on failure negative value is returned
  
  if((charsWritten < 0) || (charsWritten > NUMBER_AS_STRING_SIZE)) {
    while(1);
  }
  
  Serial_WriteLine(line);
  free(line);
}

void Serial_WriteFloatAsString(float number)
{
  char* line = calloc(NUMBER_AS_STRING_SIZE + 1, sizeof(char));
  int charsWritten = sprintf(line, "%f", number);  
  //terminating null-char automatically appended
  //on success number of characters written is returned (not acc null-terminator)
  //on failure negative value is returned
  
  if((charsWritten < 0) || (charsWritten > NUMBER_AS_STRING_SIZE)) {
    while(1);
  }
  
  Serial_WriteLine(line);
  free(line);
}

void Serial_WriteInt32(uint32_t number)
{
	int i, j;
	
  Serial._32Converter.Integer = number;
	for (i = 3; i >= 0; i--) {
		if (Buffer_Push(&Serial.OutputBuffer, Serial._32Converter.Bytes[i])) {
			for (j = 0; j < Serial.OBOH_Amount; j++)
				Serial.OutBufOverHandlers[j]();
		}
	}
  
  RunTransmission();
}

void Serial_WriteInt16(uint16_t number)
{
	int i, j;
	
  Serial.WordConverter.Word = number;
	
	for (i = 1; i >= 0; i--) {
		if (Buffer_Push(&Serial.OutputBuffer, Serial.WordConverter.Bytes[i])) {
			for (j = 0; j < Serial.OBOH_Amount; j++)
				Serial.OutBufOverHandlers[j]();
		}
	}
  
  RunTransmission();
}

void Serial_WriteFloat(float number)
{
	int i, j;
	
  Serial._32Converter.RealNum = number;
	for (i = 3; i >= 0; i--) {
		if (Buffer_Push(&Serial.OutputBuffer, Serial._32Converter.Bytes[i])) {
			for (j = 0; j < Serial.OBOH_Amount; j++)
				Serial.OutBufOverHandlers[j]();
		}
	}
  
  RunTransmission();
}

void Serial_WriteLine(const char* line)
{
  unsigned int j;
	uint16_t i = 0;
	
  while(line[i] != '\0')
  {
    if (Buffer_Push(&Serial.OutputBuffer, line[i])) {
			for (j = 0; j < Serial.OBOH_Amount; j++)
				Serial.OutBufOverHandlers[j]();
		}
    i++;
  }
  
  RunTransmission();
}

void Serial_SetReceivePattern(char* pattern)
{
  Serial.Pattern = pattern;
  Serial.PatternLength = strlen(Serial.Pattern);
  Serial.State |= SERIAL_STATE_PATTERNED;
}

bool Serial_ByteReceived_Attach(void(*handler)(uint8_t))
{
	void (**newPointer)(uint8_t receivedByte);
	
  if(handler == NULL)
  {
    return false;
  }
  
  Serial.BRH_Amount++;
  newPointer = (void (**)(uint8_t))realloc(Serial.ByteReceivedHandlers, Serial.BRH_Amount*sizeof(void(*)(uint8_t)));
  
  if(newPointer == NULL)
  {
    Serial.BRH_Amount--;
    return false;
  }
  
  Serial.ByteReceivedHandlers = newPointer;
  Serial.ByteReceivedHandlers[Serial.BRH_Amount - 1] = handler;
  
  return true;
}

void Serial_ByteReceived_Detach()
{
  free(Serial.ByteReceivedHandlers);
  Serial.ByteReceivedHandlers = NULL;
  Serial.BRH_Amount = 0;
}

bool Serial_VariableReceived_Attach(void(*handler)(void))
{
	void (**newPointer)(void);
	
  if(handler == NULL)
  {
    return false;
  }
  
  Serial.VRH_Amount++;
  newPointer = (void (**)(void))realloc(Serial.VarReceivedHandlers, Serial.VRH_Amount*sizeof(void(*)(uint8_t)));
  
  if(newPointer == NULL)
  {
    Serial.VRH_Amount--;
    return false;
  }
  
  Serial.VarReceivedHandlers = newPointer;
  Serial.VarReceivedHandlers[Serial.VRH_Amount - 1] = handler;
  
  return true;
}

void Serial_VariableReceived_Detach()
{
  free(Serial.VarReceivedHandlers);
  Serial.VarReceivedHandlers = NULL;
  Serial.VRH_Amount = 0;
}

bool Serial_InpBufOver_Attach(void(*handler)())
{
	void (**newPointer)();
	
  if(handler == NULL)
  {
    return false;
  }
  
  Serial.IBOH_Amount++;
  newPointer = (void (**)())realloc(Serial.InBufOverHandlers, Serial.IBOH_Amount*sizeof(void(*)()));
  
  if(newPointer == NULL)
  {
    Serial.IBOH_Amount--;
    return false;
  }
  
  Serial.InBufOverHandlers = newPointer;
  Serial.InBufOverHandlers[Serial.IBOH_Amount - 1] = handler;
  
  return true;
}

void Serial_InpBufOver_Detach()
{
  free(Serial.InBufOverHandlers);
  Serial.InBufOverHandlers = NULL;
  Serial.IBOH_Amount = 0;
}

bool Serial_OutBufOver_Attach(void(*handler)())
{
	void (**newPointer)();
		
  if(handler == NULL)
  {
    return false;
  }
  
  Serial.OBOH_Amount++;
  newPointer = (void (**)())realloc(Serial.OutBufOverHandlers, Serial.OBOH_Amount*sizeof(void(*)()));
  
  if(newPointer == NULL)
  {
    Serial.OBOH_Amount--;
    return false;
  }
  
  Serial.OutBufOverHandlers = newPointer;
  Serial.OutBufOverHandlers[Serial.OBOH_Amount - 1] = handler;
  
  return true;
}

void Serial_OutBufOver_Detach()
{
  free(Serial.OutBufOverHandlers);
  Serial.OutBufOverHandlers = NULL;
  Serial.OBOH_Amount = 0;
}

//-------------------------Hidden funtions--------------------------------------

void SerialInit()
{
  //attach events; lower level initializing
  USART_InitStructure();
  if(!USART_TransComplete_Attach(&TC_Handler))
  {
    while(1);
  }
  
  if(!USART_TransRegEmpty_Attach(&TXE_Handler))
  {
    while(1);
  }
  
  if(!USART_ByteReceived_Attach(&RXNE_Handler))
  {
    while(1);
  }
  
  //initialize Serial object
  Buffer_Init(&Serial.InputBuffer);
  Buffer_Init(&Serial.OutputBuffer);
  
  Serial_ByteReceived_Detach();
  Serial_VariableReceived_Detach();
  Serial_InpBufOver_Detach();
  Serial_OutBufOver_Detach();
  
  free(Serial.Pattern);
  Serial.Pattern = NULL;
  Serial.PatternLength = 0;
  
  Serial.State = SERIAL_STATE_INITIALIZED;
}

void RunTransmission()
{
  //it is necessary to check if the buffer is not empty because an interrupt
  //can take away the byte since the previous operation
  if((~Serial.State & SERIAL_STATE_BUSY) && (~Serial.OutputBuffer.Status && BUFFER_STATUS_EMPTY))
  {
    //USART_WriteByte(Buffer_Pop(&Serial.OutputBuffer));
    //TXE bit in SR is somehow set sot TXEI will handle the bytes in the buffer
    USART_EnableInterrupts(USART_IT_TXE);
  }
}

void TC_Handler()
{
  if(~Serial.OutputBuffer.Status & BUFFER_STATUS_EMPTY)
  {
    USART_EnableInterrupts(USART_IT_TXE);
    USART_WriteByte(Buffer_Pop(&Serial.OutputBuffer));
  }
  else
  {
    Serial.State &= ~SERIAL_STATE_BUSY;
  }
}

void TXE_Handler()
{
  if(~Serial.OutputBuffer.Status & BUFFER_STATUS_EMPTY)
  {
    USART_WriteByte(Buffer_Pop(&Serial.OutputBuffer));
  }
  else
  {
    USART_DisableInterrupts(USART_IT_TXE);
  }
}

void RXNE_Handler(uint8_t symbol)
{
	unsigned int i;
  for(i = 0; i < Serial.BRH_Amount; i++) {
    Serial.ByteReceivedHandlers[i](symbol);
  }
  if(Serial.State & SERIAL_STATE_PATTERNED) {
  }
}


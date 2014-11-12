#ifndef WIRE_H
#define WIRE_H

#include <stdint.h>
#include <stdbool.h>
#include "I2C.h"

#define WIRE_DD_NODATA 0

//1-30 message codes are for I2C
#define I2C_MSG_STRTST 1

#define WIRE_ST_DEVSINIT 1

typedef struct Wire_Device_Type
{
  uint8_t* (*GetNextInitOperation)(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript);
  uint8_t* (*GetNextRegOperation)(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript);
  void (*ProcessOperationResult)(I2C_Operation_Type* operation);
}Wire_Device_Type;

typedef struct Wire_Type
{
  Wire_Device_Type** Devices;
  uint8_t DevicesNum;
  int16_t CurrentDevice;
	uint8_t state;
}Wire_Type;

bool Wire_AttachDevice(Wire_Device_Type*);

void Wire_Initialize(void);

void Wire_InitDevices(void);

void Wire_BeginCycle(void);

#endif

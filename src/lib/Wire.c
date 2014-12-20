#include <stdlib.h>
#include "Wire.h"
#include "I2C.h"

Wire_Type Wire;

void OperationCompleteHandler(void* operation);

bool Wire_AttachDevice(Wire_Device_Type* newDevice)
{
  Wire_Device_Type** newPointer;
	
  if(newDevice == NULL) {
    return false;
  }
  
  Wire.DevicesNum++;
  newPointer = (Wire_Device_Type**)realloc(Wire.Devices, Wire.DevicesNum*sizeof(Wire_Device_Type*));
  
  if(newPointer == NULL) {
    Wire.DevicesNum--;
    return false;
  }
  
  Wire.Devices = newPointer;
  Wire.Devices[Wire.DevicesNum - 1] = newDevice;
  
  return true;
}

void Wire_Initialize()
{
  Wire.DevicesNum = 0;
  Wire.CurrentDevice = -1;
	Wire.state = 0;
  I2C_Begin();
  I2C_OperationComplete_Attach(&OperationCompleteHandler);
}

void Wire_InitDevices(void)
{
  I2C_OpDescript_Type opDescript;
  uint8_t* data;
  
  Wire.state |= WIRE_ST_DEVSINIT;
	
  if(Wire.DevicesNum == 0)
		return;
	
  Wire.CurrentDevice = 0;
  while(Wire.CurrentDevice < Wire.DevicesNum) {
    data = Wire.Devices[Wire.CurrentDevice]->GetNextInitOperation(I2C_DD_NODATA,
                                                                  &opDescript);		
    if(opDescript.DataDescript != WIRE_DD_NODATA) {			
      I2C_StartOperation(opDescript, data);
      break;
    }
    Wire.CurrentDevice++;
  }
}

void Wire_BeginCycle(void)
{
  I2C_OpDescript_Type opDescript;
  uint8_t* data;
	
  if(Wire.DevicesNum == 0)
    return;
	
  Wire.CurrentDevice = 0;
  while(Wire.CurrentDevice < Wire.DevicesNum) {
    data = Wire.Devices[Wire.CurrentDevice]->GetNextRegOperation(I2C_DD_NODATA, &opDescript);		
    if(opDescript.DataDescript != WIRE_DD_NODATA) {
      I2C_StartOperation(opDescript, data);
      break;		
    }
    Wire.CurrentDevice++;
  }
}

void OperationCompleteHandler(void* operation)
{
  uint8_t* data;
  I2C_OpDescript_Type opDescript;

  I2C_Operation_Type* operationCasted = (I2C_Operation_Type*)operation;
  Wire.Devices[Wire.CurrentDevice]->ProcessOperationResult(operationCasted);

  while(Wire.CurrentDevice < Wire.DevicesNum) {
    if (Wire.state & WIRE_ST_DEVSINIT)
      data = Wire.Devices[Wire.CurrentDevice]->GetNextInitOperation(operationCasted->Description.DataDescript,
                                                                    &opDescript);
    else
      data = Wire.Devices[Wire.CurrentDevice]->GetNextRegOperation(operationCasted->Description.DataDescript,
                                                                   &opDescript);
    
    if (opDescript.DataDescript == WIRE_DD_NODATA) {
            free(data);
            Wire.CurrentDevice++;
            operationCasted->Description.DataDescript = WIRE_DD_NODATA;
            continue;
    }
    
    I2C_StartOperation(opDescript, data);
    return;
  }
  
  Wire.CurrentDevice = -1;
  Wire.state &= ~WIRE_ST_DEVSINIT;
}

#include "HMC5883.h"
#include "Wire.h"
#include "Messenger.h"
#include "stdlib.h"

Wire_Device_Type HMC5883;

void HMC5883_initialize()
{
  HMC5883.GetNextInitOperation = &HMC5883_get_next_init_op;
  HMC5883.GetNextRegOperation = &HMC5883_get_next_reg_op;
  HMC5883.ProcessOperationResult = &HMC5883_process_op_res;

  Wire_AttachDevice(&HMC5883);
}

uint8_t*
HMC5883_get_next_init_op(uint16_t currentDataDescript,
                         I2C_OpDescript_Type* opDescript)
{
  uint8_t *data;

  switch(currentDataDescript) {
    case WIRE_DD_NODATA:
      I2C_SetOpDescription(opDescript, HMC5883_DD_IDA_REQ, HMC5883_ADDRESS, false, 1);
      data = (uint8_t*) malloc(1);
      data[0] = HMC5883_RA_IDA;
      return data;
    case HMC5883_DD_IDA_REQ:
      I2C_SetOpDescription(opDescript, HMC5883_DD_IDA_DATA, HMC5883_ADDRESS, true, 1);
      return NULL;
    default:
      Messenger_SendByte(HMC5883_MSG_INIT_COMP);
      I2C_SetOpDescription(opDescript, WIRE_DD_NODATA, 0, false, 0);
      return NULL;
  }
}

uint8_t* HMC5883_get_next_reg_op(uint16_t currentDataDescript,
                                I2C_OpDescript_Type* opDescript)
{
  uint8_t *data;

  switch(currentDataDescript) {
    case WIRE_DD_NODATA:
      I2C_SetOpDescription(opDescript, HMC5883_DD_IDA_REQ, HMC5883_ADDRESS, false, 1);
      data = (uint8_t*) malloc(1);
      data[0] = HMC5883_RA_IDA;
      return data;
    case HMC5883_DD_IDA_REQ:
      I2C_SetOpDescription(opDescript, HMC5883_DD_IDA_DATA, HMC5883_ADDRESS, true, 1);
      return NULL;
    default:
      I2C_SetOpDescription(opDescript, WIRE_DD_NODATA, 0, false, 0);
      return NULL;
  }
}

void HMC5883_process_op_res(I2C_Operation_Type* operation)
{
  switch(operation->Description.DataDescript) {
    case HMC5883_DD_IDA_DATA:
      Messenger_SendWord((uint16_t)operation->Bytes[0], MSNR_DD_HMC5883_IDA);
      break;
    default:
      break;
  }
}
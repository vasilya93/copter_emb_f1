#include "MPU6050.h"
#include "Wire.h"
#include "I2C.h"
#include "stdlib.h"
#include "Serial.h"
#include "Messenger.h"
#include "settings.h"

#define MPU6050_ACCELX_RENEWED	0x01
#define MPU6050_ACCELY_RENEWED	0x02
#define MPU6050_ACCELZ_RENEWED	0x04
#define MPU6050_ACCEL_RENEWED		0x07

#define MPU6050_GYROX_RENEWED		0x08
#define MPU6050_GYROY_RENEWED		0x10
#define MPU6050_GYROZ_RENEWED		0x20
#define MPU6050_GYRO_RENEWED		0x38

MPU6050_Data_Type MPU6050_Data={
  0, 0,
  0, 0, 0,
  0, 0, 0,
  NULL, 0,
  NULL, 0
};

Wire_Device_Type MPU6050;

void set_gyro_val(int16_t value, coordinate_t coord);
void set_accel_val(int16_t value, coordinate_t coord);
    
void accel_check_renew_state(void);
void gyro_check_renew_state(void);

void MPU6050_Initialize(void)
{  
  MPU6050.GetNextInitOperation = &MPU6050_GetNextInitOperation;
  MPU6050.GetNextRegOperation = &MPU6050_GetNextRegOperation;
  MPU6050.ProcessOperationResult = &MPU6050_ProcessOperationResult;
  
  Wire_AttachDevice(&MPU6050);
}

uint8_t* MPU6050_GetNextInitOperation(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript)
{
  uint8_t *data;
  
  switch(currentDataDescript) {
  case WIRE_DD_NODATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SMPRTDIVDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SMPRTDIV;
    data[1] = 0x07;
    return data;
  case MPU6050_DD_SMPRTDIVDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_CONFIGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_CONFIG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_CONFIGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROCONFIGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_GYROCONFIG;
    data[1] = 0x00; //0x00 - 250 deg/s
                    //0x08 - 500 deg/s
                    //0x10 - 1000 deg/s
                    //0x18 - 2000 deg/s
    return data;
  case MPU6050_DD_GYROCONFIGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELCONFIGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_ACCELCONFIG;
    data[1] = 0x00; //0x00 - +-2g
                    //0x08 - +-4g
                    //0x10 - +-8g
                    //0x18 - +-16g
    return data;
  case MPU6050_DD_ACCELCONFIGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_MOTTHRDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_MOTTHR;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_MOTTHRDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_FIFOENDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_FIFOEN;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_FIFOENDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_MSTCTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_MSTCTRL;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_MSTCTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV0ADDRDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV0ADDR;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV0ADDRDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV0REGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV0REG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV0REGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV0CTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV0CTRL;
    data[1] = 0x00;
    return data;	
  case MPU6050_DD_SLV0CTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV1ADDRDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV1ADDR;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV1ADDRDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV1REGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV1REG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV1REGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV1CTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV1CTRL;
    data[1] = 0x00;
    return data;	
  case MPU6050_DD_SLV1CTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV2ADDRDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV2ADDR;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV2ADDRDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV2REGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV2REG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV2REGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV2CTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV2CTRL;
    data[1] = 0x00;
    return data;	
  case MPU6050_DD_SLV2CTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV3ADDRDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV3ADDR;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV3ADDRDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV3REGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV3REG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV3REGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV3CTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV3CTRL;
    data[1] = 0x00;
    return data;	
  case MPU6050_DD_SLV3CTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV4ADDRDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV4ADDR;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV4ADDRDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV4REGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV4REG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV4REGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV4DODATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV4DO;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV4DODATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV4CTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV4CTRL;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV4CTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SLV4DIDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SLV4DI;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SLV4DIDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_INTPINCFGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_INTPINCFG;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_INTPINCFGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_INTENABLEDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_INTENABLE;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_INTENABLEDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_MSTDELAYCTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_MSTDELAYCTRL;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_MSTDELAYCTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_SIGNALPATHRESETDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_SIGNALPATHRESET;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_SIGNALPATHRESETDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_MOTDETECTCTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_MOTDETECTCTRL;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_MOTDETECTCTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_USERCTRLDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_USERCTRL;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_USERCTRLDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT1DATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_PWRMGMT1;
    data[1] = 0x00;
    return data;
  case MPU6050_DD_PWRMGMT1DATAIN: //32 instructions
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT2DATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_PWRMGMT2;
    data[1] = 0x00;
    return data;
  default:
    Messenger_SendByte(MPU6050_MSG_INIT_COMP);
    I2C_SetOpDescription(opDescript, WIRE_DD_NODATA, 0, false, 0);
    return NULL;
  }
}

uint8_t* MPU6050_GetNextRegOperation(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript)
{
  uint8_t *data;

  switch(currentDataDescript) {
    /*case WIRE_DD_NODATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_WHOAMIREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_WHOAMI;
    return data;
  case MPU6050_DD_WHOAMIREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_WHOAMIDATA, MPU6050_ADDRESS, true, 1);
    return NULL;*/
    
    /*case WIRE_DD_NODATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT1DATAREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_PWRMGMT1;
    return data;
  case MPU6050_DD_PWRMGMT1DATAREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT1DATAOUT, MPU6050_ADDRESS, true, 1);
    return NULL;*/

  case WIRE_DD_NODATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELXREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_ACCELXH;
    return data;
  case MPU6050_DD_ACCELXREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELXDATA, MPU6050_ADDRESS, true, 2);
    return NULL;
  case MPU6050_DD_ACCELXDATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELYREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_ACCELYH;
    return data;
  case MPU6050_DD_ACCELYREQ:
    //Messenger_SendByte(MPU6050_MSG_GYROXGIVEN);
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELYDATA, MPU6050_ADDRESS, true, 2);
    return NULL;
  case MPU6050_DD_ACCELYDATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELZREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_ACCELZH;
    return data;
  case MPU6050_DD_ACCELZREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELZDATA, MPU6050_ADDRESS, true, 2);
    return NULL;
  case MPU6050_DD_ACCELZDATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROXREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_GYROXH;
    return data;
  case MPU6050_DD_GYROXREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROXDATA, MPU6050_ADDRESS, true, 2);
    return NULL; // to send message before and after the suspected event
  case MPU6050_DD_GYROXDATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROYREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_GYROYH;
    return data;
  case MPU6050_DD_GYROYREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROYDATA, MPU6050_ADDRESS, true, 2);
    return NULL;
  case MPU6050_DD_GYROYDATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROZREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_GYROZH;
    return data;
  case MPU6050_DD_GYROZREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_GYROZDATA, MPU6050_ADDRESS, true, 2);
    return NULL;
  default:
    I2C_SetOpDescription(opDescript, WIRE_DD_NODATA, 0, false, 0);
    return NULL;
  }
}

void MPU6050_ProcessOperationResult(I2C_Operation_Type* operation)
{
  switch(operation->Description.DataDescript) {
  case MPU6050_DD_PWRMGMT1DATAOUT:
    MPU6050_Data.tmp = operation->Bytes[0];
    Messenger_SendWord((uint16_t)MPU6050_Data.tmp, MSNR_DD_PWRMGMT1);
    break;
  case MPU6050_DD_WHOAMIDATA:
    MPU6050_Data.tmp = operation->Bytes[0];
    Messenger_SendByte(MPU6050_MSG_WHOAMI);
    Messenger_SendWord((uint16_t)MPU6050_Data.tmp, MSNR_DD_WHOAMI);
    break;
  case MPU6050_DD_GYROCONFIGDATAOUT:
    MPU6050_Data.tmp = operation->Bytes[0];
    Messenger_SendWord((uint16_t)MPU6050_Data.tmp, MSNR_DD_WHOAMI);
    break;
  case MPU6050_DD_ACCELXDATA:
    set_accel_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_X);
    break;
  case MPU6050_DD_ACCELYDATA:
    set_accel_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Y);
    break;
  case MPU6050_DD_ACCELZDATA:
    set_accel_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Z);
    break;
  case MPU6050_DD_GYROXDATA:
    set_gyro_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_X);
    break;
  case MPU6050_DD_GYROYDATA:
    set_gyro_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Y);
    break;
  case MPU6050_DD_GYROZDATA:
    set_gyro_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Z);
    break;
  default:
    break;
  }
}

void set_gyro_val(int16_t value, coordinate_t coord)
{  
  switch (coord) {
  case COORDINATE_X:
#ifdef SEND_RAW_GYRO
    Messenger_SendWord(MPU6050_Data.gyrox, MSNR_DD_ANGSPEEDX);
#endif
    MPU6050_Data.gyrox = value;
    MPU6050_Data.renew_state |= MPU6050_GYROX_RENEWED;
    gyro_check_renew_state();
    break;
  case COORDINATE_Y:
#ifdef SEND_RAW_GYRO
    Messenger_SendWord(MPU6050_Data.gyroy, MSNR_DD_ANGSPEEDY);
#endif
    MPU6050_Data.gyroy = value;
    MPU6050_Data.renew_state |= MPU6050_GYROY_RENEWED;
    gyro_check_renew_state();
    break;
  case COORDINATE_Z:
#ifdef SEND_RAW_GYRO
    Messenger_SendWord(MPU6050_Data.gyroz, MSNR_DD_ANGSPEEDZ);
#endif
    MPU6050_Data.gyroz = value;
    MPU6050_Data.renew_state |= MPU6050_GYROZ_RENEWED;
    gyro_check_renew_state();
    break;
  }
}

void set_accel_val(int16_t value, coordinate_t coord)
{
  switch (coord) {
  case COORDINATE_X:
#ifdef SEND_RAW_ACC
    Messenger_SendWord(MPU6050_Data.accelx, MSNR_DD_ACCELX);
#endif
    MPU6050_Data.accelx = value;
    MPU6050_Data.renew_state |= MPU6050_ACCELX_RENEWED;
    accel_check_renew_state();
    break;
  case COORDINATE_Y:
#ifdef SEND_RAW_ACC
    Messenger_SendWord(MPU6050_Data.accely, MSNR_DD_ACCELY);
#endif
    MPU6050_Data.accely = value;
    MPU6050_Data.renew_state |= MPU6050_ACCELY_RENEWED;
    accel_check_renew_state();
    break;
  case COORDINATE_Z:
#ifdef SEND_RAW_ACC
    Messenger_SendWord(MPU6050_Data.accelz, MSNR_DD_ACCELZ);
#endif
    MPU6050_Data.accelz = value;
    MPU6050_Data.renew_state |= MPU6050_ACCELZ_RENEWED;
    accel_check_renew_state();
    break;
  }
}

void accel_check_renew_state()
{
  unsigned int i;
  if ((MPU6050_Data.renew_state & MPU6050_ACCELX_RENEWED) &&
      (MPU6050_Data.renew_state & MPU6050_ACCELY_RENEWED) &&
      (MPU6050_Data.renew_state & MPU6050_ACCELZ_RENEWED)) {
          MPU6050_Data.renew_state &= ~MPU6050_ACCEL_RENEWED;
          for (i = 0; i < MPU6050_Data.accel_handlers_num; i++) {
            (*MPU6050_Data.accel_handlers[i])(MPU6050_Data.accelx,
                                              MPU6050_Data.accely,
                                              MPU6050_Data.accelz);
          }
	}
}

void gyro_check_renew_state()
{
  unsigned int i;
  if ((MPU6050_Data.renew_state & MPU6050_GYROX_RENEWED) &&
      (MPU6050_Data.renew_state & MPU6050_GYROY_RENEWED) &&
        (MPU6050_Data.renew_state & MPU6050_GYROZ_RENEWED)) {
          MPU6050_Data.renew_state &= ~MPU6050_GYRO_RENEWED;
          for (i = 0; i < MPU6050_Data.gyro_handlers_num; i++) {
            (*MPU6050_Data.gyro_handlers[i])(MPU6050_Data.gyrox,
                                             MPU6050_Data.gyroy,
                                             MPU6050_Data.gyroz);
          }
	}
}

int MPU6050_attach_accel_handler(void (*new_handler)(int16_t, int16_t, int16_t))
{
  void (**new_pointer)(int16_t, int16_t, int16_t);
  
  if(new_handler == NULL) {
    return -1;
  }
  
  //(void (**)(int16_t, int16_t, int16_t))
  MPU6050_Data.accel_handlers_num++;
  new_pointer = realloc(MPU6050_Data.accel_handlers,
                        MPU6050_Data.accel_handlers_num * sizeof(void(*)(int16_t, int16_t, int16_t)));
  
  if(new_pointer == NULL) {
    MPU6050_Data.accel_handlers_num--;
    return -1;
  }
  
  MPU6050_Data.accel_handlers = new_pointer;
  MPU6050_Data.accel_handlers[MPU6050_Data.accel_handlers_num - 1] = new_handler;
  
  return 0;
}

int MPU6050_attach_gyro_handler(void (*new_handler)(int16_t, int16_t, int16_t))
{
  void (**new_pointer)(int16_t, int16_t, int16_t);
  
  if(new_handler == NULL) {
    return -1;
  }
  
  MPU6050_Data.gyro_handlers_num++;
  new_pointer = realloc(MPU6050_Data.gyro_handlers,
                        MPU6050_Data.gyro_handlers_num * sizeof(void(*)(int16_t, int16_t, int16_t)));
  
  if(new_pointer == NULL) {
    MPU6050_Data.gyro_handlers_num--;
    return -1;
  }
  
  MPU6050_Data.gyro_handlers = new_pointer;
  MPU6050_Data.gyro_handlers[MPU6050_Data.gyro_handlers_num - 1] = new_handler;
  
  return 0;
}

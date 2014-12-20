#include "MPU6050.h"
#include "Wire.h"
#include "I2C.h"
#include "stdlib.h"
#include "Serial.h"
#include "Messenger.h"

#define MPU6050_ACCELX_RENEWED	0x01
#define MPU6050_ACCELY_RENEWED	0x02
#define MPU6050_ACCELZ_RENEWED	0x04
#define MPU6050_ACCEL_RENEWED		0x07

#define MPU6050_GYROX_RENEWED		0x08
#define MPU6050_GYROY_RENEWED		0x10
#define MPU6050_GYROZ_RENEWED		0x20
#define MPU6050_GYRO_RENEWED		0x38

MPU6050_Data_Type MPU6050_Data={
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  NULL, 0,
  NULL, 0
};

Wire_Device_Type MPU6050;

void add_gyro_val(int16_t value, coordinate_t coord);
void set_gyro_val(int16_t value, coordinate_t coord);
void gyro_calibrate(int16_t value, coordinate_t coord);

void add_accel_val(int16_t value, coordinate_t coord);
void set_accel_val(int16_t value, coordinate_t coord);
void accel_calibrate(int16_t value, coordinate_t coord);
void accel_check_renew_state(void);
void gyro_check_renew_state(void);

void MPU6050_Initialize(uint8_t startup_type)
{
  MPU6050_Data.state |= (startup_type & (MPU6050_ST_CALIBRATE_ACCEL | MPU6050_ST_CALIBRATE_GYRO));
  
  MPU6050.GetNextInitOperation = &MPU6050_GetNextInitOperation;
  MPU6050.GetNextRegOperation = &MPU6050_GetNextRegOperation;
  MPU6050.ProcessOperationResult = &MPU6050_ProcessOperationResult;
  
  Wire_AttachDevice(&MPU6050);
}

uint8_t* MPU6050_GetNextInitOperation(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript)
{
  uint8_t *data;
  
  switch(currentDataDescript) {
    /*case WIRE_DD_NODATA:
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT1DATAREQ, MPU6050_ADDRESS, false, 1);
    data = (uint8_t*) malloc(1);
    data[0] = MPU6050_RA_PWRMGMT1;
    return data;
  case MPU6050_DD_PWRMGMT1DATAREQ:
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT1DATAOUT, MPU6050_ADDRESS, true, 1);
    return NULL;
  case MPU6050_DD_PWRMGMT1DATAOUT:
    I2C_SetOpDescription(opDescript, MPU6050_DD_PWRMGMT1DATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_PWRMGMT1;
    data[1] = 0;
    return data;*/
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
    data[1] = 0x08; //500 deg/s
    return data;
  case MPU6050_DD_GYROCONFIGDATAIN:
    I2C_SetOpDescription(opDescript, MPU6050_DD_ACCELCONFIGDATAIN, MPU6050_ADDRESS, false, 2);
    data = (uint8_t*) malloc(2);
    data[0] = MPU6050_RA_ACCELCONFIG;
    data[1] = 0x00;
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
    
    if (MPU6050_Data.tmp == 0x40)
      Messenger_SendByte(MPU6050_MSG_PWR1_BAD);
    else if (MPU6050_Data.tmp == 0x00)
      Messenger_SendByte(MPU6050_MSG_PWR1_GOOD);
    else
      Messenger_SendByte(MPU6050_MSG_PWR1);
    
    Messenger_SendWord((uint16_t)MPU6050_Data.tmp, MSNR_DD_PWRMGMT1);
    break;
  case MPU6050_DD_WHOAMIDATA:
    MPU6050_Data.tmp = operation->Bytes[0];
    Messenger_SendByte(MPU6050_MSG_WHOAMI);
    Messenger_SendWord((uint16_t)MPU6050_Data.tmp, MSNR_DD_WHOAMI);
    break;
  case MPU6050_DD_ACCELXDATA:
    add_accel_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_X);
    break;
  case MPU6050_DD_ACCELYDATA:
    add_accel_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Y);
    break;
  case MPU6050_DD_ACCELZDATA:
    //Messenger_SendByte(MPU6050_MSG_GYROXPROC);
    add_accel_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Z);
    break;
  case MPU6050_DD_GYROXDATA:
    add_gyro_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_X);
    break;
  case MPU6050_DD_GYROYDATA:
    add_gyro_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Y);
    break;
  case MPU6050_DD_GYROZDATA:
    Messenger_SendByte(MPU6050_MSG_CYCLECOMP);
    add_gyro_val((operation->Bytes[0] << 8) | operation->Bytes[1], COORDINATE_Z);
    break;
  default:
    break;
  }
}

void add_gyro_val(int16_t value, coordinate_t coord)
{
  uint8_t coord_is_calibrated;
  switch (coord) {
  case COORDINATE_X:
    coord_is_calibrated = MPU6050_ST_GRXCBTD;
    break;
  case COORDINATE_Y:
    coord_is_calibrated = MPU6050_ST_GRYCBTD;
    break;
  case COORDINATE_Z:
    coord_is_calibrated = MPU6050_ST_GRZCBTD;
    break;
  }
  
  if ((MPU6050_Data.state & coord_is_calibrated) ||
      !(MPU6050_Data.state & MPU6050_ST_CALIBRATE_GYRO)) {
        set_gyro_val(value, coord);
        return;
      }
  
  gyro_calibrate(value, coord);
}

void set_gyro_val(int16_t value, coordinate_t coord)
{
  int16_t *gyro;
  int16_t *gyro_zero;
  uint8_t data_descr;
  uint8_t coord_renewed = 0;
  
  switch (coord) {
  case COORDINATE_X:
    gyro = &MPU6050_Data.gyrox;
    gyro_zero = &MPU6050_Data.gyrox_zero;
    data_descr = MSNR_DD_ANGSPEEDX;
    coord_renewed = MPU6050_GYROX_RENEWED;
    break;
  case COORDINATE_Y:
    gyro = &MPU6050_Data.gyroy;
    gyro_zero = &MPU6050_Data.gyroy_zero;
    data_descr = MSNR_DD_ANGSPEEDY;
    coord_renewed = MPU6050_GYROY_RENEWED;
    break;
  case COORDINATE_Z:
    gyro = &MPU6050_Data.gyroz;
    gyro_zero = &MPU6050_Data.gyroz_zero;
    data_descr = MSNR_DD_ANGSPEEDZ;
    coord_renewed = MPU6050_GYROZ_RENEWED;
    break;
  }
  
  *gyro = value - *gyro_zero;
  Messenger_SendWord(*gyro, data_descr);
  MPU6050_Data.renew_state |= coord_renewed;
  gyro_check_renew_state();
}

void gyro_calibrate(int16_t value, coordinate_t coord)
{
  int16_t *gyro_zero;
  int32_t *accum;
  uint16_t *counter;
  uint8_t data_descr;
  uint8_t coord_is_calibrated;
  
  switch (coord) {
  case COORDINATE_X:
    gyro_zero = &MPU6050_Data.gyrox_zero;
    accum = &MPU6050_Data.gyrox_accum;
    counter = &MPU6050_Data.x_counter;
    data_descr = MSNR_DD_GYROXOFF;
    coord_is_calibrated = MPU6050_ST_GRXCBTD;
    break;
  case COORDINATE_Y:
    gyro_zero = &MPU6050_Data.gyroy_zero;
    accum = &MPU6050_Data.gyroy_accum;
    counter = &MPU6050_Data.y_counter;
    data_descr = MSNR_DD_GYROYOFF;
    coord_is_calibrated = MPU6050_ST_GRYCBTD;
    break;
  case COORDINATE_Z:
    gyro_zero = &MPU6050_Data.gyroz_zero;
    accum = &MPU6050_Data.gyroz_accum;
    counter = &MPU6050_Data.z_counter;
    data_descr = MSNR_DD_GYROZOFF;
    coord_is_calibrated = MPU6050_ST_GRZCBTD;
    break;
  }
  
  *accum += value;
  if (++(*counter) >= MPU6050_CALIBRATION_CYCLES) {
    MPU6050_Data.state |= coord_is_calibrated;
    *gyro_zero = *accum / *counter;
    Messenger_SendWord(*gyro_zero, data_descr);
  }
}

void add_accel_val(int16_t value, coordinate_t coord)
{
  uint8_t coord_is_calibrated;
  switch (coord) {
  case COORDINATE_X:
    coord_is_calibrated = MPU6050_ST_ACCXCBTD;
    break;
  case COORDINATE_Y:
    coord_is_calibrated = MPU6050_ST_ACCYCBTD;
    break;
  case COORDINATE_Z:
    coord_is_calibrated = MPU6050_ST_ACCZCBTD;
    break;
  }
  
  if ((MPU6050_Data.state & coord_is_calibrated) ||
      !(MPU6050_Data.state & MPU6050_ST_CALIBRATE_ACCEL)) {
        set_accel_val(value, coord);
        return;
      }
  
  accel_calibrate(value, coord);
}

void set_accel_val(int16_t value, coordinate_t coord)
{
  int16_t *accel;
  int16_t *accel_zero;
  uint8_t data_descr;
  uint8_t coord_renewed;
  
  switch (coord) {
  case COORDINATE_X:
    accel = &MPU6050_Data.accelx;
    accel_zero = &MPU6050_Data.accelx_zero;
    data_descr = MSNR_DD_ACCELX;
    coord_renewed = MPU6050_ACCELX_RENEWED;
    break;
  case COORDINATE_Y:
    accel = &MPU6050_Data.accely;
    accel_zero = &MPU6050_Data.accely_zero;
    data_descr = MSNR_DD_ACCELY;
    coord_renewed = MPU6050_ACCELY_RENEWED;
    break;
  case COORDINATE_Z:
    accel = &MPU6050_Data.accelz;
    accel_zero = &MPU6050_Data.accelz_zero;
    data_descr = MSNR_DD_ACCELZ;
    coord_renewed = MPU6050_ACCELZ_RENEWED;
    break;
  }
  
  *accel = value - *accel_zero;
  Messenger_SendWord(*accel, data_descr);
  MPU6050_Data.renew_state |= coord_renewed;
  accel_check_renew_state();
}

void accel_calibrate(int16_t value, coordinate_t coord)
{
  int16_t *accel_zero;
  int32_t *accum;
  uint16_t *counter;
  uint8_t data_descr;
  uint8_t coord_is_calibrated;
  
  switch (coord) {
  case COORDINATE_X:
    accel_zero = &MPU6050_Data.accelx_zero;
    accum = &MPU6050_Data.accelx_accum;
    counter = &MPU6050_Data.accelx_counter;
    data_descr = MSNR_DD_ACCELXOFF;
    coord_is_calibrated = MPU6050_ST_ACCXCBTD;
    break;
  case COORDINATE_Y:
    accel_zero = &MPU6050_Data.accely_zero;
    accum = &MPU6050_Data.accely_accum;
    counter = &MPU6050_Data.accely_counter;
    data_descr = MSNR_DD_ACCELYOFF;
    coord_is_calibrated = MPU6050_ST_ACCYCBTD;
    break;
  case COORDINATE_Z:
    accel_zero = &MPU6050_Data.accelz_zero;
    accum = &MPU6050_Data.accelz_accum;
    counter = &MPU6050_Data.accelz_counter;
    data_descr = MSNR_DD_ACCELZOFF;
    coord_is_calibrated = MPU6050_ST_ACCZCBTD;
    break;
  }
  
  *accum += value;
  if (++(*counter) >= MPU6050_CALIBRATION_CYCLES) {
    MPU6050_Data.state |= coord_is_calibrated;
    *accel_zero = *accum / *counter;
    Messenger_SendWord(*accel_zero, data_descr);
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

//normalization and prefiltration can be performed in sensor
//data fusion algorithms must be abstracted
//dependency tree, each separate unit must be initialized from main in sequence
//units don't have access to objects within other units but have access to interfaces
// Data fusion. 

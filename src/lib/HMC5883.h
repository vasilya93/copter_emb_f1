#ifndef HMC5883_H
#define HMC5883_H

#include "Wire.h"
#include "stdint.h"

#define HMC5883_ADDRESS 0x1E

#define HMC5883_RA_IDA 0x0A
#define HMC5883_RA_IDB 0x0B
#define HMC5883_RA_IDC 0x0B

#define HMC5883_DD_IDA_REQ 1
#define HMC5883_DD_IDA_DATA 2
#define HMC5883_DD_IDB_REQ 3
#define HMC5883_DD_IDB_DATA 4
#define HMC5883_DD_IDC_REQ 5
#define HMC5883_DD_IDC_DATA 6

#define HMC5883_MSG_IDA 61
#define HMC5883_MSG_IDB 62
#define HMC5883_MSG_IDC 63
#define HMC5883_MSG_INIT_COMP 64

void HMC5883_initialize();

uint8_t* HMC5883_get_next_init_op(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript);
uint8_t* HMC5883_get_next_reg_op(uint16_t currentDataDescript, I2C_OpDescript_Type* opDescript);
void HMC5883_process_op_res(I2C_Operation_Type* operation);

#endif
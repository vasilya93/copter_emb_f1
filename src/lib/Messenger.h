#ifndef MESSENGER_H
#define MESSENGER_H

#include "Serial.h"

#define MSNR_MT_BYTE 0x01
#define MSNR_MT_WORD 0x02
#define MSNR_MT_DWRD 0x03

#define MSNR_DD_MASK 0xFC
#define MSNR_DD_NONE 0x04
#define MSNR_DD_ACCELX 0x08
#define MSNR_DD_ACCELY 0x0C
#define MSNR_DD_ACCELZ 0x10
#define MSNR_DD_WHOAMI 0x14
#define MSNR_DD_PWRMGMT1 0x18
#define MSNR_DD_ANGSPEEDX 0x1C
#define MSNR_DD_ANGSPEEDY 0x20
#define MSNR_DD_ANGSPEEDZ 0x24
#define MSNR_DD_GYROXOFF 0x28
#define MSNR_DD_GYROYOFF 0x2C
#define MSNR_DD_GYROZOFF 0x30
#define MSNR_DD_ACCELXOFF 0x34
#define MSNR_DD_ACCELYOFF 0x38
#define MSNR_DD_ACCELZOFF 0x3C
#define MSNR_DD_DCM11 0x40
#define MSNR_DD_DCM12 0x44
#define MSNR_DD_DCM13 0x48
#define MSNR_DD_DCM21 0x4C
#define MSNR_DD_DCM22 0x50
#define MSNR_DD_DCM23 0x54
#define MSNR_DD_DCM31 0x58
#define MSNR_DD_DCM32 0x5C
#define MSNR_DD_DCM33 0x60
#define MSNR_DD_HMC5883_IDA 0x64
#define MSNR_DD_HMC5883_IDB 0x68
#define MSNR_DD_HMC5883_IDC 0x6C
#define MSNR_DD_SENSFUS_ACCXNORM 0x70
#define MSNR_DD_SENSFUS_ACCYNORM 0x74
#define MSNR_DD_SENSFUS_ACCZNORM 0x78
#define MSNR_DD_SENSFUS_ROLL 0x7C
#define MSNR_DD_SENSFUS_PITCH 0x80
#define MSNR_DD_SENSFUS_YAW 0x84

#define MSNR_STARTOP_PATTERN "start"

typedef enum msnr_mode{
	MSNR_MODE_5BYTE = 1,
	MSNR_MODE_1BYTE
}msnr_mode;

void Messenger_Initialize(void (*start_operation)(void), msnr_mode mode);
void messenger_attach_pwm(void (*callback)(uint8_t, uint16_t));
void Messenger_SendByte(uint8_t);
void Messenger_SendWord(uint16_t word, uint8_t data_descr);
void Messenger_SendDWord(uint32_t dword, uint8_t data_descr);
void Messenger_SendFloat(float value, uint8_t data_descr);

static void messenger_parse_packet(uint8_t *packet);
static void byte_received_handler(uint8_t rec_byte);
#endif

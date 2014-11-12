#include "Messenger.h"
#include "Serial.h"
#include "stdint.h"
#include "string.h"

char last_bytes[MSNR_PATTERNSIZE] = {
	0, 0, 0, 0, 0
};

static msnr_mode current_mode = MSNR_MODE_5BYTE;

void (*start_op_callback)(void);

void byte_received_handler(uint8_t rec_byte);

void Messenger_Initialize(void (*start_operation)(void), msnr_mode mode)
{
	current_mode = mode;

	if (!Serial_ByteReceived_Attach(&byte_received_handler))
		while(1);
	
	start_op_callback = start_operation;
}

void Messenger_SendByte(uint8_t message)
{
	switch (current_mode) {
		case MSNR_MODE_5BYTE: {
			Serial_WriteByte(MSNR_MT_BYTE);
			Serial_WriteByte(0);
			Serial_WriteByte(0);
			Serial_WriteByte(0);
			Serial_WriteByte(message);
		}
		break;
		case MSNR_MODE_1BYTE: {
			Serial_WriteByte(message);
		}
		break;
	}
}

void Messenger_SendWord(uint16_t word, uint8_t data_descr)
{
	if (current_mode == MSNR_MODE_1BYTE)
		return;

	if (!(data_descr & MSNR_DD_MASK))
		return;
	if (data_descr & ~MSNR_DD_MASK)
		return;

  Serial_WriteByte(MSNR_MT_WORD | data_descr);
  Serial_WriteByte(0);
  Serial_WriteByte(0);
  Serial_WriteInt16(word);
}

void Messenger_SendDWord(uint32_t dword, uint8_t data_descr)
{
	if (current_mode == MSNR_MODE_1BYTE)
		return;

	if (!(data_descr & MSNR_DD_MASK))
		return;
	if (data_descr & ~MSNR_DD_MASK)
		return;

  Serial_WriteByte(MSNR_MT_DWRD | data_descr);
  Serial_WriteInt32(dword);
}

void byte_received_handler(uint8_t rec_byte)
{
	unsigned int i;
	for (i = 1; i < MSNR_PATTERNSIZE; i++)
		last_bytes[i-1] = last_bytes[i];
	
	last_bytes[MSNR_PATTERNSIZE - 1] = (char) rec_byte;
	
	if (!strncmp(last_bytes, MSNR_STARTOP_PATTERN, MSNR_PATTERNSIZE))
		start_op_callback();
}

#include <stdint.h>
#include <string.h>
#include "fifo.h"
#include "Messenger.h"
#include "Serial.h"
#include "PWM.h"

#define MSNR_SIZE_MESSAGE 5
#define MSNR_STAT_STARTED 0x01

static msnr_mode current_mode = MSNR_MODE_5BYTE;

void (*start_op_callback)(void);
void (*callback_pwm)(uint8_t, uint16_t);
uint8_t msgr_state = 0;
fifo_t fifo_received;

void Messenger_Initialize(void (*start_operation)(void), msnr_mode mode)
{
  if (fifo_initialize(&fifo_received, MSNR_SIZE_MESSAGE))
    while(1);
  
  current_mode = mode;
  
  if (!Serial_ByteReceived_Attach(&byte_received_handler))
    while(1);
  
  start_op_callback = start_operation;
}

void messenger_attach_pwm(void (*new_callback)(uint8_t, uint16_t))
{
  callback_pwm = new_callback;
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

void Messenger_SendFloat(float value, uint8_t data_descr)
{
  if (current_mode == MSNR_MODE_1BYTE)
    return;
  
  if (!(data_descr & MSNR_DD_MASK))
    return;
  if (data_descr & ~MSNR_DD_MASK)
    return;
  
  Serial_WriteByte(MSNR_MT_DWRD | data_descr);
  Serial_WriteFloat(value);
}

void byte_received_handler(uint8_t rec_byte)
{
  uint8_t *packet;
  fifo_push(&fifo_received, rec_byte);
  
  if (!fifo_is_full(&fifo_received))
    return;
  
  packet = fifo_get_array(&fifo_received, NULL);
  if (msgr_state & MSNR_STAT_STARTED) {
    fifo_reset(&fifo_received);
    messenger_parse_packet(packet);
  } else if (!strncmp((char *)packet, MSNR_STARTOP_PATTERN, MSNR_SIZE_MESSAGE)){
    fifo_reset(&fifo_received);
    msgr_state |= MSNR_STAT_STARTED;
    start_op_callback();
  }
  free(packet);
}

static void messenger_parse_packet(uint8_t *packet)
{
  uint16_t *p_value = (uint16_t *)(packet + 3);
  callback_pwm(PWM_CHANNEL1, *p_value);
  callback_pwm(PWM_CHANNEL2, *p_value);
  callback_pwm(PWM_CHANNEL3, *p_value);
  callback_pwm(PWM_CHANNEL4, *p_value); //350 - 1140
}
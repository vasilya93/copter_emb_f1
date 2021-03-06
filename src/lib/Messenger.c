#include <stdint.h>
#include <string.h>
#include "fifo.h"
#include "Messenger.h"
#include "Serial.h"
#include "PWM.h"
#include "settings.h"
#include "controller.h"

#define MSNR_SIZE_MESSAGE 5
#define MSNR_STAT_STARTED 0x01

void (*start_op_callback)(void);
void (*callback_pwm)(uint8_t, uint16_t);
uint8_t msgr_state = 0;
fifo_t fifo_received;

void msnr_init(void (*start_operation)(void))
{
  if (fifo_initialize(&fifo_received, MSNR_SIZE_MESSAGE))
    while(1);
  
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
#ifndef SETTINGS_MSNR_1BYTE
  Serial_WriteByte(MSNR_MT_BYTE);
  Serial_WriteByte(0);
#ifdef SETTINGS_MSNR_5BYTE
  Serial_WriteByte(0);
  Serial_WriteByte(0);
#endif
#endif
  Serial_WriteByte(message);
}

#ifndef SETTINGS_MSNR_1BYTE
void Messenger_SendWord(uint16_t word, uint8_t data_descr)
{  
  if (!(data_descr & MSNR_DD_MASK))
    return;
  if (data_descr & ~MSNR_DD_MASK)
    return;
  
  Serial_WriteByte(MSNR_MT_WORD | data_descr);
#ifdef SETTINGS_MSNR_5BYTE
  Serial_WriteByte(0);
  Serial_WriteByte(0);
#endif
  Serial_WriteInt16(word);
}
#endif

#ifndef SETTINGS_MSNR_1BYTE
#ifndef SETTINGS_MSNR_3BYTE
void Messenger_SendDWord(uint32_t dword, uint8_t data_descr)
{
  if (!(data_descr & MSNR_DD_MASK))
    return;
  if (data_descr & ~MSNR_DD_MASK)
    return;
  
  Serial_WriteByte(MSNR_MT_DWRD | data_descr);
  Serial_WriteInt32(dword);

}
#endif
#endif

#ifndef SETTINGS_MSNR_1BYTE
void Messenger_SendFloat(float value, uint8_t data_descr)
{
  if (!(data_descr & MSNR_DD_MASK))
    return;
  if (data_descr & ~MSNR_DD_MASK)
    return;

#ifdef SETTINGS_MSNR_5BYTE
  Serial_WriteByte(MSNR_MT_DWRD | data_descr);
  Serial_WriteFloat(value);
#endif

#ifdef SETTINGS_MSNR_3BYTE
  Serial_WriteByte(MSNR_MT_WORD | data_descr);
  int16_t value_conv = (int16_t) value;
  Serial_WriteInt16(value_conv);
#endif
}
#endif

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
  }
  free(packet);
}

static void messenger_parse_packet(uint8_t *packet)
{
  switch (packet[0]) {
  case MSNR_DDIN_PWM: {
    uint16_t *p_value = (uint16_t *)(packet + 3);
    on_pwm_value_received(*p_value);
    break;
  }
  case MSNR_DDIN_STARTOP: {
    start_op_callback();
    break;
  }
#ifdef SETTINGS_CONTROLLER_USE_INTEGRAL
  case MSNR_DDIN_INTCOEF: {
    float *p_value = (float *)(packet + 1);
    controller_set_int_coef(*p_value);
    break;
  }
#endif
  default:
    break;
  }
}

static void on_pwm_value_received(uint16_t pwm_value)
{
  (void) pwm_value;
  callback_pwm(PWM_CHANNEL1, pwm_value);
  callback_pwm(PWM_CHANNEL2, pwm_value);
  callback_pwm(PWM_CHANNEL3, pwm_value);
  callback_pwm(PWM_CHANNEL4, pwm_value); //350 - 1140
}
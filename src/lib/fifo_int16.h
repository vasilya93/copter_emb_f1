#ifndef FIFO_INT16_H
#define FIFO_INT16_H

#include "stdint.h"

#define FIFO_INT16_STAT_FULL 0x01
#define FIFO_INT16_STAT_EMPTY 0x02

typedef struct fifo_int16_t {
  int16_t *array;
  uint16_t size;
  uint16_t index_first; //index of the first element added into buffer
  uint16_t index_last; //index of the last element added into buffer
  uint16_t free_elems;
  uint8_t stat;
}fifo_int16_t;

int fifo_int16_initialize(fifo_int16_t *fifo, uint16_t size);
int fifo_int16_push_array(fifo_int16_t *fifo, int16_t* array, uint16_t size);
int inline fifo_int16_push(fifo_int16_t *fifo, int16_t new_elem);
int16_t fifo_int16_pop(fifo_int16_t *fifo);
inline int fifo_int16_is_full(fifo_int16_t *fifo);
inline int fifo_int16_is_empty(fifo_int16_t *fifo);
int fifo_int16_reset(fifo_int16_t *fifo);
int16_t *fifo_int16_get_array(fifo_int16_t *fifo, uint16_t *size);

static inline void fifo_int16_increase_elems_count(fifo_int16_t *fifo,
                                                   uint16_t value_added);
static inline void fifo_int16_decrease_elems_count(fifo_int16_t *fifo,
                                                   uint16_t value_added);

#endif
#ifndef FIFO_FLOAT_H
#define FIFO_FLOAT_H

#include "stdint.h"

#define FIFO_FLOAT_STAT_FULL 0x01
#define FIFO_FLOAT_STAT_EMPTY 0x02

typedef struct fifo_float_t {
  float *array;
  uint16_t size;
  uint16_t index_first; //index of the first element added into buffer
  uint16_t index_last; //index of the last element added into buffer
  uint16_t free_elems;
  uint8_t stat;
}fifo_float_t;

int fifo_float_initialize(fifo_float_t *fifo, uint16_t size);
int fifo_float_push_array(fifo_float_t *fifo, float* array, uint16_t size);
int inline fifo_float_push(fifo_float_t *fifo, float new_elem);
float fifo_float_pop(fifo_float_t *fifo);
inline int fifo_float_is_full(fifo_float_t *fifo);
inline int fifo_float_is_empty(fifo_float_t *fifo);
int fifo_float_reset(fifo_float_t *fifo);
int fifo_float_get_array(const fifo_float_t *const fifo,
                         float *const array,
                         uint16_t *const size);
static inline void fifo_float_increase_elems_count(fifo_float_t *fifo,
                                                   uint16_t value_added);
static inline void fifo_float_decrease_elems_count(fifo_float_t *fifo,
                                                   uint16_t value_added);

#endif
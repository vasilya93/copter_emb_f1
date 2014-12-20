#ifndef STORAGE_H
#define STORAGE_H

#include "stdint.h"

#define FIFO_STAT_FULL 0x01
#define FIFO_STAT_EMPTY 0x02

typedef struct fifo_t {
  uint8_t *array;
  uint16_t size;
  uint16_t index_first; //index of the first element added into buffer
  uint16_t index_last; //index of the last element added into buffer
  uint16_t free_elems;
  uint8_t stat;
}fifo_t;

int fifo_initialize(fifo_t *fifo, uint16_t size);
int fifo_push_array(fifo_t *fifo, uint8_t* array, uint16_t size);
int inline fifo_push(fifo_t *fifo, uint8_t new_elem);
uint8_t fifo_pop(fifo_t *fifo);
inline int fifo_is_full(fifo_t *fifo);
inline int fifo_is_empty(fifo_t *fifo);
int fifo_reset(fifo_t *fifo);
uint8_t *fifo_get_array(fifo_t *fifo, uint16_t *size);

static inline void fifo_increase_elems_count(fifo_t *fifo, uint16_t value_added);
static inline void fifo_decrease_elems_count(fifo_t *fifo, uint16_t value_added);

#endif /*STORAGE_H*/
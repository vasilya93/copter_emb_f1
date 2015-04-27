#include <stdlib.h>
#include <string.h>
#include "fifo_float.h"

int fifo_float_initialize(fifo_float_t *fifo, uint16_t size)
{
  if (fifo == NULL)
    return 1;

  fifo->array = malloc(size * sizeof(float));
  if (fifo->array == NULL)
    return 1;

  memset(fifo->array, 0, size * sizeof(float));

  fifo->size = size;
  fifo->free_elems = size;
  fifo->index_first = 0;
  fifo->index_last = 0;
  fifo->stat = FIFO_FLOAT_STAT_EMPTY;

  return 0;
}

int fifo_float_push_array(fifo_float_t *fifo, float *array_pushed, uint16_t bytes_count)
{
  unsigned long left_to_end;
  
  if (fifo == NULL || array_pushed == NULL || fifo->array == NULL)
    return 1;
  
  if (fifo->size < bytes_count)
    return 1;
  
  if (bytes_count > fifo->free_elems)
    fifo_float_decrease_elems_count(fifo, bytes_count - fifo->free_elems);

  left_to_end = fifo->size - fifo->index_last;
  if (left_to_end >= bytes_count) {
    memcpy(fifo->array + fifo->index_last,
           array_pushed,
           bytes_count * sizeof(float));
  } else {
    memcpy(fifo->array + fifo->index_last,
           array_pushed,
           left_to_end * sizeof(float));
    memcpy(fifo->array,
           array_pushed + left_to_end,
           (bytes_count - left_to_end) * sizeof(float));
  }
  
  fifo_float_increase_elems_count(fifo, bytes_count);
  return 0;
}

int fifo_float_push(fifo_float_t *fifo, float new_elem)
{
  if (fifo == NULL)
    return 1;

  return fifo_float_push_array(fifo, &new_elem, 1);
}

float fifo_float_pop(fifo_float_t *fifo)
{
  float symbol = 0;
  
  if (fifo == NULL)
    return symbol;

  fifo->stat &= ~FIFO_FLOAT_STAT_FULL;
  if(fifo->stat & FIFO_FLOAT_STAT_EMPTY) {
    return symbol;
  } else {
    symbol = fifo->array[fifo->index_first];
    fifo_float_decrease_elems_count(fifo, 1);
    return symbol;
  }
}

int fifo_float_is_full(fifo_float_t *fifo)
{
  return fifo->stat & FIFO_FLOAT_STAT_FULL;
}

int fifo_float_is_empty(fifo_float_t *fifo)
{
  return fifo->stat & FIFO_FLOAT_STAT_EMPTY;
}

int fifo_float_reset(fifo_float_t *fifo)
{
  if (fifo == NULL)
    return 1;

  memset(fifo->array, 0, fifo->size * sizeof(float));
  fifo->free_elems = fifo->size;
  fifo->index_first = 0;
  fifo->index_last = 0;
  fifo->stat = FIFO_FLOAT_STAT_EMPTY;
  return 0;
}

int fifo_float_get_array(const fifo_float_t *const fifo,
                         float *const array,
                         uint16_t *const size)
{
  uint16_t left_to_end;
  uint16_t elems_count = fifo->size - fifo->free_elems;
  if (size != NULL) {
    if (elems_count > *size) {
      return 1;
    }
  }
  
  if (fifo == NULL) {
    if (size != NULL)
      *size = 0;
    return 1;
  }

  left_to_end = fifo->size - fifo->index_first;
  if (left_to_end >= elems_count) {
    memcpy(array,
           fifo->array + fifo->index_first,
           elems_count * sizeof(float));
  } else {
    memcpy(array, fifo->array + fifo->index_first, left_to_end * sizeof(float));
    memcpy(array + left_to_end,
           fifo->array,
           (elems_count - left_to_end) * sizeof(float));
  }

  if (size != NULL)
    *size = elems_count;

  return 0;
}

//it is assumed in this function that no overwrite happens,
//as number of free elements is just decreased and position of the first elemnt
//is not changed
void fifo_float_increase_elems_count(fifo_float_t *fifo, uint16_t value_added)
{
  fifo->index_last += value_added;
  if (fifo->index_last >= fifo->size)
    fifo->index_last = fifo->index_last - fifo->size;
  
  fifo->free_elems -= value_added;
  fifo->stat &= ~(FIFO_FLOAT_STAT_EMPTY);
  if (fifo->index_first == fifo->index_last)
    fifo->stat |= FIFO_FLOAT_STAT_FULL;
}

void fifo_float_decrease_elems_count(fifo_float_t *fifo, uint16_t value_added)
{
  fifo->index_first += value_added;
  if (fifo->index_first >= fifo->size)
    fifo->index_first = fifo->index_first - fifo->size;
  
  fifo->free_elems += value_added;
  fifo->stat &= ~(FIFO_FLOAT_STAT_FULL);
  if (fifo->index_first == fifo->index_last)
    fifo->stat |= FIFO_FLOAT_STAT_EMPTY;
}
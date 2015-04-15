#include <stdlib.h>
#include <string.h>
#include "fifo.h"

int fifo_initialize(fifo_t *fifo, uint16_t size)
{
  if (fifo == NULL)
    return 1;

  fifo->array = malloc(size);
  if (fifo->array == NULL)
    return 1;
  
  memset(fifo->array, 0, size);
  
  fifo->size = size;
  fifo->free_elems = size;
  fifo->index_first = 0;
  fifo->index_last = 0;
  fifo->stat = FIFO_STAT_EMPTY;

  return 0;
}

int fifo_push_array(fifo_t *fifo, uint8_t* array_pushed, uint16_t bytes_count)
{
  unsigned long left_to_end;
  
  if (fifo == NULL || array_pushed == NULL || fifo->array == NULL)
    return 1;
  
  if (fifo->size < bytes_count)
    return 1;
  
  if (bytes_count > fifo->free_elems)
    fifo_decrease_elems_count(fifo, bytes_count - fifo->free_elems);

  left_to_end = fifo->size - fifo->index_last;
  if (left_to_end >= bytes_count) {
    memcpy(fifo->array + fifo->index_last, array_pushed, bytes_count);
  } else {
    memcpy(fifo->array + fifo->index_last, array_pushed, left_to_end);
    memcpy(fifo->array, array_pushed + left_to_end, bytes_count - left_to_end);
  }
  
  fifo_increase_elems_count(fifo, bytes_count);
  return 0;
}

int fifo_push(fifo_t *fifo, uint8_t new_elem)
{
  if (fifo == NULL)
    return 1;

  return fifo_push_array(fifo, &new_elem, 1);
}

uint8_t fifo_pop(fifo_t *fifo)
{
  uint8_t symbol = '\0';
  
  if (fifo == NULL)
    return symbol;

  fifo->stat &= ~FIFO_STAT_FULL;
  if(fifo->stat & FIFO_STAT_EMPTY) {
    return symbol;
  } else {
    symbol = fifo->array[fifo->index_first];
    fifo_decrease_elems_count(fifo, 1);
    return symbol;
  }
}

int fifo_is_full(fifo_t *fifo)
{
  return fifo->stat & FIFO_STAT_FULL;
}

int fifo_is_empty(fifo_t *fifo)
{
  return fifo->stat & FIFO_STAT_EMPTY;
}

int fifo_reset(fifo_t *fifo)
{
  if (fifo == NULL)
    return 1;

  memset(fifo->array, 0, fifo->size);
  fifo->free_elems = fifo->size;
  fifo->index_first = 0;
  fifo->index_last = 0;
  fifo->stat = FIFO_STAT_EMPTY;
  return 0;
}

uint8_t *fifo_get_array(fifo_t *fifo, uint16_t *size)
{
  if (fifo == NULL) {
    if (size != NULL)
      *size = 0;
    return NULL;
  }

  uint16_t left_to_end;
  uint16_t elems_count = fifo->size - fifo->free_elems;
  uint8_t *array = malloc(elems_count);  

  if (array == NULL) {
    if (size != NULL)
      *size = 0;
    return NULL;
  }

  left_to_end = fifo->size - fifo->index_first;
  if (left_to_end >= elems_count) {
    memcpy(array, fifo->array + fifo->index_first, elems_count);
  } else {
    memcpy(array, fifo->array + fifo->index_first, left_to_end);
    memcpy(array + left_to_end, fifo->array, elems_count - left_to_end);
  }

  if (size != NULL)
    *size = elems_count;

  return array;
}

//it is assumed in this function that no overwrite happens,
//as number of free elements is just decreased and position of the first elemnt
//is not changed
void fifo_increase_elems_count(fifo_t *fifo, uint16_t value_added)
{
  fifo->index_last += value_added;
  if (fifo->index_last >= fifo->size)
    fifo->index_last = fifo->index_last - fifo->size;
  
  fifo->free_elems -= value_added;
  fifo->stat &= ~(FIFO_STAT_EMPTY);
  if (fifo->index_first == fifo->index_last)
    fifo->stat |= FIFO_STAT_FULL;
}

void fifo_decrease_elems_count(fifo_t *fifo, uint16_t value_added)
{
  fifo->index_first += value_added;
  if (fifo->index_first >= fifo->size)
    fifo->index_first = fifo->index_first - fifo->size;
  
  fifo->free_elems += value_added;
  fifo->stat &= ~(FIFO_STAT_FULL);
  if (fifo->index_first == fifo->index_last)
    fifo->stat |= FIFO_STAT_EMPTY;
}
typedef struct TYPED_NAME(fifo_t) {
  TYPE *array;
  uint16_t size;
  uint16_t index_first; //index of the first element added into buffer
  uint16_t index_last; //index of the last element added into buffer
  uint16_t free_elems;
  uint8_t stat;
}TYPED_NAME(fifo_t);

int TYPED_NAME(fifo_initialize)(TYPED_NAME(fifo_t) *fifo, uint16_t size);
int TYPED_NAME(fifo_push_array)(TYPED_NAME(fifo_t) *fifo,
                                TYPE *array_pushed,
                                uint16_t size);
int inline TYPED_NAME(fifo_push)(TYPED_NAME(fifo_t) *fifo, TYPE new_elem);
TYPE TYPED_NAME(fifo_pop)(TYPED_NAME(fifo_t) *fifo);
inline int TYPED_NAME(fifo_is_full)(TYPED_NAME(fifo_t) *fifo);
inline int TYPED_NAME(fifo_is_empty)(TYPED_NAME(fifo_t) *fifo);
int TYPED_NAME(fifo_reset)(TYPED_NAME(fifo_t) *fifo);
TYPE *TYPED_NAME(fifo_get_array)(TYPED_NAME(fifo_t) *fifo, uint16_t *size);
static inline void TYPED_NAME(fifo_increase_elems_count)(TYPED_NAME(fifo_t) *fifo,
                                                         uint16_t value_added);
static inline void TYPED_NAME(fifo_decrease_elems_count)(TYPED_NAME(fifo_t) *fifo,
                                                         uint16_t value_added);

int TYPED_NAME(fifo_initialize)(TYPED_NAME(fifo_t) *fifo, uint16_t size)
{
  if (fifo == NULL)
    return 1;

  fifo->array = malloc(size * sizeof(TYPE));
  if (fifo->array == NULL)
    return 1;
  
  fifo->size = size;
  fifo->free_elems = size;
  fifo->index_first = 0;
  fifo->index_last = 0;
  fifo->stat = FIFO_STAT_EMPTY;

  return 0;
}

int TYPED_NAME(fifo_push_array)(TYPED_NAME(fifo_t) *fifo,
                                TYPE *array_pushed,
                                uint16_t size)
{
  unsigned long left_to_end;

  if (fifo == NULL || array_pushed == NULL || fifo->array == NULL)
    return 1;

  if (fifo->size < size)
    return 1;

  if (size > fifo->free_elems)
    TYPED_NAME(fifo_decrease_elems_count)(fifo, size - fifo->free_elems);

  left_to_end = fifo->size - fifo->index_last;
  if (left_to_end >= size) {
    memcpy(fifo->array + fifo->index_last, array_pushed, size * sizeof(TYPE));
  } else {
    memcpy(fifo->array + fifo->index_last,
           array_pushed,
           left_to_end * sizeof(TYPE));

    memcpy(fifo->array,
           array_pushed + left_to_end,
           (size - left_to_end) * sizeof(TYPE));
  }

  TYPED_NAME(fifo_increase_elems_count)(fifo, size);
  return 0;
}

int inline TYPED_NAME(fifo_push)(TYPED_NAME(fifo_t) *fifo, TYPE new_elem)
{
  if (fifo == NULL)
    return 1;

  return TYPED_NAME(fifo_push_array)(fifo, &new_elem, 1);
}

TYPE TYPED_NAME(fifo_pop)(TYPED_NAME(fifo_t) *fifo)
{
  TYPE value_to_return = 0;

  if (fifo == NULL)
    return value_to_return;

  fifo->stat &= ~FIFO_STAT_FULL;
  if(fifo->stat & FIFO_STAT_EMPTY) {
    return value_to_return;
  } else {
    value_to_return = fifo->array[fifo->index_first];
    TYPED_NAME(fifo_decrease_elems_count)(fifo, 1);
    return value_to_return;
  }
}

inline int TYPED_NAME(fifo_is_full)(TYPED_NAME(fifo_t) *fifo)
{
  return fifo->stat & FIFO_STAT_FULL;
}

inline int TYPED_NAME(fifo_is_empty)(TYPED_NAME(fifo_t) *fifo)
{
  return fifo->stat & FIFO_STAT_EMPTY;
}

int TYPED_NAME(fifo_reset)(TYPED_NAME(fifo_t) *fifo)
{
  if (fifo == NULL)
    return 1;

  fifo->free_elems = fifo->size;
  fifo->index_first = 0;
  fifo->index_last = 0;
  fifo->stat = FIFO_STAT_EMPTY;
  return 0;
}

TYPE *TYPED_NAME(fifo_get_array)(TYPED_NAME(fifo_t) *fifo, uint16_t *size)
{
  uint16_t left_to_end;
  uint16_t elems_count = fifo->size - fifo->free_elems;
  TYPE *array = malloc(elems_count * sizeof(TYPE));  
  if (fifo == NULL) {
    if (size != NULL)
      *size = 0;
    return NULL;
  }

  left_to_end = fifo->size - fifo->index_first;
  if (left_to_end >= elems_count) {
    memcpy(array, fifo->array + fifo->index_first, elems_count * sizeof(TYPE));
  } else {
    memcpy(array, fifo->array + fifo->index_first, left_to_end * sizeof(TYPE));
    memcpy(array + left_to_end,
           fifo->array,
           (elems_count - left_to_end) * sizeof(TYPE));
  }

  if (size != NULL)
    *size = elems_count;

  return array;
}

static inline void TYPED_NAME(fifo_increase_elems_count)(TYPED_NAME(fifo_t) *fifo,
                                                         uint16_t value_added)
{
  fifo->index_last += value_added;
  if (fifo->index_last >= fifo->size)
    fifo->index_last = fifo->index_last - fifo->size;
  
  fifo->free_elems -= value_added;
  fifo->stat &= ~(FIFO_STAT_EMPTY);
  if (fifo->index_first == fifo->index_last)
    fifo->stat |= FIFO_STAT_FULL;
}

static inline void TYPED_NAME(fifo_decrease_elems_count)(TYPED_NAME(fifo_t) *fifo,
                                                         uint16_t value_added)
{
  fifo->index_first += value_added;
  if (fifo->index_first >= fifo->size)
    fifo->index_first = fifo->index_first - fifo->size;
  
  fifo->free_elems += value_added;
  fifo->stat &= ~(FIFO_STAT_FULL);
  if (fifo->index_first == fifo->index_last)
    fifo->stat |= FIFO_STAT_EMPTY;
}
#include "stdint.h"
#include "stdbool.h"

#ifndef TIMER_H
#define TIMER_H

#define TIMER6 6
#define TIMER7 7
#define TIMER_USECONDS_MAX 60000000

//timer6 and timer7 are on apb1 bus

typedef struct timer_t
{
  void(*handler)();
	bool is_initialized;
	bool is_busy;
  bool do_repeat;
}timer_t;

void Timer_init(uint8_t timer);

//returns true if frequency is available, false - if unavailable
bool Timer_start(uint8_t timer, void (*handler)(), uint32_t useconds, bool do_repeat);

void Timer_Stop(const char* timName);

bool Timer_Attach(const char* timName, void(*)());

#endif

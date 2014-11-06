#ifndef CLOCK_CONTROL_H
#define CLOCK_CONTROL_H

typedef struct ClockControlType
{
    unsigned int APB1_Frequency;
    unsigned int APB2_Frequency;
    unsigned int AHB_Frequency;
    unsigned int SYSCLK_Frequency;
}ClockControlType;

extern ClockControlType ClockControl;

void ClockControl_Initialize(void);

#endif

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t sysTime_t;
typedef uint32_t clockTicks_t;



//----------- CLOCK TICK BASED STOPWATCH. HAS NANO-SECOND PRECISION ------------//

void STOPWATCH_InitClock();

void STOPWATCH_ResetClock();

clockTicks_t STOPWATCH_GetClockTicks();

float STOPWATCH_NanosecondsSince(clockTicks_t clock);



//----------- BASIC SYSTICK BASED STOPWATCH. ONLY HAS MILLISECOND PRECISION ------------//

sysTime_t STOPWATCH_GetTicks();

sysTime_t STOPWATCH_MillisecondsSince(sysTime_t start);

float STOPWATCH_SecondsSince(sysTime_t start);



//----------- TIMER BASED STOPWATCH FOR EVENTS -----------//

void STOPWATCH_ScheduleEventInNanoseconds(float eventTime);

void STOPWATCH_ScheduleEventInMicroseconds(float eventTime);

void STOPWATCH_ScheduleEventInMilliseconds(float eventTime);

void STOPWATCH_ScheduleEventInSeconds(float eventTime);

bool STOPWATCH_EventHasOccurred();

bool STOPWATCH_EventIsPending();

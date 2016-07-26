#include <stm32f0xx_hal.h>

#include "stopwatch.h"
#include "tim.h"

//----------- CLOCK TICK BASED STOPWATCH. HAS NANOSECOND PRECISION ------------//

// #define DEMCR_ENABLE (0x01000000)
// #define DWTCTRL_ENABLE (0x00000001)
//
// void STOPWATCH_InitClock() {
// 	// Enable TRC
// 	CoreDebug->DEMCR &= ~DEMCR_ENABLE;
// 	CoreDebug->DEMCR |= DEMCR_ENABLE;
//
// 	// Enable Counter
// 	DWT->CTRL &= ~DWTCTRL_ENABLE;
// 	DWT->CTRL |= DWTCTRL_ENABLE;
//
// 	STOPWATCH_ResetClock();
// }
//
// void STOPWATCH_ResetClock() {
// 	DWT->CYCCNT = 0;
// }
//
// clockTicks_t STOPWATCH_GetClockTicks() {
// 	return DWT->CYCCNT;
// }
//
// float STOPWATCH_NanosecondsSince(clockTicks_t clock) {
// 	uint32_t tickDiff = (uint32_t) STOPWATCH_GetClockTicks() - (uint32_t) clock;
// 	float period_ns = 1e9f / SystemCoreClock;
// 	float since_ns = tickDiff * period_ns;
//
// 	return since_ns;
// }



//----------- BASIC SYSTICK BASED STOPWATCH. ONLY HAS MILLISECOND PRECISION ------------//
//
// sysTime_t STOPWATCH_GetTicks() {
// 	return HAL_GetTick();
// }
//
// sysTime_t STOPWATCH_MillisecondsSince(sysTime_t start) {
// 	sysTime_t diff = STOPWATCH_GetTicks() - start;
// 	return diff;
// }
//
// float STOPWATCH_SecondsSince(sysTime_t start) {
// 	return (float) (STOPWATCH_MillisecondsSince(start)) / 1000.0f;
// }



//----------- TIMER BASED STOPWATCH FOR EVENTS -----------//

#define TIMER_HANDLE (&htim2)
#define TIMER_IRQ    (TIM2_IRQn)
#define TIMER_FREQ   (48e6)

volatile bool eventOccurred = false;
volatile bool eventPending = false;

void STOPWATCH_ScheduleEventInNanoseconds(float eventTime) {
	STOPWATCH_ScheduleEventInSeconds(eventTime * 1e-9f);
}

void STOPWATCH_ScheduleEventInMicroseconds(float eventTime) {
	STOPWATCH_ScheduleEventInSeconds(eventTime * 1e-6f);
}

void STOPWATCH_ScheduleEventInMilliseconds(float eventTime) {
	STOPWATCH_ScheduleEventInSeconds(eventTime * 1e-3f);
}

void STOPWATCH_ScheduleEventInSeconds(float eventTime) {
	// Stop the timer and interrupts
	HAL_NVIC_DisableIRQ(TIMER_IRQ);
	HAL_TIM_Base_Stop_IT(TIMER_HANDLE);

	// Initialize the timer
	TIMER_HANDLE->Init.Period = (uint32_t) (eventTime * TIMER_FREQ) - 1; // -1 since it counts from 0 to this number
	eventOccurred = false;

	// Start the countdown
	HAL_TIM_OnePulse_Init(TIMER_HANDLE, TIM_OPMODE_SINGLE);
	__HAL_TIM_CLEAR_IT(TIMER_HANDLE, TIM_IT_UPDATE);

	// Start the timer and interrupts
	eventPending = true;
	HAL_TIM_Base_Start_IT(TIMER_HANDLE);
	HAL_NVIC_EnableIRQ(TIMER_IRQ);
}

bool STOPWATCH_EventHasOccurred() {
	if (eventOccurred) {
		eventOccurred = false;
		eventPending = false;
		return true;
	} else {
		return false;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == TIMER_HANDLE) {
		eventPending = false;
		eventOccurred = true;
	}
}

bool STOPWATCH_EventIsPending() {
	return eventPending;
}

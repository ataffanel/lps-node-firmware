/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the LPS Firmware.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stm32f0xx_hal.h>

#include "stopwatch.h"
#include "tim.h"

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

/*
 * RTC.h
 *
 *  Created on: 29 jun 2023
 *      Author: maxim
 */

#ifndef RTC_H_
#define RTC_H_

#include <msp430.h>

void RTC_disabling();
void RTC_setTime(int hour, int min);
void RTC_setDate(int day, int month, int year);
void RTC_setAlarm(int min_A);
void RTC_enable();

#endif /* RTC_H_ */

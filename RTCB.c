#include <RTC.h>

//Drivers para la el control en actualizacion del programa del UC maestro de forma periodica
//maunual del MSP430 slau367p cap27
void RTC_disabling(){
    RTCCTL01_H = RTCHOLD_H;  //(p.701)
}

void RTC_setTime(int hour, int min){
    RTCSEC =  0x00;         //(p.703)
    RTCHOUR = hour;         //(p.705)
    RTCMIN = min;           //(p.704)
}

void RTC_setDate(int day, int month, int year){
    RTCDAY = day;
    RTCMON = month;
    RTCYEAR = year;
}

void RTC_setAlarm(int min_A){
    RTCCTL01 |= RTCAIE;     //(p.701)
    __enable_interrupt();
    RTCAMIN = min_A;        //(p.709)
    RTCAMIN |= BIT7; //Activa la alarma de Min
    RTCAHOUR = 0x00;        //(p.710)
    RTCADOW = 0x00;         //(p.711)
    RTCADAY = 0x00;         //(p.712)
}

void RTC_enable(){
    RTCCTL01_H &= ~(RTCHOLD_H);//(p.701)
}

#pragma vector =  RTC_VECTOR
__interrupt void RTC_ISR(){
    RTCCTL01 &= ~RTCAIFG;   //(p.701)
    _low_power_mode_off_on_exit();
}

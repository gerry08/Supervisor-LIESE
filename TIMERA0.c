#include <TIMERA0.h>
//Driver para el control de timing en la secuencia de acceso a bootloader
//manual del MSP430 slau367p cap 25


//Inicializa los registros del Timer (p.645)
void timer_Init(void){
    //SMCLK = 1 Mhz
    TA0CTL = TACLR; //Limpua el timer
    TA0CTL |= TASSEL__SMCLK; //Configura SMCLK como fuente de reloj, para el timer. (p.658)
    TA0CTL |= ID__8; //Divide el reloj por 8
    TA0CTL |= MC_0; //Para el timer
    //TA0CTL |= TAIE;
    TA0EX0 |= TAIDEX_7;
    TA0CCTL0 = CCIE;
    _enable_interrupt();
}

void timer_setPeriod(uint16_t sec){
    //Valor maximo alcanzable con 16 bits = 65,535
    uint16_t count = 0;
    count = 15625 * sec;
    TA0CCR0 = count;
}

void timer_Stop(void){
    TA0CTL &= ~MC_3;
    TA0R = 0;
}

void timer_Enable(void){
    TA0CTL |= MC__UP;
}

void timer_Wait_N_ms(int Num){
    int i;
    for (i=0;i<Num;i++){
        timer_Wait_1_ms();
    }
}

void timer_Wait_1_ms(){
    TA0R = 0;         //Pone la cuenta en 0
    while(TA0R <= 1000){} //Espera hasta el timer cuente hasta 1000 o 1 ms (p.659)
}


#include "peripheral_setup.h"

/**
 * main.c
 */


uint32_t count=0, index=0, index400=0;
uint16_t sinetable[125];
uint16_t sinetable400[400];

uint16_t adc1 = 0;
uint16_t adc2 = 0;

uint16_t plot[400];
uint16_t *adc = &adc1;

uint16_t deleta;


__interrupt void isr_cpu_timer0(void);
__interrupt void isr_adc(void);

int main(void)
{
    InitSysCtrl();                  //Initialize System Control
    DINT;                           //Disable CPU interrupts
    InitPieCtrl();                  //Initialize the PIE control register to their default state
    IER = 0x0000;                   //Disable CPU interrupts
    IFR = 0x0000;                   //Clear all CPU interrupt flags
    InitPieVectTable();             //Initialize the PIE vector table

    Setup_GPIO();
    Setup_ePWM();
    Setup_ADC();

    EALLOW;
    PieVectTable.TIMER0_INT = &isr_cpu_timer0;  //atribui endereco para funcao interrupt timer0
    PieVectTable.ADCA1_INT = &isr_adc;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;          //timer0 //coluna 7 da tabela 'PIE Group 1 Vectors - Muxed into CPU INT1'
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;          //ADC //coluna 1 da mesma linha que o timer
    EDIS;
    IER |= M_INT1;  //como esta na mesma linha q o timer, n precisa habilitar outra linha

    InitCpuTimers(); //inicializa timer
    ConfigCpuTimer(&CpuTimer0, 200, 160); //configura o timer (enredeco, clock in Mega, periodo em Micro)
    CpuTimer0Regs.TCR.all = 0x4001; //importante setar o bit 14, ele habilita o timer

    for (index = 0; index < 125; index++){
        sinetable[index] = (uint16_t) (1000.0*(1.0 + sin(6.28318531/125.0*((float)index))));
    }

    for (index = 0; index < 400; index++){
        sinetable400[index] = 500 + (uint16_t) (2000.0*(1.0 + sin(6.28318531/400.0*((float)index))));
    }
    index = 0;

    EINT;                           //Enable Global interrupt INTM
    ERTM;                           //Enable Global realtime interrupt DBGM

    while(1){
        //
    }
}

__interrupt void isr_cpu_timer0(void){
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    /*
    index = (index == 124) ? 0 : (index+1);

    EPwm7Regs.CMPA.bit.CMPA = sinetable[index];
    EPwm8Regs.CMPA.bit.CMPA = sinetable[index];
    */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //flag q sinaliza fim da interrupt, n apagar
}


__interrupt void isr_adc(void){
    GpioDataRegs.GPADAT.bit.GPIO14 = 1;

    adc1 = AdcaResultRegs.ADCRESULT0;
    adc2 = AdcaResultRegs.ADCRESULT1;

    index400 = (index400 == 400) ? 0 : (index400+1);

    EPwm7Regs.CMPA.bit.CMPA = sinetable[index400];
    EPwm8Regs.CMPA.bit.CMPA = sinetable[index400];

    plot[index400] = *adc;

    deleta++;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPADAT.bit.GPIO14 = 0;
}

#include "peripheral_setup.h"
#include "sogi.h"
#include "CLA1_Config.h"

/**
 * main.c
 */


uint32_t count=0, index=0, index400=0, count_task;
// uint16_t sinetable[125];
// uint16_t sinetable400[400];

/*
float min = 0;
float max = 0;
*/


volatile SPLL_SOGI cla1_pll; //cria struct pro PLL usado no CLA
volatile float vrede_CLA = 0;
volatile float phase_CLA = 0;
volatile float ampl_CLA = 0.5;
//passar variaveis para o segmento de memoria que o CLA tem acesso
#pragma DATA_SECTION(vrede_CLA, "Cla1ToCpuMsgRAM"); //esse segmento a CPU consegue observar
#pragma DATA_SECTION(cla1_pll, "CLADataLS0");
#pragma DATA_SECTION(phase_CLA, "CLADataLS0"); //LS0 foi alocado para dados do CLA
#pragma DATA_SECTION(ampl_CLA, "CLADataLS0");
// #pragma CODE_SECTION(adc_isr,"CLADataLS5"); //codigo nao precisa pq ja vai automatico pra memoria certa


SPLL_SOGI v_pll; //cria struct pro PLL
float vrede = 0;
float vsync = 0;
float phase = 0;
float ampl = 0.5;

float plot1[512], plot2[512];
float *padc1 = &vrede;
float *padc2 = &vsync;

uint16_t valor;

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
    Setup_DAC();
    CLA1_ConfigCLAMemory();
    CLA1_InitCpu1Cla1();

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

    /*
    for (index = 0; index < 125; index++){
        sinetable[index] = (uint16_t) (1000.0*(1.0 + sin(6.28318531/125.0*((float)index))));
    }

    for (index = 0; index < 400; index++){
        sinetable400[index] = 500 + (uint16_t) (2000.0*(1.0 + sin(6.28318531/400.0*((float)index))));
    }
    index = 0;
    */
    //inicializa PLL da interrupcao do ADC A3
    SOGI_init(60, 32.5520833E-06, &v_pll); //inicializa PLL //(float Grid_freq, float DELTA_T, SPLL_SOGI *spll_obj)
    SOGI_coeff_update(32.5520833E-06, 376.99112, 0.7, &v_pll); //(float delta_T, float wn, float k, SPLL_SOGI *spll)

    //inicializa PLL do CLA 1
    SOGI_init(60, 32.5520833E-06, &cla1_pll); //inicializa PLL //(float Grid_freq, float DELTA_T, SPLL_SOGI *spll_obj)
    SOGI_coeff_update(32.5520833E-06, 376.99112, 0.7, &cla1_pll); //(float delta_T, float wn, float k, SPLL_SOGI *spll)


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

    //vrede = 0.0005*((int)AdcaResultRegs.ADCRESULT0 - 0x7FF); //-2047
    vrede = 0.000732869*((int)AdcaResultRegs.ADCRESULT0 - 1364.0);

    /*
    if (min > vrede)
        min = vrede;

    if (max < vrede)
        max = vrede;
    */

    //adc1 = AdcaResultRegs.ADCRESULT0;
    //adc2 = AdcaResultRegs.ADCRESULT1;

    //index400 = (index400 == 400) ? 0 : (index400+1);

    v_pll.u[0] = vrede;
    SPLL_SOGI_CALC(&v_pll);

    vsync = v_pll.sin_;
    EPwm7Regs.CMPA.bit.CMPA = (uint16_t) (1627.0 * (1.0 + ampl * __sin(v_pll.theta[1]+phase)));
    //EPwm7Regs.CMPA.bit.CMPA = (uint16_t) (1627.0 * (1.0 + ampl * sin(v_pll.theta[1]+phase)));
    GpioDataRegs.GPADAT.bit.GPIO14 = 0;

    //EPwm7Regs.CMPA.bit.CMPA = sinetable[index400];
    //EPwm8Regs.CMPA.bit.CMPA = sinetable[index400];

    plot1[index] = *padc1; //inicialmente float *padc1 = &vrede;
    plot2[index] = *padc2; //inicialmente float *padc2 = &vsync;

    index = (index == 511) ? 0 : (index+1);
    valor = (uint16_t) (2047.0 * (1.0 + ampl * __sin( v_pll.theta[1] + phase)));
    //valor = (uint16_t) (2047.0 * (1.0 + ampl * sin( v_pll.theta[1] + phase)));
    if (valor < 4095){
        EALLOW;
        DacaRegs.DACVALS.bit.DACVALS = (uint16_t) valor;
        EDIS;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// Definicoes em CLA_Config.h
__interrupt void CLA1_isr1(void){
    count_task++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
}

__interrupt void CLA1_isr2(void){
    asm(" ESTOP0");
}

__interrupt void CLA1_isr3(void){
    asm(" ESTOP0");
}

__interrupt void CLA1_isr4(void){
    asm(" ESTOP0");
}

__interrupt void CLA1_isr5(void){
    asm(" ESTOP0");
}

__interrupt void CLA1_isr6(void){
    asm(" ESTOP0");
}

__interrupt void CLA1_isr7(void){
    //asm(" ESTOP0");
    asm(" ESTOP0");
}

__interrupt void CLA1_isr8(void){
    // Acknowledge the end-of-task interrupt for task 8
    PieCtrlRegs.PIEACK.all = M_INT11;
    // Uncomment to halt debugger and stop here
    //    asm(" ESTOP0");
}

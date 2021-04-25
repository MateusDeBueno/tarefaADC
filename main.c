#include "peripheral_setup.h"
#include "sogi.h"
#include "CLA1_Config.h"
#include "PLL_tri.h"

/**
 * main.c
 */



// uint16_t sinetable400[400];

/*
float min = 0;
float max = 0;
*/

/*
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
*/

SPLL_SOGI v_pll; //cria struct pro PLL
float vrede = 0;
float vsync = 0;
float phase = 0;
float ampl = 0.5;

uint16_t valor;
uint32_t count=0, index=0, count_task;
uint16_t sinetable[125], sinetable2[106];
uint32_t index_Va, index_Vb = 42, index_Vc = 83;

//variaveis lidas pelos ADC, esse sinais saem dos 3 DAC que simulam as tensoes de fase
float Va, Vb, Vc;
volatile float Va_unit, Vb_unit, Vc_unit;
float Vab_unit, Vbc_unit, Vca_unit;
int ADC_Va, ADC_Vb, ADC_Vc;
int index_plot;
volatile float plot1[125], plot2[125];
volatile float *padc1 = &Va_unit;
volatile float *padc2 = &Vb_unit;
bool plot_ADC = false;

volatile PLL_tri PLL;

uint16_t ADC_Vab_PLL, ADC_Vbc_PLL, ADC_Vca_PLL;
uint16_t ADC_Vab, ADC_Vbc, ADC_Vca;
#define CHOOSE_VAB 0
#define CHOOSE_VBC 1
#define CHOOSE_VCA 2
#define CHOOSE_VAB_VBC 3
#define CHOOSE_VAB_VCA 4
int choose_dac = 0;

#define CHOOSE_50 0
#define CHOOSE_52 1
int choose_freq = 0;

__interrupt void isr_cpu_timer0(void);
__interrupt void isr_adc(void);

int main(void)
{
    InitSysCtrl();                  //Initialize System Control
    DINT;                           //Disable CPU interrupts
    InitPieCtrl();                  //Initialize the PIE control register to their default state
    IER = 0x0000;                   //Disable CPU interrupts
    IFR = 0x0000;                   //Cl ear all CPU interrupt flags
    InitPieVectTable();             //Initialize the PIE vector table

    Setup_GPIO();
    Setup_ePWM();
    Setup_ADC_A();
    Setup_ADC_B();
    Setup_DAC_A();
    Setup_DAC_B();
    //CLA1_ConfigCLAMemory();
    //CLA1_InitCpu1Cla1();

    PLL.wt[0] = -314.0;

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
        sinetable[index] = (uint16_t) (950.0*(1.0 + __sin(6.28318531/125.0*((float)index))));
    }

    for (index = 0; index < 120; index++){
        sinetable2[index] = (uint16_t) (950.0*(1.0 + __sin(6.28318531/120.0*((float)index))));
    }

    //iniciar PLL monofasico - funcao pronta
    //SOGI_init(60, 32.5520833E-06, &v_pll); //inicializa PLL //(float Grid_freq, float DELTA_T, SPLL_SOGI *spll_obj)
    //SOGI_coeff_update(32.5520833E-06, 376.99112, 0.7, &v_pll); //(float delta_T, float wn, float k, SPLL_SOGI *spll)
    /*
    //inicializa PLL do CLA 1
    SOGI_init(60, 32.5520833E-06, &cla1_pll); //inicializa PLL //(float Grid_freq, float DELTA_T, SPLL_SOGI *spll_obj)
    SOGI_coeff_update(32.5520833E-06, 376.99112, 0.7, &cla1_pll); //(float delta_T, float wn, float k, SPLL_SOGI *spll)
    */

    EINT;                           //Enable Global interrupt INTM
    ERTM;                           //Enable Global realtime interrupt DBGM

    while(1){
        //
    }
}

__interrupt void isr_cpu_timer0(void){
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    switch(choose_freq)
    {
    case CHOOSE_50:
        index_Va = (index_Va == 124) ? 0 : (index_Va+1);
        index_Vb = (index_Vb == 124) ? 0 : (index_Vb+1);
        index_Vc = (index_Vc == 124) ? 0 : (index_Vc+1);

        EPwm8Regs.CMPA.bit.CMPA = sinetable[index_Va]; //DAC1
        EPwm8Regs.CMPB.bit.CMPB = sinetable[index_Vb]; //DAC2
        EPwm7Regs.CMPA.bit.CMPA = sinetable[index_Vc]; //DAC3
    break;
    case CHOOSE_52:
        index_Va = (index_Va == 119) ? 0 : (index_Va+1);
        index_Vb = (index_Vb == 119) ? 0 : (index_Vb+1);
        index_Vc = (index_Vc == 119) ? 0 : (index_Vc+1);
        EPwm8Regs.CMPA.bit.CMPA = sinetable2[index_Va]; //DAC1
        EPwm8Regs.CMPB.bit.CMPB = sinetable2[index_Vb]; //DAC2
        EPwm7Regs.CMPA.bit.CMPA = sinetable2[index_Vc]; //DAC3
    break;
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //flag q sinaliza fim da interrupt, n apagar
}


__interrupt void isr_adc(void){
    GpioDataRegs.GPADAT.bit.GPIO14 = 1; //J8 GPIO14 pin74

    while(!AdcbRegs.ADCINTFLG.bit.ADCINT1);       // espera o fim da conversao do B

    //modulo ADC A
    ADC_Va =  ((int)AdcaResultRegs.ADCRESULT0); //DAC1 - ADCIN15 - J7 pin63
    ADC_Vb =  ((int)AdcaResultRegs.ADCRESULT1); //DAC2 - ADCINA5 - J7 pin66

    //modulo ADC B
    ADC_Vc = ((int)AdcbResultRegs.ADCRESULT0);  //DAC3 - ADCINB3 - J3 pin25

    //normaliza os valores de fase
    Va_unit = (float)ADC_Va*0.000487329 -1.0;
    Vb_unit = (float)ADC_Vb*0.000487329 -1.0;
    Vc_unit = (float)ADC_Vc*0.000487329 -1.0;

    //plot para visualizar
    if (plot_ADC == true){
        plot1[index_plot] = *padc1;
        plot2[index_plot] = *padc2;
        index_plot = (index_plot == 124) ? 0 : (index_plot+1);
    }

    find_wt(Va_unit, Vb_unit, Vc_unit);

    Vab_unit = Va_unit - Vb_unit; //tensao da fase Vab
    Vbc_unit = Vb_unit - Vc_unit; //tensao da fase Vbc
    Vca_unit = Vc_unit - Va_unit; //tensao da fase Vca

    //Converte a senoide proveniente do PWM para ser convertida pelo DAC
    ADC_Vab = (uint16_t) ((__divf32(Vab_unit,1.732050808) + 1)*2000.0);
    ADC_Vbc = (uint16_t) ((__divf32(Vbc_unit,1.732050808) + 1)*2000.0);
    ADC_Vca = (uint16_t) ((__divf32(Vca_unit,1.732050808) + 1)*2000.0);

    //Converte a senoide sintetizada para ser convertida pelo DAC
    ADC_Vab_PLL = (uint16_t) ((__divf32(PLL.VabPLL,1.732050808) + 1)*2000.0);
    ADC_Vbc_PLL = (uint16_t) ((__divf32(PLL.VbcPLL,1.732050808) + 1)*2000.0);
    ADC_Vca_PLL = (uint16_t) ((__divf32(PLL.VcaPLL,1.732050808) + 1)*2000.0);


    //ESCOLHE O QUE VAI SAIR NOS DACS
    switch(choose_dac)
    {
        case CHOOSE_VAB:
            EALLOW;
            DacaRegs.DACVALS.bit.DACVALS = ADC_Vab;
            DacbRegs.DACVALS.bit.DACVALS = ADC_Vab_PLL;
            EDIS;
        break;
        case CHOOSE_VBC:
            EALLOW;
            DacaRegs.DACVALS.bit.DACVALS = ADC_Vbc;
            DacbRegs.DACVALS.bit.DACVALS = ADC_Vbc_PLL;
            EDIS;
        break;
        case CHOOSE_VCA:
            EALLOW;
            DacaRegs.DACVALS.bit.DACVALS = ADC_Vca;
            DacbRegs.DACVALS.bit.DACVALS = ADC_Vca_PLL;
            EDIS;
        break;
        case CHOOSE_VAB_VBC:
            EALLOW;
            DacaRegs.DACVALS.bit.DACVALS = ADC_Vab_PLL;
            DacbRegs.DACVALS.bit.DACVALS = ADC_Vbc_PLL;
            EDIS;
        break;
        case CHOOSE_VAB_VCA:
            EALLOW;
            DacaRegs.DACVALS.bit.DACVALS = ADC_Vab_PLL;
            DacbRegs.DACVALS.bit.DACVALS = ADC_Vca_PLL;
            EDIS;
        break;
    }


    GpioDataRegs.GPADAT.bit.GPIO14 = 0;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
/*
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
*/

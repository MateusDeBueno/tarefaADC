/*
 * Peripheral_Setup.c
 *
 *  Created on: 7 de abr de 2021
 *      Author: Mateus
 */


#include "peripheral_setup.h"


void Setup_GPIO(void){
    EALLOW;

    /*
    ao escolher um GPIO, deve-se olhar em qual MUX ele participa, varia de A ate F
    GpioCtrlRegs.GPxMUXy.bit.GPIOz = 0;
    x = A,B,C...
    y = 1,2
    z = 0...15, 16...31 (A)
    z = 32...47, 48...63 (B)
    z = 64...79, 80...95 (C)
    z = 96...111, 112...127(D)
    z = 128...143, 144...159(E)
    z = 160...168 (F)
    */

    //GPIO31 - LED
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     //configura como saida, so é necessario se for um uso generico, feito GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;     //adiciona pull-down

    //GPIO34 - LED
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

    GpioDataRegs.GPBSET.bit.GPIO34 = 1; //apaga leds chatos, eh com logica reversa
    GpioDataRegs.GPASET.bit.GPIO31 = 1;


    //PWM1A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;    //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;      //desabilita pull up

    //PWM1B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;    //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;      //desabilita pull up

    //PWM7A - DAC3
    GpioCtrlRegs.GPEGMUX2.bit.GPIO157 = 0;    //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPEMUX2.bit.GPIO157 = 1;     //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPEPUD.bit.GPIO157 = 1;      //desabilita pull up

    //PWM8A - DAC1
    GpioCtrlRegs.GPEGMUX2.bit.GPIO159 = 0;    //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPEMUX2.bit.GPIO159 = 1;     //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPEPUD.bit.GPIO159 = 1;      //desabilita pull up

    //PWM8B  - DAC2
    GpioCtrlRegs.GPFGMUX1.bit.GPIO160 = 0;    //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPFMUX1.bit.GPIO160 = 1;     //configura como PWM, escolhe a funcao
    GpioCtrlRegs.GPFPUD.bit.GPIO160 = 1;      //desabilita pull up


    //GPIO14
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;

    //GPIO15
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;
    GpioCtrlRegs.GPACSEL2.bit.GPIO15 = GPIO_MUX_CPU1CLA;

    EDIS;
}


void Setup_ePWM(void){
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       //desativo o pwm para poder configurar

    /////////////CONFIGURA PWM1/////////////

    //configura portadora
    EPwm1Regs.TBPRD = 2000;                             //25kz //PRD=MCclock/(2*fs), porem se updown PRD = MCclock/(4*fs)
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                      //defasagem 0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;         //quando igual a zero gera pulso de referencia pra outros PWM
    EPwm1Regs.TBCTR = 0x0000;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      //count up/down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             //desabilitar deslocamente de fase
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            //pre scale do clock do pwm, util para baixas frequencias, para o TBPRD n estourar o valor maximo
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;               //TBPRD eh unit16, entao TBPRD vale entre 0 ate 65535

    //configura shadow (comparadores so atualizam apos um evento)
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         //habilita shadow
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   //atualiza comparador A somente no ZERO e no PRD
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;         //habilita shadow
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;   //atualiza comparador B somente no ZERO e no PRD

    //configura as ações para PWM1A
    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //EPWM B é complementar do EPWM A
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //habilita dead band
    EPwm1Regs.DBFED.bit.DBFED = 100;                //rising edges
    EPwm1Regs.DBRED.bit.DBRED = 100;                //falling edges

    //Trigger ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                  //enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;   //dispara ADC no vale e no pico da triangular
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;            //trigger on every event


    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD/2;


    /////////////CONFIGURA PWM7/////////////
    EPwm7Regs.TBPRD = 2000;                             //PRD=MCclock/(2*fs), porem se updown PRD = MCclock/(4*fs)
    EPwm7Regs.CMPA.bit.CMPA = 0;
    EPwm7Regs.TBPHS.bit.TBPHS = 0;                      //defasagem 0
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;          //recebe pulso de sincronismo
    EPwm7Regs.TBCTR = 0x0000;
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      //count up/down
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;              //habilita deslocamente de fase
    EPwm7Regs.TBCTL.bit.PHSDIR = TB_DOWN;               //configura se a fase avanca ou atrasa
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            //pre scale do clock do pwm, util para baixas frequencias, para o TBPRD n estourar o valor maximo
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;               //TBPRD eh unit16, entao TBPRD vale entre 0 ate 65535

    //configura shadow (comparador so atualiza apos evento)
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         //habilita shadow
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   //atualiza comparador A somente no ZERO e no PRD

    //configura as ações para PWM7A
    EPwm7Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;


    /////////////CONFIGURA PWM8/////////////
    EPwm8Regs.TBPRD = 2000;                             //25kz //PRD=MCclock/(2*fs), porem se updown PRD = MCclock/(4*fs)
    EPwm8Regs.CMPA.bit.CMPA = 0;
    EPwm8Regs.TBPHS.bit.TBPHS = 0;                      //defasagem 0
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;          //recebe pulso de sincronismo
    EPwm8Regs.TBCTR = 0x0000;
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      //count up/down
    EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;              //habilita deslocamente de fase
    EPwm8Regs.TBCTL.bit.PHSDIR = TB_DOWN;               //configura se a fase avanca ou atrasa
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            //pre scale do clock do pwm, util para baixas frequencias, para o TBPRD n estourar o valor maximo
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;               //TBPRD eh unit16, entao TBPRD vale entre 0 ate 65535

    //configura shadow (comparadores so atualizam apos um evento)
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         //habilita shadow
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   //atualiza comparador A somente no ZERO e no PRD
    //configura shadow (comparadores so atualizam apos um evento)
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;         //habilita shadow
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;   //atualiza comparador A somente no ZERO e no PRD

    //configura as ações para PWM8A e PWM8B
    EPwm8Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm8Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;
    EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm8Regs.AQCTLB.bit.CBD = AQ_SET;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;       //ativa o pwm
    EDIS;
}


void Setup_ADC_A(void){
    Uint16 acqps;
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if (ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14;             //aproximadamente 75ns
    else                        //16 bits de resolucao
        acqps = 63;             //aproximadamente 320ns

    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                      //seta para ADCCLK divitor por 4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;                   //seta pulso um ciclo antes do resultado
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ =1;                       //liga o ADC
    DELAY_US(1000);                                         //delay de 1ms pra permitir q os ADC ligem

    //SOC 0 do modulo A -- Va
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 15;                     //SOC0 vai converter o pino ADCIN15 //J7 pin63
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;                  //sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;  //trigger on ePWM1 SOCA

    //SOC 1 do modulo A -- Vb
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 5;                      //SOC1 vai converter o pino ADCINA5 //J7 pin66
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;                  //sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;  //trigger on ePWM1 SOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x01;               //end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                    //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                  //make sure INT1 flag is cleared

    EDIS;
}


void Setup_ADC_B(void){
    Uint16 acqps;
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if (ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14;             //aproximadamente 75ns
    else                        //16 bits de resolucao
        acqps = 63;             //aproximadamente 320ns

    EALLOW;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;                      //seta para ADCCLK divitor por 4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;                   //seta pulso um ciclo antes do resultado
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ =1;                       //liga o ADC
    DELAY_US(1000);                                         //delay de 1ms pra permitir q os ADC ligem

    //SOC 0 do modulo B - Vc
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;                     //SOC0 vai converter o pino ADCINB3 //J3 pin25
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;                  //sample window is 15 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;  //trigger on ePWM1 SOCA

    // a interrupcao do modulo B n esta habilitada, esta somente avisa que finalizou a conversao
    // quem gera a interrupcao mesmo eh o modulo A
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x00;               //end of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;                    //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                  //make sure INT1 flag is cleared

    EDIS;
}



void Setup_DAC_A(void){
    EALLOW;
    //adicionado DAC A - J3 pin30 ADCINA0 DACOUTA
    DacaRegs.DACCTL.bit.SYNCSEL = 0x00; //0 EPWM1SYNCPER //DAC EPWMSYNCPER select
    DacaRegs.DACCTL.bit.LOADMODE = 0x01; //Determines when the DACVALA register is updated with the value from DACVALS
    //1 Load on next EPWMSYNCPER specified by SYNCSEL
    DacaRegs.DACCTL.bit.DACREFSEL = 0x01; //DAC reference select. Selects which voltage references are used by the DAC. (3v ou 3.3v)
    DacaRegs.DACVALS.bit.DACVALS = 0;   //12bits
    DacaRegs.DACOUTEN.bit.DACOUTEN =    1;
    DacaRegs.DACLOCK.all = 0x00;
    EDIS;
}

void Setup_DAC_B(void){
    EALLOW;
    //adicionado DAC B - J7 pin70 ADCINA1 DACOUTB
    DacbRegs.DACCTL.bit.SYNCSEL = 0x00; //0 EPWM1SYNCPER //DAC EPWMSYNCPER select
    DacbRegs.DACCTL.bit.LOADMODE = 0x01; //Determines when the DACVALA register is updated with the value from DACVALS
    //1 Load on next EPWMSYNCPER specified by SYNCSEL
    DacbRegs.DACCTL.bit.DACREFSEL = 0x01; //DAC reference select. Selects which voltage references are used by the DAC. (3v ou 3.3v)
    DacbRegs.DACVALS.bit.DACVALS = 0;   //12bits
    DacbRegs.DACOUTEN.bit.DACOUTEN =    1;
    DacbRegs.DACLOCK.all = 0x00;
    EDIS;
}

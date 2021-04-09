#include "CLA1_Config.h"
/*
 * CLA_Config.c
 *
 *  Created on: 30 de mar de 2020
 *      Author: waner
 */
void CLA1_ConfigCLAMemory(void){
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;
#ifdef _FLASH
    // Copy over code from FLASH to RAM
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart, (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH
    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};
    // Initialize and wait for CPUToCLA1MsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};
    // Select LS5RAM to be the memory space for the CLA (1) or CPU (0)
    // First configure the CLA to be the master for LS5 and then set the space to be a program block (1)
    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1; //se 0 eh propriedade da CPU, se 1 CLA tem acesso
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1; //se 1 eh memoria de programa, se 0 eh memoria de dado
    // Select LS0RAM and  LS1RAM to be the memory space for the CLA (1)
    // First configure the CLA to be the master for LS0/LS1 and then set the space to be a data block (0)
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1; //se 0 eh propriedade da CPU, se 1 CLA tem acesso
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0; //se 1 eh memoria de programa, se 0 eh memoria de dado
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1; //se 0 eh propriedade da CPU, se 1 CLA tem acesso
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0; //se 1 eh memoria de programa, se 0 eh memoria de dado
    EDIS;
}

void CLA1_InitCpu1Cla1(void){
    EALLOW;

    Cla1Regs.MVECT1 = (uint16_t)(&CLA1Task1);   //passar os enderecos das funcoes
    Cla1Regs.MVECT2 = (uint16_t)(&CLA1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&CLA1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&CLA1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&CLA1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&CLA1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&CLA1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&CLA1Task8);

    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.all   = M_INT1;               // Enable the respective tasks  //habilitada somente a tarefa 1

    // Configure the vectors for the end-of-task interrupt for all 8 tasks
    // toda vez que uma tarefa eh finalizada, ela pode ou nao disparar uma interrupcao, se for usar, esses sao os enderecos la no main
    PieVectTable.CLA1_1_INT   = &CLA1_isr1;
    PieVectTable.CLA1_2_INT   = &CLA1_isr2;
    PieVectTable.CLA1_3_INT   = &CLA1_isr3;
    PieVectTable.CLA1_4_INT   = &CLA1_isr4;
    PieVectTable.CLA1_5_INT   = &CLA1_isr5;
    PieVectTable.CLA1_6_INT   = &CLA1_isr6;
    PieVectTable.CLA1_7_INT   = &CLA1_isr7;
    PieVectTable.CLA1_8_INT   = &CLA1_isr8;

    // exemplo para executar triggar com software
    // Table 6-1 spruhm8i.pdf
    // 0-> software, Cla1ForceTask1andWait() or Cla1ForceTask1();
    // Cla1Regs.MIRUN.bit.INT1 = 1, task in execution
    // asm(" EALLOW"); //enable protected register access
    // Cla1Regs.MIFRC.bit.INT4 = 1; //start task 4
    // asm(" EDIS"); //disable protected register access

    /******** TRIGGER SOURCE FOR EACH TASK (unlisted numbers are reserved) ********/
    // 0=software   8=ADCBINT3  16=ADCDINT1  32=XINT4     42=EPWM7INT   70=TINT2     78=ECAP4INT   95=SD1INT     114=SPIRXINTC
    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT
    // 2=ADCAINT2  10=ADCBEVT   18=ADCDINT3  36=EPWM1INT  44=EPWM9INT   72=MREVTA    80=ECAP6INT  107=UPP1INT
    // 3=ADCAINT3  11=ADCCINT1  19=ADCDINT4  37=EPWM2INT  45=EPWM10INT  73=MXEVTB    83=EQEP1INT  109=SPITXINTA
    // 4=ADCAINT4  12=ADCCINT2  20=ADCDEVT   38=EPWM3INT  46=EPWM11INT  74=MREVTB    84=EQEP2INT  110=SPIRXINTA
    // 5=ADCAEVT   13=ADCCINT3  29=XINT1     39=EPWM4INT  47=EPWM12INT  75=ECAP1INT  85=EQEP3INT  111=SPITXINTB
    // 6=ADCBINT1  14=ADCCINT4  30=XINT2     40=EPWM5INT  48=TINT0      76=ECAP2INT  87=HRCAP1INT 112=SPIRXINTB
    // 7=ADCBINT2  15=ADCCEVT   31=XINT3     41=EPWM6INT  69=TINT1      77=ECAP3INT  88=HRCAP2INT 113=SPITXINTC

    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 1; //a tarefa um corresponde ao trigger do ADC A1
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 0;     // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL2 = 0;     // Write a 1 to lock (cannot be cleared once set)
    // Enable CLA interrupts at the group and subgroup levels
    //PieCtrlRegs.PIEIER11.bit.INTx1 = 1;
    //IER |= (M_INT11 );
    EDIS;
}


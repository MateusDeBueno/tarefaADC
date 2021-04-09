/*
 * CLA1.h
 *
 *  Created on: 8 de abr de 2021
 *      Author: Mateus
 */

#ifndef _CLA_H_
#define _CLA_H_

#include "F2837xD_Cla_defines.h"
#include "CLAMath.h"            //funcoes matematicas exclusivas para usar no CLA
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "sogi.h"               //vou fazer o PLL no CLA

// shared variables
extern volatile float vrede_CLA;
extern volatile SPLL_SOGI cla1_pll;
extern volatile float phase_CLA;
extern volatile float ampl_CLA;
//

//Task 1 (ASM) Variables

//Task 2 (ASM) Variables

//Task 3 (ASM) Variables

//Task 4 (ASM) Variables

//Task 5 (ASM) Variables

//Task 6 (ASM) Variables

//Task 7 (ASM) Variables

//Task 8 (ASM) Variables

//Common (ASM) Variables

//*****************************************************************************
// function prototypes
//*****************************************************************************

//CLA ASM Tasks
__interrupt void CLA1Task1();
__interrupt void CLA1Task2();
__interrupt void CLA1Task3();
__interrupt void CLA1Task4();
__interrupt void CLA1Task5();
__interrupt void CLA1Task6();
__interrupt void CLA1Task7();
__interrupt void CLA1Task8();

#endif //end of _CLA_H_ definition

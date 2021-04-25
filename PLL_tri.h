/*
 * PLL_tri.h
 *
 *  Created on: 9 de abr de 2021
 *      Author: Mateus
 */

#ifndef PLL_TRI_H_
#define PLL_TRI_H_

#include "F28x_Project.h"


typedef struct{
    float Vab,Vbc,Vca;
    float Valpha, Vbeta;
    float erroc[3], erro[3];
    float wt[2];
    float VabPLL, VbcPLL, VcaPLL;
    uint32_t count;
}PLL_tri;

void find_wt(volatile float Vin_A, volatile float Vin_B, volatile float Vin_C);

//extern volatile float Va_unit, Vb_unit, Vc_unit;
extern volatile PLL_tri PLL;
extern volatile uint32_t timer_count;


#endif /* PLL_TRI_H_ */

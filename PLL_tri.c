/*
 * PLL_tri.c
 *
 *  Created on: 9 de abr de 2021
 *      Author: Mateus
 */

#include "PLL_tri.h"
#include "math.h"


#define Ts 0.00002
#define c1 64.96
#define c2 -64.93


void find_wt(volatile float Vin_A, volatile float Vin_B, volatile float Vin_C){
    PLL.Vab = Vin_A - Vin_B;
    PLL.Vbc = Vin_B - Vin_C;
    PLL.Vca = Vin_C - Vin_A;

    PLL.count++;

    //calcula-se Valpha e Vbeta
    PLL.Valpha = 1.22474497*PLL.Vab;
    PLL.Vbeta = 0.707106781*PLL.Vab + 1.414213562*PLL.Vbc;

    //calculo erro
    PLL.erro[2] = PLL.erro[1];
    PLL.erro[1] = PLL.erro[0];
    PLL.erro[0] = __cos(PLL.wt[0]*PLL.count*Ts)*PLL.Valpha + __sin(PLL.wt[0]*PLL.count*Ts)*PLL.Vbeta;

    //CONTROLADOR//
    PLL.erroc[2] = PLL.erroc[1];
    PLL.erroc[1] = PLL.erroc[0];
    PLL.erroc[0] = PLL.erro[0]*c1+PLL.erro[1]*c2 + PLL.erroc[1];

    //INTEGRAR A AÇÃO DE CONTROLE
    PLL.wt[1]=PLL.wt[0];
    PLL.wt[0]=PLL.wt[0]+Ts/2*(PLL.erroc[0]+PLL.erroc[1]);

    PLL.VabPLL = 1.732050808*__sin(PLL.wt[0]*PLL.count*Ts);
    PLL.VbcPLL = 1.732050808*__sin(PLL.wt[0]*PLL.count*Ts-2.0943951);
    PLL.VcaPLL = 1.732050808*__sin(PLL.wt[0]*PLL.count*Ts+2.0943951);
}

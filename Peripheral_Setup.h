/*
 * Peripheral_Setup.h
 *
 *  Created on: 7 de abr de 2021
 *      Author: Mateus
 */

#ifndef PERIPHERAL_SETUP_H_
#define PERIPHERAL_SETUP_H_

#include "F28x_Project.h"
#include "math.h"

void Setup_GPIO(void);
void Setup_ePWM(void);
void Setup_ADC_A(void);
void Setup_ADC_B(void);
void Setup_DAC_A(void);
void Setup_DAC_B(void);

//criar define manual para selecionar trigger do ADC
#define TRIG_SEL_SOFT   0x00
#define TRIG_SEL_TIM0   0x01
#define TRIG_SEL_TIM1   0x02
#define TRIG_SEL_TIM2   0x03
#define TRIG_SEL_GPIO   0x04
#define TRIG_SEL_ePWM1_SOCA 0x05
#define TRIG_SEL_ePWM1_SOCB 0x06
#define TRIG_SEL_ePWM2_SOCA 0x07
#define TRIG_SEL_ePWM2_SOCB 0x08
#define TRIG_SEL_ePWM3_SOCA 0x09
#define TRIG_SEL_ePWM3_SOCB 0x0A
#define TRIG_SEL_ePWM4_SOCA 0x0B
#define TRIG_SEL_ePWM4_SOCB 0x0C
#define TRIG_SEL_ePWM5_SOCA 0x0D
#define TRIG_SEL_ePWM5_SOCB 0x0E
#define TRIG_SEL_ePWM6_SOCA 0x0F
#define TRIG_SEL_ePWM6_SOCB 0x10
#define TRIG_SEL_ePWM7_SOCA 0x11
#define TRIG_SEL_ePWM7_SOCB 0x12
#define TRIG_SEL_ePWM8_SOCA 0x13
#define TRIG_SEL_ePWM8_SOCB 0x14
#define TRIG_SEL_ePWM9_SOCA 0x15
#define TRIG_SEL_ePWM9_SOCB 0x16
#define TRIG_SEL_ePWM10_SOCA    0x17
#define TRIG_SEL_ePWM10_SOCB    0x18
#define TRIG_SEL_ePWM11_SOCA    0x19
#define TRIG_SEL_ePWM11_SOCB    0x1A
#define TRIG_SEL_ePWM12_SOCA    0x1B
#define TRIG_SEL_ePWM12_SOCB    0x1C
#define TRIG_SEL_CPU2_TIM0  0x1D
#define TRIG_SEL_CPU2_TIM1  0x1E
#define TRIG_SEL_CPU2_TIM2  0x1F

#endif /* PERIPHERAL_SETUP_H_ */

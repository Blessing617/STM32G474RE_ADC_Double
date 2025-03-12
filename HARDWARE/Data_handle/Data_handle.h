#ifndef _Data_handle_H
#define _Data_handle_H

#include "main.h" 
#include "adc.h"
#include "arm_math.h"
#include "OLED_I2C.h"
#include "tim.h"
#include "myusart1.h"
#include "delay.h"

void Fre_Control(uint32_t Fre);
void ADC_Init(void);
void ADC_Get(void);
void Data_handle(void);
int fft_getpeak(float *inputx,float *input,float *output,uint16_t inlen,uint8_t x,uint8_t N,float y);

#endif



/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#define Narr 200
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "adc.h"
#include "rti.h"
#include "het.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
adcData_t adc_data[2];
int32_t eh[Narr];

//VARIABLES GLOBALES
int32_t sens_mm=0; POT_mm=0,clmin=0,clmax=0,refe=0, j=0, k=0;
float e[5];
float kp=2.6,ki=0.06,kd=0.23,Uc=0,Ui=0,Ud=0,T=0.02,Up=0;

float ALPHA=1,previous_val=1;

hetSIGNAL_t miPWM;
static const uint32 s_het1pwmPolarity[8U] =
{
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
};

void pwmSetSignalMIO(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal);

float ema_filter(float a)
{
    float result;

    result = ALPHA * a + (1-ALPHA)*previous_val;
    previous_val=a;
    return result;
}

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    rtiInit();
    adcInit();
    hetInit();
    double mvolts=0;
    int32_t ch_count;
    int32_t adc_sen[3]={0}, med[3],sto, sens_mv=0;
    uint32_t i;
    float sens=0;
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    _enable_interrupt_();
    while(1)
    {
        adcStartConversion(adcREG1, adcGROUP1);
        while((adcIsConversionComplete(adcREG1, adcGROUP1)==0));
        ch_count = adcGetData(adcREG1, adcGROUP1, &adc_data[0]);

        ///////////   LEER SENSOR   ///////////
        adc_sen[0]=adc_sen[1]; //RECORRE LOS DATOS PARA adc_mv[];
        adc_sen[1]=adc_sen[2];
        adc_sen[2] = adc_data[0].value; //OBTIENE SEÑAL DEL SENSOR

        for(i=0;i<3;i++){//HACER IGUALES EL ARREGLO DE DATOS CON EL ARREGLO DEL QUE OBTENDREMOS LA MEDIANA
            med[i]=adc_sen[i];
        }

        while(med[0]>med[1] || med[1]>med[2]){//FUNCION PARA ORDENAR EL ARREGLO DE LA MEDIANA DE MENOR A MAYOR
            if(med[0]>med[1]){
                sto=med[0];
                med[0]=med[1];
                med[1]=sto;
            }
            else{
                sto=med[1];
                med[1]=med[2];
                med[2]=sto;
            }
        }

        sens=ema_filter((float)med[1]);


        sens_mv=((int32_t)sens*3100)/4096;
        mvolts=(double)sens_mv;
        sens_mm=(int)(10*pow((147737/(mvolts*10)),1.2134));//DISTANCIA EN MM DEL SENSOR
        if(sens_mm>230){
                    sens_mm=230;
                }

        ///////////   LEER POT   ///////////
        POT_mm=50+((181*adc_data[1].value)/4096);

        miPWM.period = 20000;

        //sciSend(scilinREG,sprintf(command,"\r %i\n", sens_mm),(uint8 *)command);
    }
/* USER CODE END */
    return 0;
}


/* USER CODE BEGIN (4) */
void rtiNotification(uint32 notification)
{

    //corrimiento de errores
    e[4]=e[3];
    e[3]=e[2];
    e[2]=e[1];
    e[1]=e[0];

    e[0]=(float)(-sens_mm+POT_mm);

    j++;
        if(j>=Narr)
        {
            j=0;
        }
        for ( k = 1; k < Narr; ++k) {
            eh[(k-1)%Narr]=eh[k%Narr];
        }
        e[0] = (float)(-sens_mm+POT_mm);  //REF-medicion
        eh[Narr-1]=(int32_t)(10*e[0]);

    Up=kp*e[0];
    Ui=Ui+ki*T*e[0];
    Ud=(kd/T)*(e[0]-e[4]);

    limitControl(-100,100,&Ui);
    Uc=Up+Ui+Ud;
    limitControl(-400,400,&Uc);
    miPWM.duty=650+(int32_t)Uc;
    pwmSetSignalMIO(hetRAM1,pwm0,miPWM);
}

void limitControl(int32_t MIN, int32_t MAX, float *x){
    float *aux=x;
    if(*aux<MIN){*aux=MIN;}
    if(*aux>MAX){*aux=MAX;}
}

void pwmSetSignalMIO(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal)
{
    uint32 action;
    uint32 pwmPolarity = 0U;
    float64 pwmPeriod = 0.0F;

    if(hetRAM == hetRAM1)
    {
        pwmPeriod = (signal.period * 1000.0F) / 640.000F;
        pwmPolarity = s_het1pwmPolarity[pwm];
    }
    else
    {
    }
    if (signal.duty == 0U)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (signal.duty >= 10000U)
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control) & (~(uint32)(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((((uint32)pwmPeriod * signal.duty) / 10000U) << 7U ) + 128U;
    hetRAM->Instruction[(pwm << 1U) + 42U].Data = ((uint32)pwmPeriod << 7U) - 128U;

}
/* USER CODE END */

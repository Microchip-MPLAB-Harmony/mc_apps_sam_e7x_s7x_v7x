/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    pfc_app.c

  Summary:
    This file contains the power factor correction functions.

  Description:
    This file contains the power factor correction functions.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "definitions.h"                // SYS function prototypes
#include "pfc_app.h"
#include "userparams.h"
#include "X2Cscope.h"
#include "X2CscopeComm.h"
#include "math.h"

#define PFC_CURRENT_OFFSET_SAMPLES   100U
#define PFC_CURRENT_OFFSET_MIN       1900U
#define PFC_CURRENT_OFFSET_MAX       2100U
static uintptr_t dummyforMisra;
static float PFC_IacOffset_df32;
#if(PFC_ENABLE == 1U)
/******************************************************************************/
/* Structure declaration  */
/******************************************************************************/
static PFCAPP_CONTROL_PARAM      gPfcControlParam;
static PFCAPP_AC_VOLTAGE         gAcVoltageParam;
static PFCAPP_DC_BUS_VOLTAGE     gDcVoltageParam;
static PFCAPP_AC_CURRENT         gAcCurrentParam;
static PFCAPP_AVG_FILTER_STATE   gVacAvgFilter;
static PFCAPP_AVG_FILTER_STATE   gIacAvgFilter;

/******************************************************************************/
/* Extern                                                                     */
/******************************************************************************/
 MCLIB_PI  gPIParmIpfc;            /* PFC Current PI controllers */
 MCLIB_PI  gPIParmVpfc;            /* PFC Voltage PI controllers */

/******************************************************************************/
/*                  FUNCTION DEFINATION                                       */
/******************************************************************************/
/******************************************************************************/
/* Function name: PFCAPP_init                                                 */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: PFC Controller and filter initialization                      */
/******************************************************************************/
void PFCAPP_init(void)
{
    gPfcControlParam.firstPass = ENABLE;
    gPfcControlParam.kmul = KMUL;
    gPfcControlParam.rampRate = PFC_SOFT_START_RAMP_PRESCALER;
    gDcVoltageParam.filterCoeff = KFILTER_PFC_BUS_VOLTAGE;
    gAcVoltageParam.avgOutput = VAC_AVG_200V; // average ac voltage for 200 RMS

    gPIParmIpfc.kp = PFC_CURRCNTR_PTERM;
    gPIParmIpfc.ki = PFC_CURRCNTR_ITERM;
    gPIParmIpfc.kc = PFC_CURRCNTR_CTERM;
    gPIParmIpfc.outMax = PFC_CURRCNTR_OUTMAX;
    gPIParmIpfc.outMin = 0.0f;
    gPIParmIpfc.dSum = 0.0f;
    gPIParmIpfc.out = 0.0f;

    gPIParmVpfc.kp = PFC_VOLTAGE_PTERM;
    gPIParmVpfc.ki = PFC_VOLTAGE_ITERM;
    gPIParmVpfc.kc = PFC_VOLTAGE_CTERM;
    gPIParmVpfc.outMax = PFC_VOLTAGE_OUTMAX;
    gPIParmVpfc.outMin = 0.0f;
    gPIParmVpfc.dSum = 0.0f;
    gPIParmVpfc.out = 0.0f;

    gVacAvgFilter.B0 = B0_AVG_FILT;
    gVacAvgFilter.B1 = B1_AVG_FILT;
    gVacAvgFilter.B2 = B2_AVG_FILT;
    gVacAvgFilter.A1 = A1_AVG_FILT;
    gVacAvgFilter.A2 = A2_AVG_FILT;

    gIacAvgFilter.B0 = B0_AVG_FILT;
    gIacAvgFilter.B1 = B1_AVG_FILT;
    gIacAvgFilter.B2 = B2_AVG_FILT;
    gIacAvgFilter.A1 = A1_AVG_FILT;
    gIacAvgFilter.A2 = A2_AVG_FILT;
}


/******************************************************************************/
/* Function name: PFCAPP_offsetCalibration                                    */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Offset calibration                                            */
/******************************************************************************/


static void PFCAPP_offsetCalibration(void)
{
    uint32_t AdcSampleCounter = 0;
    int32_t delayCounter = 0xFFFF;
    uint32_t IACOffsetBuffer = 0;

    /* Disable interrupt generation */
    AFEC1_ChannelsInterruptDisable( AFEC_INTERRUPT_EOC_6_MASK );

    /* Software trigger used for ADC conversion. */
    AFEC1_REGS->AFEC_MR &= ~(AFEC_MR_TRGEN_Msk);

    for(AdcSampleCounter = 0; AdcSampleCounter < PFC_CURRENT_OFFSET_SAMPLES; AdcSampleCounter++)
    {
        /* Software trigger for phase current measurements */
        AFEC1_ConversionStart();

        /* Delay to stabilize voltage levels on board and adc conversion to complete */
        do
        {
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            delayCounter--;
        } while (delayCounter > 0);

        /* re-load delay counter for next adc sample */
        delayCounter = 0xFFFF;

        IACOffsetBuffer  += (uint32_t)AFEC1_ChannelResultGet((AFEC_CHANNEL_NUM)PFC_CURRENT_ADC_CH );
    }
    PFC_IacOffset_df32 = (float)((float)IACOffsetBuffer/(float)PFC_CURRENT_OFFSET_SAMPLES);

    /* Limit motor phase B current offset calibration to configured Min/Max levels. */
    if(PFC_IacOffset_df32 >  (float)PFC_CURRENT_OFFSET_MAX)
    {
        PFC_IacOffset_df32 = (float)PFC_CURRENT_OFFSET_MAX;
    }
    else if(PFC_IacOffset_df32 <  (float)PFC_CURRENT_OFFSET_MIN)
    {
        PFC_IacOffset_df32 = (float)PFC_CURRENT_OFFSET_MIN;
    }else
    {
        /* Dummy branch for MISRAC compliance*/
    }

    /* Enable adc end of conversion interrupt generation to execute FOC loop */
    AFEC1_ChannelsInterruptEnable( AFEC_INTERRUPT_EOC_6_MASK);

    /* Set adc trigger from PWM event lines */
    AFEC1_REGS->AFEC_MR |= (AFEC_MR_TRGEN_Msk);
}

 /******************************************************************************/
 /* Function name: PFCAPP_AvgFilter                                    */
 /* Function parameters: pParam                                                  */
 /* Function return: None                                                      */
 /* Description: This function implements a 2nd order IIR Filter with Cut-off  */
 /*              frequency of 15Hz. This filter is used to calculate average   */
 /*              values of AC line voltage and inductor current                */
 /******************************************************************************/


static inline void PFCAPP_AvgFilter(PFCAPP_AVG_FILTER_STATE *pParam)
{
    pParam->y_n  =(pParam->A1*pParam->y_n_1) ;
    pParam->y_n +=(pParam->A2*pParam->y_n_2);
    pParam->y_n +=(pParam->B0*pParam->x_n);
    pParam->y_n +=(pParam->B1*pParam->x_n_1);
    pParam->y_n +=(pParam->B2*pParam->x_n_2);

    pParam->y_n_2 = pParam->y_n_1;
    pParam->y_n_1 = pParam->y_n;

    pParam->x_n_2 = pParam->x_n_1;
    pParam->x_n_1 = pParam->x_n;
}

/******************************************************************************/
/* Function name: PFCAPP_faultsCheck                                          */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: This function checks for a input under voltage/over voltage,  */
/*              output under voltage/over voltage and over current  faults    */
/******************************************************************************/

__STATIC_INLINE void PFCAPP_faultsCheck(void)
{
    if(gPfcControlParam.softStart == DISABLE)
    {
        /* DC bus under voltage and over voltage faults */
        if(gDcVoltageParam.measured >= PFC_DC_BUS_OVER_VOLTAGE)
        {
            gPfcControlParam.overvoltage_faultCount++;
            if(gPfcControlParam.overvoltage_faultCount  > 200U)
            {
                gPfcControlParam.faultBit = 1U;
                gPfcControlParam.overvoltage_faultCount=0U;
            }

        }
        /* AC input under voltage and over voltage faults */
        else if((gAcVoltageParam.avgOutput >= PFC_AC_OVER_VOLTAGE) || (gAcVoltageParam.avgOutput <= PFC_AC_UNDER_VOLTAGE))
        {
            gPfcControlParam.ac_voltage_faultCount++;
            if(gPfcControlParam.ac_voltage_faultCount  > 200U)
            {
                gPfcControlParam.faultBit = 1U;
                gPfcControlParam.ac_voltage_faultCount=0U;
            }
        }
        /* AC input over current fault */
        else if(gAcCurrentParam.avgOutput >= PFC_OVER_CURRENT_AVG)
        {
            gPfcControlParam.overcurrent_faultCount++;
            if(gPfcControlParam.overcurrent_faultCount  > 200U)
            {
                gPfcControlParam.faultBit = 1U;
                gPfcControlParam.overcurrent_faultCount=0U;
            }
        }
        else
        {
            gPfcControlParam.overvoltage_faultCount=0U;
            gPfcControlParam.overcurrent_faultCount=0U;
            gPfcControlParam.ac_voltage_faultCount=0U;
        }
        if(gPfcControlParam.faultBit == 1U)
        {
            PFCAPP_Disable();
        }
    }
}

/******************************************************************************/
/* Function name: PFCAPP_voltageControl                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: in this Function the voltage loop PI related calculations     */
/*                 are performed                                              */
/******************************************************************************/
__STATIC_INLINE void PFCAPP_voltageControl(void)
{
    /*Run voltage control loop for every 4Khz */
    if (gPfcControlParam.voltLoopExeRate >= PFC_VOLTAGE_LOOP_PRESCALER)
    {
        gPIParmVpfc.inMeas = gDcVoltageParam.filtered;
        MCLIB_PIControl( &gPIParmVpfc );
        gPfcControlParam.voltLoopExeRate  = 0;
    }
    else
    {
        gPfcControlParam.voltLoopExeRate ++;
    }
}

/******************************************************************************/
/* Function name: PFCAPP_dcmCompensation                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: This function generates the constant that is used to reshape  */
/*              the current waveform into a better rectified sinusoidal wave  */
/*              under light load conditions and zero crossings                */
/******************************************************************************/
__STATIC_INLINE void PFCAPP_dcmCompensation(void)
{
     float temp;

     /* DCM Mode Compensation = DutyCycle * VDC/(VDC-VAC) */

     temp = (float)(gDcVoltageParam.measured - gAcVoltageParam.measured);
     if(temp > 0.0f)
     {
        /* Dividing Vdc */
        temp = (float)(gDcVoltageParam.measured/temp);

        /* Calculating sample correction factor */
        gPfcControlParam.sampleCorrection = (gPIParmIpfc.out * temp) ;

        /* Set the correction factor to 1 if the PWM duty is outside range */
        if((gPfcControlParam.sampleCorrection <= 0.0f) || (gPfcControlParam.sampleCorrection >= 1.0f))
        {
            gPfcControlParam.sampleCorrection = 1.0f;
        }
      }
}


/******************************************************************************/
/* Function name: PFCAPP_PowerFactCorrISR                                     */
/* Function parameters: status, context                                       */
/* Function return: None                                                      */
/* Description: in this Function the adc channels are configured to sample    */
/*              & convert the MC signals and it also executes the PFC control */
/*              loops                                                         */
/******************************************************************************/
void PFCAPP_PowerFactCorrISR(uint32_t status, uintptr_t context)
{
    float i_currentRef_df32;
    float i_currentMeasured_df32;
    float v_dcBusVoltage_df32;
    float v_acBusVoltage_df32;

    /* Read the DC voltage from ADC channel */
    v_dcBusVoltage_df32 = (float)((uint32_t)AFEC1_ChannelResultGet((AFEC_CHANNEL_NUM) PFC_DC_BUS_VOLTAGE_ADC_CH ));
    gDcVoltageParam.measured = (float)(v_dcBusVoltage_df32) * VOLTAGE_ADC_TO_PHY_RATIO;

    /* Filter DC Bus voltage remove 100Hz/ 120Hz Ripple */
    gDcVoltageParam.filtered =  gDcVoltageParam.filtered + gDcVoltageParam.filterCoeff
                                    * ( gDcVoltageParam.measured - gDcVoltageParam.filtered );

    /* Measure current for power factor correction */
    i_currentMeasured_df32 = (float)((uint32_t)AFEC1_ChannelResultGet( (AFEC_CHANNEL_NUM)PFC_CURRENT_ADC_CH )) ;

    gAcCurrentParam.measured = (float)((i_currentMeasured_df32 - PFC_IacOffset_df32) * PFC_ADC_CURR_SCALE);

    if ( gAcCurrentParam.measured <= 0.0f)
    {
        gAcCurrentParam.measured =(float)PFC_ADC_CURR_SCALE;
    }

    /* Get AC voltage from ADC channel */
    v_acBusVoltage_df32 = (float)((uint32_t)AFEC1_ChannelResultGet( (AFEC_CHANNEL_NUM)PFC_VOLTAGE_ADC_CH));
    gAcVoltageParam.measured = (float)((v_acBusVoltage_df32 - (float)AC_VOLTAGE_OFFSET) * PFC_AC_VOLTAGE_ADC_TO_PHY_RATIO);

    if (gAcVoltageParam.measured >= PFC_AC_MAX_VOLTAGE_PEAK)
    {
        gAcVoltageParam.measured = PFC_AC_MAX_VOLTAGE_PEAK;
    }

    /* Average AC line Voltage Calculation */
    gVacAvgFilter.x_n = gAcVoltageParam.measured;
    PFCAPP_AvgFilter( &gVacAvgFilter );
    gAcVoltageParam.avgOutput = gVacAvgFilter.y_n;
    gAcVoltageParam.avgSquare = (float)(gVacAvgFilter.y_n * gVacAvgFilter.y_n);

    /*Average input Current Calculation */
    gIacAvgFilter.x_n = gAcCurrentParam.measured;
    PFCAPP_AvgFilter( &gIacAvgFilter );
    gAcCurrentParam.avgOutput = gIacAvgFilter.y_n;

    if((( gPfcControlParam.firstPass == ENABLE && gAcVoltageParam.avgOutput >= VAC_AVG_88V )
        || ( gPfcControlParam.firstPass == DISABLE && gDcVoltageParam.measured >= PFC_AC_MIN_VOLTAGE_PEAK ))
        &&   gPfcControlParam.faultBit == 0U )
    {
        if (gPfcControlParam.firstPass == ENABLE)
        {
                   /* Reference voltage is a measured DC bus voltage as a start point. */
                  gPIParmVpfc.inRef  = gDcVoltageParam.measured;
                  gPfcControlParam.firstPass = DISABLE;
                  gPfcControlParam.softStart = ENABLE;
        }
        else
        {
            if (gPfcControlParam.softStart == ENABLE)
            {
                /* Check if Voltagereftemp is less than given reference voltage  and ramp it slowly */
                if (gPIParmVpfc.inRef  <= PFC_DC_BUS_VOLTAGE_REF)
                {
                    if(gPfcControlParam.rampRate == 0U)
                    {
                        gPIParmVpfc.inMeas = gDcVoltageParam.filtered;
                        gPIParmVpfc.inRef  = gPIParmVpfc.inRef  + (float)PFC_SOFT_START_STEP_SIZE;
                        gPfcControlParam.rampRate = PFC_SOFT_START_RAMP_PRESCALER;
                    }
                }
                else
                {
                    gPIParmVpfc.inRef  = PFC_DC_BUS_VOLTAGE_REF;
                    if(gDcVoltageParam.measured >= PFC_DC_BUS_VOLTAGE_REF)
                    {
                        gPfcControlParam.softStart = DISABLE;
                        gPfcControlParam.pfc_good = 1;
                    }
                }
                gPfcControlParam.rampRate--;
            }
        }

        /* Voltage PI control */
        PFCAPP_voltageControl();

        if(gAcVoltageParam.avgSquare < 5861.0f) // 5861 = avg(85VACrms)^2
        {
             gAcVoltageParam.avgSquare = 5861.0f;
        }
        i_currentRef_df32 = (float) gPIParmVpfc.out * gAcVoltageParam.measured;
        gPIParmIpfc.inRef  =  (float) ((i_currentRef_df32 * gPfcControlParam.kmul)/gAcVoltageParam.avgSquare);

        /* Check if inductive current reference exceeds Over Current Peak value */
        if ( gPIParmIpfc.inRef  > PFC_OVER_CURRENT_PEAK)
        {
            /* If true, saturate it to Over Current Peak Value */
            gPIParmIpfc.inRef  = PFC_OVER_CURRENT_PEAK;
        }
        else if (gPIParmIpfc.inRef  <= 0.0f)
        {
            gPIParmIpfc.inRef  = 0.0f;
        }else
        {
            /* Dummy branch for MISRAC compliance*/
        }


        /* Calling sample correction  function to shape the current waveform under light loadsand near zero crossings */
        PFCAPP_dcmCompensation();

        /* Multiplying the measured AC current with sample correction factor */
        gAcCurrentParam.corrected = (float)(gAcCurrentParam.measured * gPfcControlParam.sampleCorrection);

        /* if Vavg greater than average of 200Vrms */
        if(gAcVoltageParam.avgOutput < VAC_AVG_200V )
        {
            gPIParmIpfc.inMeas = gAcCurrentParam.corrected;
        }
        else
        {
            /* Current Error Calculation */
            gPIParmIpfc.inMeas = gAcCurrentParam.measured;
        }

        /* Current control for power factor correction  */
        MCLIB_PIControl( &gPIParmIpfc );

        /* Current loop PI output and Multiplying it with PWM period */
        gPfcControlParam.duty  = (uint16_t)((float)(gPIParmIpfc.out * (float)PFC_PERIOD_TIMER_TICKS));

        /* Limit the PWM duty cycle */
        if (gPfcControlParam.duty  >= (uint16_t)MAX_PFC_DC)
        {
            gPfcControlParam.duty = PWM1_ChannelPeriodGet(PWM_CHANNEL_0);
        }
        else if (gPfcControlParam.duty  <= MIN_PFC_DC)
        {
            gPfcControlParam.duty  = MIN_PFC_DC;
        }else
        {
            /* Dummy branch for MISRAC compliance*/
        }

        /*Loading calculated value of PFC duty to PDC register*/
        PWM1_ChannelDutySet(PWM_CHANNEL_0, gPfcControlParam.duty);

        /* Sampling point is always chosen to be at half of duty */
        gPfcControlParam.samplePoint = PFC_PERIOD_TIMER_TICKS;

        /*Calling PFC fault function */
        PFCAPP_faultsCheck();
    }
}

/******************************************************************************/
/* Function name: PFCAPP_Enable                                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Enables PFC                                                   */
/******************************************************************************/
void PFCAPP_Enable(void)
{
    /* Enable PFC */
    GPIO_PA2_Set();

    /* ADC end of conversion interrupt generation for PFC control */
    NVIC_DisableIRQ(AFEC1_IRQn);
    NVIC_ClearPendingIRQ(AFEC1_IRQn);
    NVIC_SetPriority(AFEC1_IRQn, 1);
    AFEC1_CallbackRegister( PFCAPP_PowerFactCorrISR, (uintptr_t)dummyforMisra);
    NVIC_EnableIRQ(AFEC1_IRQn);
    AFEC1_ChannelsInterruptEnable(AFEC_INTERRUPT_EOC_6_MASK);

    /* Enable PFC PWM channel */
    PWM1_ChannelsStart( PWM_CHANNEL_0_MASK );
}

/******************************************************************************/
/* Function name: PFCAPP_Disable                                              */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Disables PFC                                                  */
/******************************************************************************/

void PFCAPP_Disable(void)
{
    /* Disable PFC PWM channel */
    PWM1_ChannelsStop(PWM_CHANNEL_0_MASK );

    /* Disable PFC */
    GPIO_PA2_Clear();

    /* Disable ADC end of conversion interrupt generation for PFC control */
    AFEC1_ChannelsInterruptDisable( AFEC_INTERRUPT_EOC_6_MASK );
    NVIC_DisableIRQ(AFEC1_IRQn);
    NVIC_ClearPendingIRQ(AFEC1_IRQn);
}

#endif

/******************************************************************************/
/* Function name: PFCAPP_Tasks                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: PFC State Machine                                           */
/******************************************************************************/
void PFCAPP_Tasks(void)
{
#if(1U == PFC_ENABLE)
   switch(gPfcControlParam.state)
   {
        case PFCAPP_STATE_INIT:
            ((pio_registers_t*)PIO_PORT_D)->PIO_PER = ~0xFFFFFFFEU; // Disable PWML output.
            NVIC_DisableIRQ(AFEC1_IRQn);
            NVIC_ClearPendingIRQ(AFEC1_IRQn);
            AFEC1_ChannelsInterruptDisable(AFEC_INTERRUPT_EOC_6_MASK);
            PFCAPP_offsetCalibration();
            PFCAPP_init();
            gPfcControlParam.state = PFCAPP_STATE_START;
            break;

        case PFCAPP_STATE_START:
            ((pio_registers_t*)PIO_PORT_D)->PIO_PDR = ~0xFFFFFFFEU; // Enable PWML output.
            PFCAPP_Enable();
            gPfcControlParam.state = PFCAPP_STATE_RUNNING;
            break;

        case PFCAPP_STATE_RUNNING:
            /* Do nothing. The functionality is taken care in the ISR */
            /* Improvement point: The failure diagnosis can be moved here */
            break;
       case PFCAPP_STATE_STOP:
            /* Do nothing */
            /* Improvement point: The disable function can be moved here */
            break;
       default:
            /* Undefined state: Should never come here */
           break;
   }
#endif
}

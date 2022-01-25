/*******************************************************************************
 Hardware abstraction functions 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_hardware_abstraction.c

  Summary:
   Hardware abstraction layer 

  Description:
  Hardware abstraction layer 
 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 * Copyright (C) 2021 Microchip Technology Inc. and its subsidiaries.
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


/*******************************************************************************
Headers inclusions
 *******************************************************************************/
#include <stdint.h>
#include "mc_hardware_abstraction.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcHal_ButtonState_s     mcHal_PressButtonState_mds;

/*******************************************************************************
 Private variables 
 *******************************************************************************/

/*******************************************************************************
 Interface variables 
 *******************************************************************************/

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/*! \brief Set PWM Channels of the voltage source inverter
 * 
 * Details.
 * Set PWM Channels of the voltage source inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_VoltageSourceInverterPwmSet( tmcHal_InverterInstanceId_e Id, uint16_t * duty )
{
    if( Id == 0u )
    {

            PWM0_ChannelDutySet(PWM_CHANNEL_0, duty[0] );
        PWM0_ChannelDutySet(PWM_CHANNEL_1, duty[1] );
        PWM0_ChannelDutySet(PWM_CHANNEL_2, duty[2] );
    }
    else 
     {
         
     }
}


/*! \brief Disable PWM Channels of the voltage source inverter
 * 
 * Details.
 * Disable PWM Channels of the voltage source inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_VoltageSourceInverterPwmDisable(void)
{
    PWM0_ChannelOverrideDisable( PWM_CHANNEL_0 );
    PWM0_ChannelOverrideDisable( PWM_CHANNEL_1 );
    PWM0_ChannelOverrideDisable( PWM_CHANNEL_2 );

}

/*! \brief Enable PWM Channels of the voltage source inverter
 * 
 * Details.
 * Enable PWM Channels of the voltage source inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_VoltageSourceInverterPwmEnable(void)
{
    PWM0_ChannelOverrideEnable( PWM_CHANNEL_0 );
    PWM0_ChannelOverrideEnable( PWM_CHANNEL_1 );
    PWM0_ChannelOverrideEnable( PWM_CHANNEL_2 );
   
}

/*! \brief Get voltage signal from ADC port
 * 
 * Details.
 * Get voltage signal from ADC port
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_VoltageSignalGet( void )
{
    return AFEC0_ChannelResultGet(AFEC_CH7);
}


/*! \brief Get phase A current from ADC port 
 * 
 * Details.
 * Get phase A current from ADC port 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PhaseACurrentSignalGet( uint8_t Id )
{
    return AFEC0_ChannelResultGet(AFEC_CH0);
}



/*! \brief Get voltage signal from ADC port
 * 
 * Details.
 * Get voltage signal from ADC port
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PhaseBCurrentSignalGet( uint8_t Id )
{
    return AFEC0_ChannelResultGet(AFEC_CH6);
   
}



/*! \brief Get potentiometer signal from ADC port
 * 
 * Details.
 * Get potentiometer signal from ADC port
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PotentiometerSignalGet( void )
{
    return AFEC0_ChannelResultGet(AFEC_CH10);
}


/*! \brief Button Polling 
 * 
 * Details.
 * Button polling 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHal_ButtonResponse( const tmcHal_ButtonState_e  buttonState,  void (*buttonFunction)(void) )
{
    switch(mcHal_PressButtonState_mds.state)
    {
        case buttonStateMachine_Ready:
        {
            if( buttonState_Pressed == buttonState )
            {
                buttonFunction();
                mcHal_PressButtonState_mds.debounceCounter = 0;
                mcHal_PressButtonState_mds.state = buttonStateMachine_Wait;
            }
        }
        break;

        case buttonStateMachine_Wait:
        {
            if( SW_DEBOUNCE_DLY_500MS <= mcHal_PressButtonState_mds.debounceCounter)
            {
                mcHal_PressButtonState_mds.state = buttonStateMachine_Ready;
                mcHal_PressButtonState_mds.debounceCounter = 0;
            }
            else
            {
                mcHal_PressButtonState_mds.debounceCounter++;
            }
        }
        break;
        default:
        {
              /* Should never come here */
        }
    }
}

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcInterruptDisable( void )
{
    NVIC_DisableIRQ(AFEC0_IRQn);
} 

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcInterruptEnable( void )
{
    NVIC_EnableIRQ(AFEC0_IRQn); 
} 


/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcInterruptClear( void )
{
    NVIC_ClearPendingIRQ(AFEC0_IRQn);
}


/*! \brief ADC Enable
 * 
 * Details.
 * ADC Enable
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_ADCEnable( void )
{

}

/*! \brief ADC callback function
 * 
 * Details.
 * ADC callback function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcCallBackRegister(ADC_CALLBACK  callback, uintptr_t context)
{
    AFEC0_CallbackRegister( callback, context);
}


/*! \brief PWM timer Start
 * 
 * Details.
 * PWM timer Start
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmTimerStart( void )
{
    PWM0_ChannelsStart(0x7); 
}

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmInterruptDisable( void )
{
    NVIC_DisableIRQ(PWM0_IRQn);
} 

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmInterruptEnable( void )
{
    NVIC_EnableIRQ(PWM0_IRQn);

} 


/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmInterruptClear( void )
{
    NVIC_ClearPendingIRQ(PWM0_IRQn);
}

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmEnable( void )
{

   
}

/*! \brief Get PWM period
 * 
 * Details.
 * Get PWM period
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PwmPeriodGet( void )
{
    return PWM0_ChannelPeriodGet( PWM_CHANNEL_0 );
}

/*! \brief PWM callback function
 * 
 * Details.
 * PWM callback function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmCallbackRegister( TCC_CALLBACK  callback, uintptr_t context )
{
    PWM0_CallbackRegister( callback, (uintptr_t)context);


}

/*! \brief Read Group 01 signal 
 * 
 * Details.
 * Read Group 01 signal
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group01SignalRead( uint16_t Id  )
{
    mcHalI_PhaseACurrent_gdu16 = AFEC0_ChannelResultGet(AFEC_CH0);
    mcHalI_PhaseBCurrent_gdu16 = AFEC0_ChannelResultGet(AFEC_CH6);

}

/*! \brief Read Group 02 signal 
 * 
 * Details.
 * Read Group 02 signal
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group02SignalRead( uint16_t Id  )
{
    mcHalI_DcBusVoltage_gdu16  = AFEC0_ChannelResultGet(AFEC_CH7);
    mcHalI_Potentiometer_gdu16 = AFEC0_ChannelResultGet(AFEC_CH10);

    
}

/*! \brief Re-assign ADC channels for  Group 01 signals and enable hardware trigger 
 * 
 * Details.
 * Re-assign ADC channels for  Group 01 signals and enable hardware trigger
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group01SignalHardwareTrigger( uint16_t Id  )
{

}

/*! \brief Re-assign ADC channels for  Group 02 signals and enable software trigger 
 * 
 * Details.
 * Re-assign ADC channels for  Group 02 signals and enable software trigger 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group02SignalSoftwareTrigger( uint16_t Id  )
{


}



/**
 * @brief 
 *    Hardware abstraction header file
 *
 * @Company 
 *    Microchip Technology Inc.
 *
 * @File Name
 *   mc_hardware_abstraction.h
 *
 * @Summary
 *   Header file which shares global variables and function prototypes.
 *
 * @Description
 *   This file contains the global variables and function prototypes for hardware abstraction.
 */

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef MCHAL_H
#define MCHAL_H


/*******************************************************************************
  Header inclusions
*******************************************************************************/
#include "mc_types.h"
#include "definitions.h"

/*******************************************************************************
 * Interface variables
*******************************************************************************/
extern uint16_t mcHalI_IaAdcInput_gdu16; /**< Phase A current ADC input */
extern uint16_t mcHalI_IbAdcInput_gdu16; /**< Phase B current ADC input */
extern uint16_t mcHalI_UbusAdcInput_gdu16; /**< DC bus voltage ADC input */
extern uint16_t mcHalI_Potentiometer_gdu16; /**< Potentiometer ADC input */
extern int16_t mcPwmI_Duty_gau16[3u];  /**< PWM duty cycle array */

/*******************************************************************************
 * User defined data structure
*******************************************************************************/


/*******************************************************************************
 * Static interface Functions
*******************************************************************************/

/**
 * @brief Get PWM period.
 *
 * @details
 * Get PWM period.
 *
 * @return PWM period.
 */
__STATIC_INLINE uint16_t mcHalI_PwmPeriodGet( void )
{
    return (uint16_t)PWM0_ChannelPeriodGet(PWM_CHANNEL_0);
}

/**
 * @brief Set the PWM inverter duty cycle.
 *
 * @details
 * Sets the PWM inverter duty.
 *
 * @param[in] dutyCycle Pointer to the duty cycle array.
 */
__STATIC_FORCEINLINE void mcHalI_InverterPwmSet( const int16_t * const dutyCycle )
{
    uint16_t period;
    uint16_t duty[3u] = {0u};

    period = (uint16_t)mcHalI_PwmPeriodGet();
    duty[0u] = period - (uint16_t)dutyCycle[0u];
    duty[1u] = period - (uint16_t)dutyCycle[1u];
    duty[2u] = period - (uint16_t)dutyCycle[2u];

    PWM0_ChannelDutySet(PWM_CHANNEL_0, duty[0u] );
    PWM0_ChannelDutySet(PWM_CHANNEL_1, duty[1u] );
    PWM0_ChannelDutySet(PWM_CHANNEL_2, duty[2u] );

}

/**
 * @brief Get Phase A current from ADC peripheral.
 *
 * @details
 * Get analog signals from ADC peripheral.

 * @param[out] Global variable 'mcHalI_IaAdcInput_gdu16'
 */
__STATIC_FORCEINLINE void mcHalI_PhaseACurrentGet( void )
{
    mcHalI_IaAdcInput_gdu16 = AFEC0_ChannelResultGet(AFEC_CH0);
}

/**
 * @brief Get Phase B current from ADC peripheral.
 *
 * @details
 * Get Phase B current from ADC peripheral.

 * @param[out] Global variable 'mcHalI_IbAdcInput_gdu16'
 */
__STATIC_FORCEINLINE void mcHalI_PhaseBCurrentGet( void )
{
    mcHalI_IbAdcInput_gdu16 =  AFEC0_ChannelResultGet(AFEC_CH6);
}

/**
 * @brief Get DC link voltage from ADC peripheral.
 *
 * @details
 * Get DC link voltage from ADC peripheral.

 * @param[out] Global variable 'mcHalI_UbusAdcInput_gdu16'
 */
__STATIC_FORCEINLINE void mcHalI_DcLinkVoltageGet( void )
{
    /** Get ADC value for DC bus voltage */
    mcHalI_UbusAdcInput_gdu16 = AFEC0_ChannelResultGet(AFEC_CH7);
}

/**
 * @brief Get potentiometer input from ADC peripheral.
 *
 * @details
 * Get potentiometer input from ADC peripheral.

 * @param[out] Global variable 'mcHalI_Potentiometer_gdu16'
 */
__STATIC_FORCEINLINE void mcHalI_PotentiometerInputGet( void )
{
    /** Get ADC value for DC bus voltage */
    mcHalI_Potentiometer_gdu16 = AFEC0_ChannelResultGet(AFEC_CH10);
}


/**
 * @brief Clear ADC interrupt flag
 *
 * @details
 * Clear ADC interrupt flag
 */
__STATIC_FORCEINLINE void mcHalI_AdcInterruptClear( void )
{
    NVIC_ClearPendingIRQ(AFEC0_IRQn);
}

/**
 * @brief ADC interrupt disable
 *
 * @details
 * ADC interrupt disable
 */
__STATIC_FORCEINLINE void mcHalI_AdcInterruptDisable( void )
{
    NVIC_DisableIRQ(AFEC0_IRQn);
}

/**
 * @brief ADC interrupt enable
 *
 * @details
 * ADC interrupt enable
 */
__STATIC_FORCEINLINE void mcHalI_AdcInterruptEnable( void )
{
    NVIC_EnableIRQ(AFEC0_IRQn);
}


/*******************************************************************************
 * Interface Functions
*******************************************************************************/
/**
 * @brief Enable three phase inverter
 *
 * @details
 * Enable three phase inverter
 */
void mcHalI_InverterPwmEnable( void );

/**
 * @brief Disable three phase inverter
 *
 * @details
 * Disable three phase inverter
 */
void mcHalI_InverterPwmDisable( void );


/**
 * @brief Set direction indication
 *
 * @details
 * Set direction indication
 */
void mcHal_DirectionIndication( void );

/**
 * @brief Fault indication led state set
 *
 * @details
 * Fault indication led state set
 */
void mcHal_FaultIndicationSet( void );

/**
 * @brief Enable ADC peripheral
 *
 * @details
 * Enable ADC peripheral
 */
void mcHalI_AdcEnable( void );

/**
 * @brief Start PWM timer
 *
 * @details
 * Start PWM timer
 */
void mcHalI_PwmTimerStart( void );

/**
 * @brief ADC conversion complete interrupt callback function
 *
 * @details
 * ADC conversion complete interrupt callback function
 */
void mcHalI_AdcCallBackRegister( AFEC_CALLBACK callback, uintptr_t context );

/**
 * @brief PWM fault interrupt callback function
 *
 * @details
 * PWM fault interrupt callback function
 */
void mcHalI_PwmCallbackRegister( PWM_CALLBACK callback, uintptr_t context );

/**
 * @brief Enable PWM fault interrupt
 *
 * @details
 * Enable PWM fault interrupt
 */
void mcHalI_PwmInterruptEnable( void );



/**
 * @brief Get start-stop button state
 *
 * @details
 * Get start-stop button state
 *
 * @return  True if button is pressed, false if button is not pressed
 */
bool mcHalI_StartStopButtonState( void );


/**
 * @brief Get direction button state
 *
 * @details
 * Get direction button state
 *
 * @return  True if button is pressed, false if button is not pressed
 */
bool mcHalI_DirectionButtonState( void );


#endif // MCHAL_H

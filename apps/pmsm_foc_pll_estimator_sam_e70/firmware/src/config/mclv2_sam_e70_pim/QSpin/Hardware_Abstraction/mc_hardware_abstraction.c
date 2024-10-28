/**
 * @brief 
 *    Hardware abstraction source file
 *
 * @Company 
 *    Microchip Technology Inc.
 *
 * @File Name
 *   mc_hardware_abstraction.c
 *
 * @Summary
 *   This file contains implementation of the hardware abstraction
 *
 * @Description
 *   This file contains implementation of the hardware abstraction
 */

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
#include "mc_hardware_abstraction.h"

/*******************************************************************************
 * Constants
*******************************************************************************/

/*******************************************************************************
 * Interface variables
*******************************************************************************/
uint16_t mcHalI_IaAdcInput_gdu16;         /**< Phase A current ADC input */
uint16_t mcHalI_IbAdcInput_gdu16;         /**< Phase B current ADC input */
uint16_t mcHalI_UbusAdcInput_gdu16;     /**< DC bus voltage ADC input   */
uint16_t mcHalI_Potentiometer_gdu16;      /**< Potentiometer ADC input     */
int16_t mcPwmI_Duty_gau16[3u];              /**< PWM duty cycle array        */

/*******************************************************************************
 * Interface Functions
*******************************************************************************/

/**
 * @brief Enable three phase inverter
 *
 * @details
 * Enable three phase inverter
 */
void mcHalI_InverterPwmEnable( void )
{
    PWM0_ChannelOverrideEnable( PWM_CHANNEL_0 );
    PWM0_ChannelOverrideEnable( PWM_CHANNEL_1 );
    PWM0_ChannelOverrideEnable( PWM_CHANNEL_2 );
}

/**
 * @brief Disable three phase inverter
 *
 * @details
 * Disable three phase inverter
 */
void mcHalI_InverterPwmDisable( void )
{
/** Disable PWM override*/
    PWM0_ChannelOverrideDisable( PWM_CHANNEL_0 );
    PWM0_ChannelOverrideDisable( PWM_CHANNEL_1 );
    PWM0_ChannelOverrideDisable( PWM_CHANNEL_2 );
}


/**
 * @brief Set direction indication
 *
 * @details
 * Set direction indication
 */
void mcHal_DirectionIndication( void )
{
    PIO_PinToggle( PIO_PIN_PC23);

}

/**
 * @brief Set fault indication
 *
 * @details
 * Set fault indication
 */
void mcHal_FaultIndicationSet( void )
{
    PIO_PinSet( PIO_PIN_PC23);

}

/**
 * @brief Enable ADC peripheral
 *
 * @details
 * Enable ADC peripheral
 */
void mcHalI_AdcEnable( void )
{
    /** Nothing to do */
    __NOP();
}

/**
 * @brief Enable PWM fault interrupt
 *
 * @details
 * Enable PWM fault interrupt
 */
void mcHalI_PwmInterruptEnable( void )
{
    NVIC_EnableIRQ(PWM0_IRQn);
}

/**
 * @brief Start PWM timer
 *
 * @details
 * Start PWM timer
 */
void mcHalI_PwmTimerStart( void )
{
    PWM0_ChannelsStart(PWM_CHANNEL_0_MASK);
    PWM0_ChannelsStart(PWM_CHANNEL_1_MASK);
    PWM0_ChannelsStart(PWM_CHANNEL_2_MASK);
}

/**
 * @brief ADC conversion complete interrupt callback function
 *
 * @details
 * ADC conversion complete interrupt callback function
 */
void mcHalI_AdcCallBackRegister( AFEC_CALLBACK callback, uintptr_t context )
{
    AFEC0_CallbackRegister( callback, context);
}

/**
 * @brief PWM fault interrupt callback function
 *
 * @details
 * PWM fault interrupt callback function
 */
void mcHalI_PwmCallbackRegister( PWM_CALLBACK callback, uintptr_t context )
{
    PWM0_CallbackRegister( callback, (uintptr_t)context);
}




/**
 * @brief Get start-stop button state
 *
 * @details
 * Get start-stop button state
 *
 * @return  True if button is pressed, false if button is not pressed
 */
bool mcHalI_StartStopButtonState( void )
{
    return  PIO_PinRead( PIO_PIN_PC1);
}

/**
 * @brief Get direction button state
 *
 * @details
 * Get direction button state
 *
 * @return  True if button is pressed, false if button is not pressed
 */
bool mcHalI_DirectionButtonState( void )
{
    return  PIO_PinRead( PIO_PIN_PC3);
}


/*******************************************************************************
 Application

  Company:
    Microchip Technology Inc.

  File Name:
    mc_tasks.h

  Summary:
 Application

  Description:
   Application 
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

#ifndef MCAPP_H    // Guards against multiple inclusion
#define MCAPP_H

/*******************************************************************************
 Header files inclusions
 *******************************************************************************/
#include "mc_types.h"
#include "definitions.h"
#include "mc_hardware_abstraction.h" 
#include "mc_current_calculation.h"
#include "mc_voltage_measurement.h"
#include "mc_motor.h"
#include "mc_field_oriented_control.h"

/*******************************************************************************
 Private data-types
 *******************************************************************************/

/*******************************************************************************
 * Interface variables 
 *******************************************************************************/


/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/*! \brief Application initialization 
 * 
 * Details.
 * Application initialization 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_ApplicationInit( void );

/*! \brief Over current reaction ISR
 *
 * Details.
 * Fault reaction ISR
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */

void mcAppI_OverCurrentReactionIsr( uint32_t status,  uintptr_t context );


/*! \brief Motor control application calibration
 * 
 * Details.
 *  Motor Control application calibration 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_AdcCalibrationIsr( uint32_t status, uintptr_t context );


/*! \brief Interrupt tasks execution
 *
 * Details.
 *  Interrupt tasks execution
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_AdcFinishedIsr( uint32_t status, uintptr_t context );

/*! \brief Non-ISR tasks
 *
 * Details.
 * Non-ISR tasks
 *
 * @param[in]:
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_NonISRTasks( void );

/*! \brief Application reset 
 * 
 * Details.
 * Application reset 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_ApplicationReset( void );

#endif //MCAPP_H

/**
 End of File
*/

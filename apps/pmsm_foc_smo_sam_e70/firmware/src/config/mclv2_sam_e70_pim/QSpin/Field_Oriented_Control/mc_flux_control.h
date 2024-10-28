/*******************************************************************************
  System Definitions

  File Name:
    mc_flux_control.h

  Summary:
    Header file which contains variables and function prototypes for flux control

  Description:
    This file contains variables and function prototypes which are generally used for flux
    control

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
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

#ifndef MCFLX_H
#define MCFLX_H

/*******************************************************************************
 * Header inclusions
*******************************************************************************/
#include "mc_types.h"
#include "mc_pi_control.h"
#include "mc_motor.h"

/*******************************************************************************
 Default Module configuration parameters
*******************************************************************************/

/*******************************************************************************
Type Definition
*******************************************************************************/
typedef struct
{
    float32_t Kp;     /**< Proportional gain for PI controller */
    float32_t Ki;     /**< Integral gain for PI controller */
    float32_t dt;      /**< Sampling time */

    void * pStatePointer;     /**< Pointer to the state structure */
}tmcFlx_Parameters_s;

/*******************************************************************************
 * Interface variables
*******************************************************************************/

/*******************************************************************************
 Static Interface Functions
*******************************************************************************/
/*! 
 * @brief Set module parameters
 *
 * This function sets the parameters for the flux control module.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @return None
 */
__STATIC_INLINE void mcFlxI_ParametersSet( tmcFlx_Parameters_s * const pParameters )
{
    pParameters->Kp = (float32_t)0.016776;
    pParameters->Ki =  (float32_t)17.366188;
    pParameters->dt = (float32_t)(0.00005);


}

/*******************************************************************************
 Interface Functions
*******************************************************************************/
/*! 
 * @brief Initialize flux control module
 *
 * Initializes the flux control module.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @return None
 */
void  mcFlxI_FluxControlInit( tmcFlx_Parameters_s * const pParameters );

/*! 
 * @brief Enable flux control module
 *
 * Enables the flux control module.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @return None
 */
void  mcFlxI_FluxControlEnable( tmcFlx_Parameters_s * const pParameters );

/*! 
 * @brief Disable flux control module
 *
 * Disables the flux control module.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @return None
 */
void  mcFlxI_FluxControlDisable( tmcFlx_Parameters_s * const pParameters );

/*! 
 * @brief Manual flux control
 *
 * Performs manual flux control.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @param[in] Out Output value for manual control
 * @return None
 */
void mcFlxI_FluxControlManual(  const tmcFlx_Parameters_s * const pParameters,
                                                         const float32_t  Out );

/*! 
 * @brief Automatic flux control
 *
 * Performs automatic flux control.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @param[in] iDref Reference d-axis current
 * @param[in] iDact Actual d-axis current
 * @param[out] pOut Pointer to output variable for automatic control
 * @return None
 */
void mcFlxI_FluxControlAuto(  const tmcFlx_Parameters_s * const pParameters,
                                              const float32_t iDref, const  float32_t iDact, const float32_t iDmax, float32_t * const pOut );

/*! 
 * @brief Reset flux control
 *
 * Resets the flux control module.
 *
 * @param[in] pParameters Pointer to module parameters structure
 * @return None
 */
void mcFlxI_FluxControlReset( const tmcFlx_Parameters_s * const pParameters );

#endif // MCFLX_H

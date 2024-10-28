/**
 * @brief
 *  Header file which contains variables and function prototypes of PI controller functions.
 *
 * @Company
 *  Microchip Technology Inc.
 *
 * @File name
 *  mc_pi_control.h
 *
 * @Summary
 *  Header file which contains variables and function prototypes for PI controller functions
 *
 * @Description
 *  This file contains variables and function prototypes for PI controller functions  which
 *  are generally used in Motor Control. Implemented in Q2.14 Fixed Point Arithmetic.
 *
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

#ifndef MC_PI
#define MC_PI

#include "mc_types.h"
#include "mc_utilities.h"

/******************************************************************************
 * Constants
******************************************************************************/

/******************************************************************************
 * User-defined data structure
******************************************************************************/
/**
 * @brief PI control structure.
 *
 * This structure holds the necessary parameters and state variables for the PI (Proportional-Integral) controller.
 */
typedef struct
{
    float32_t error;
    float32_t Kp;
    float32_t Ki;
    float32_t Yint;
    float32_t Ymin;
    float32_t Ymax;
    float32_t Yo;
}tmcUtils_PiControl_s;

/******************************************************************************
 * Interface variables
******************************************************************************/

/******************************************************************************
 * Interface functions
******************************************************************************/
/**
 * @brief Initializes the PI control structure with the given parameters.
 *
 * This function sets up the PI control structure with proportional and integral
 * gains, and computes the shifts required for fixed-point arithmetic.
 *
 * @param[in] Kp Proportional gain.
 * @param[in] Ki Integral gain.
 * @param[in] dt Sampling time.
 * @param[in,out] pControl Pointer to the PI control structure to be initialized.
 *
 * @return None.
 */
void mcUtils_PiControlInit( float32_t Kp, float32_t Ki, float32_t dt,  tmcUtils_PiControl_s  * const pControl);

/**
 * @brief Updates the PI control limits.
 *
 * This function updates the minimum and maximum output limits for the PI control.
 *
 * @param[in] Ymin Minimum output limit.
 * @param[in] Ymax Maximum output limit.
 * @param[in,out] pControl Pointer to the PI control structure to be updated.
 *
 * @return None.
 */
void mcUtils_PiLimitUpdate( const float32_t Ymin, const float32_t Ymax, tmcUtils_PiControl_s  * const pControl );

/**
 * @brief Updates the integral part of the PI control.
 *
 * This function updates the integral term of the PI control with the given value.
 *
 * @param[in] value Integral value to be updated.
 * @param[in,out] pControl Pointer to the PI control structure to be updated.
 *
 * @return None.
 */
void mcUtils_PiIntegralUpdate( const float32_t value, tmcUtils_PiControl_s  * const pControl );

/**
 * @brief Executes the PI control algorithm.
 *
 * This function computes the PI control output based on the given error and
 * updates the control structure accordingly.
 *
 * @param[in] error The current error value.
 * @param[in,out] pControl Pointer to the PI control structure to be updated.
 *
 * @return None.
 */
void mcUtils_PiControl(const float32_t error, tmcUtils_PiControl_s  * const pControl  );

/**
 * @brief Resets the PI control output.
 *
 * This function resets the PI control output to the given value.
 *
 * @param[in] out The output value to reset.
 * @param[in,out] pControl Pointer to the PI control structure to be reset.
 *
 * @return None.
 */
void mcUtils_PiControlReset(const float32_t integral, tmcUtils_PiControl_s  * const pControl);

#endif // MC_PI
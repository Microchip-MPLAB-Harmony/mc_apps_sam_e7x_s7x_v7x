/**
 * @brief 
 *    Field Oriented Control (FOC) header file
 *
 * @File Name 
 *    mc_field_oriented_control.h
 *
 * @Company 
 *   Microchip Technology Inc.
 *
 * @Summary
 *    This file contains declarations for Field Oriented Control (FOC) functions.
 *
 * @Description
 *   This file provides the function prototypes and data structures necessary for Field Oriented
 *    Control (FOC), which is used to control motor currents based on given inputs.
 *    Functions include initialization, execution, resetting, current updating,
 *    torque calculation, and field weakening control.
 */

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

#ifndef MCFOC_H
#define MCFOC_H

/*******************************************************************************
 * Header inclusions
*******************************************************************************/
#include "mc_types.h"
#include "mc_pwm.h"
#include "mc_open_loop_startup.h"
#include "mc_flux_control.h"
#include "mc_torque_control.h"
#include "mc_speed_control.h"
#include "mc_reference_control.h"
#include "mc_rotor_position_calculation.h"
#include "mc_utilities.h"
#include "mc_hardware_abstraction.h"
#include "mc_current_calculation.h"
#include "mc_voltage_measurement.h"

/*******************************************************************************
 Default Module configuration parameters
*******************************************************************************/

/*******************************************************************************
Type Definition
*******************************************************************************/
typedef struct
{
    tmcTypes_ABC_s iABC;    /**< Motor phase currents */
    float32_t elecAngle;    /**< Electrical angle in rads */
    float32_t elecSpeed;   /**< Electrical speed in rads */
    float32_t uBus;           /**< Bus voltage */
    float32_t reference;   /**< Reference value for control (e.g., speed or torque) */
}tmcFoc_Input_s;

typedef struct
{
    tmcTypes_AlphaBeta_s  iAlphaBeta;     /**< Alpha-beta components of motor currents */
    tmcTypes_AlphaBeta_s  uAlphaBeta;    /**< Alpha-beta components of voltages */
    tmcTypes_DQ_s  iDQ;                         /**< DQ components of motor currents */
    int16_t duty[3u];                                  /**< PWM duty cycles for three phases */
}tmcFoc_Output_s;

/**
 * @brief Data structure for FOC parameters
 */
typedef struct
{
   tmcMot_PMSM_s * pMotorParameters;    /**< Pointer to motor parameters */
   float32_t potInputToRef;                       /**< Potentiometer to reference value factor  */
}tmcFoc_Parameters_s;

/**
 * @brief Data structure for FOC module data
 */
typedef struct
{
    tmcFoc_Input_s dInput;             /**< Input ports */
    tmcFoc_Output_s dOutput;           /**< Output ports */
    tmcFoc_Parameters_s dParameters;   /**< User parameters */
    void * pStatePointer;              /**< Pointer to state data */
} tmcFocI_ModuleData_s;

/*******************************************************************************
 * Interface variables
*******************************************************************************/
extern tmcFocI_ModuleData_s mcFocI_ModuleData_gds;


/*******************************************************************************
 Static Interface Functions
*******************************************************************************/
/**
 * @brief Read input ports
 *
 * @details Reads the input ports for the FOC module.
 *
 * @param[in] None
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
__STATIC_INLINE void mcFocI_InputsRead( tmcFocI_ModuleData_s * const pModule )
{
    tmcFoc_Input_s * pInput = &pModule->dInput;

    pInput->iABC.a = (float32_t)mcCurI_ModuleData_gds.dOutput.iABC.a;
    pInput->iABC.b = (float32_t)mcCurI_ModuleData_gds.dOutput.iABC.b;

    pInput->elecAngle = (float32_t)mcRpcI_ModuleData_gds.dOutput.elecAngle;
    pInput->elecSpeed = (float32_t)mcRpcI_ModuleData_gds.dOutput.elecSpeed;


    pInput->uBus = (float32_t)mcVolI_ModuleData_gds.dOutput.uBus;

    pInput->reference = ((float32_t)mcHalI_Potentiometer_gdu16 ) * pModule->dParameters.potInputToRef;

}


/**
 * @brief Set module parameters
 *
 * @details Sets the parameters for the FOC module.
 *
 * @param[in] pParameters Pointer to the FOC parameters
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
__STATIC_INLINE void mcFocI_ParametersSet( tmcFoc_Parameters_s * const pParameters )
{
   pParameters->pMotorParameters = &mcMotI_PMSM_gds;

   pParameters->potInputToRef = mcMotI_PMSM_gds.NratedInRpm  / 4095.0f;
}

/**
 * @brief Write output ports
 *
 * @details Writes the output ports for the FOC module.
 *
 * @param[in] pOutput Pointer to the FOC output structure
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
__STATIC_INLINE void mcFocI_OutputPortWrite( tmcFoc_Output_s * const pOutput )
{
    mcPwmI_Duty_gau16[0u] = pOutput->duty[0u];
    mcPwmI_Duty_gau16[1u] = pOutput->duty[1u];
    mcPwmI_Duty_gau16[2u] = pOutput->duty[2u];
}

/*******************************************************************************
 Interface Functions
*******************************************************************************/
/**
 * @brief Initialize Field Oriented Control (FOC) module
 *
 * @details Initializes the FOC module.
 *
 * @param[in] pModule Pointer to the FOC module data
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void  mcFocI_FieldOrientedControlInit( tmcFocI_ModuleData_s * const pModule );

/**
 * @brief Enable Field Oriented Control (FOC) module
 *
 * @details Enables the FOC module.
 *
 * @param[in] pParameters Pointer to the FOC parameters
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void  mcFocI_FieldOrientedControlEnable( tmcFocI_ModuleData_s * const pParameters );

/**
 * @brief Disable Field Oriented Control (FOC) module
 *
 * @details Disables the FOC module.
 *
 * @param[in] pParameters Pointer to the FOC parameters
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void  mcFocI_FieldOrientedControlDisable( tmcFocI_ModuleData_s * const pParameters );

/**
 * @brief Execute Field Oriented Control (FOC) fast loop
 *
 * @details Executes the fast loop of the FOC algorithm.
 *
 * @param[in] pModule Pointer to the FOC module data
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void mcFocI_FieldOrientedControlFast(  tmcFocI_ModuleData_s * const pModule );

 /**
 * @brief Execute Field Oriented Control (FOC) slow loop
 *
 * @details Executes the slow loop of the FOC algorithm.
 *
 * @param[in] pParameters Pointer to the FOC parameters
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void mcFocI_FieldOrientedControlSlow( const tmcFocI_ModuleData_s * const pParameters );

/**
 * @brief Change motor direction
 *
 * @details Changes the direction of the motor.
 *
 * @param[in] pModule Pointer to the FOC module data
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void mcFocI_MotorDirectionChange(const tmcFocI_ModuleData_s * const pParameters);

/**
 * @brief Reset Field Oriented Control (FOC)
 *
 * @details Resets the FOC module.
 *
 * @param[in] pParameters Pointer to the FOC parameters
 * @param[in,out] None
 * @param[out] None
 * @return None
 */
void mcFocI_FieldOrientedControlReset( const tmcFocI_ModuleData_s * const pParameters );

#endif // MCFOC_H

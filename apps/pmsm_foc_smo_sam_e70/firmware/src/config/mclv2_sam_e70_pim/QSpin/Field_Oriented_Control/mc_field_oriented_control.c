/**
 * @brief 
 *    Field Oriented Control (FOC) source file
 *
 * @File Name 
 *    mc_field_oriented_control.c
 *
 * @Company 
 *   Microchip Technology Inc.
 *
 * @Summary
 *    This file implements functions for Field Oriented Control (FOC).
 *
 * @Description
 *   This file contains the implementation of functions necessary for Field Oriented
 *    Control (FOC), which is used to control motor currents based on given inputs.
 *    Functions include initialization, execution, resetting, current updating,
 *    torque calculation, and field weakening control.
 */

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

/*******************************************************************************
Headers inclusions
*******************************************************************************/
#include "mc_field_oriented_control.h"

/*******************************************************************************
Local configuration options
*******************************************************************************/

/*******************************************************************************
 Private data types
*******************************************************************************/
typedef enum
{
    FocState_Startup,
    FocState_ClosingLoop,
    FocState_CloseLoop
}tmcFoc_FocState_e;

typedef struct
 {
    bool enable;
    bool initDone;
    tmcFoc_FocState_e FocState;
    tmcTypes_DQ_s uDQ;
    float32_t openLoopAngle;
    float32_t openLoopSpeed;
    float32_t iQref;
    float32_t iDref;
    float32_t nRef;
    float32_t angleDifference;
    float32_t commandDirection;
    float32_t ratedSpeedInRpm;
    tmcSup_Parameters_s bOpenLoopStartup;
    tmcPwm_Parameters_s bPwmModulator;
    tmcFlx_Parameters_s bFluxController;
    tmcTor_Parameters_s bTorqueController;
    tmcSpe_Parameters_s bSpeedController;
    tmcRef_Parameters_s bReferenceController;
    tmcRpe_Parameters_s bPositionEstimation;

    uint16_t duty[3u];
}tmcFoc_State_s;

/*******************************************************************************
Private variables
*******************************************************************************/
static tmcFoc_State_s mcFoc_State_mds;

/*******************************************************************************
Interface  variables
*******************************************************************************/

tmcFocI_ModuleData_s mcFocI_ModuleData_gds;

/*******************************************************************************
Macro Functions
*******************************************************************************/
/**
 * Constant value of 2/PI
 */
#define TWO_BY_PI (float32_t)(0.6366198)

/**
 *  Open loop angle to close loop angle transition rate.
 */
#define ROTOR_ANGLE_RAMP_RATE     (float32_t)( 1.0e-5 )

/*******************************************************************************
Private Functions
*******************************************************************************/
/**
 * @brief Clarke Transformation
 *
 * @details
 * This function performs the Clarke transformation to convert three-phase
 * currents into two-phase alpha-beta currents.
 *
 * @param[in] pABC Pointer to the structure holding three-phase currents
 * @param[out] pAlphaBeta Pointer to the structure holding alpha-beta currents
 */
__STATIC_FORCEINLINE void mcFoc_ClarkeTransformation( const tmcTypes_ABC_s * pABC,
                                                                                       tmcTypes_AlphaBeta_s * const pAlphaBeta )
{
    pAlphaBeta->alpha = pABC->a;
    pAlphaBeta->beta = (pABC->a * ONE_BY_SQRT3 ) + ( pABC->b * TWO_BY_SQRT3 );
}

/**
 * @brief Park Transformation
 *
 * @details
 * This function performs the Park transformation to convert alpha-beta currents
 * into DQ currents using the provided sine and cosine values.
 *
 * @param[in] pAlphaBeta Pointer to the structure holding alpha-beta currents
 * @param[in] sine Sine value of the electrical angle
 * @param[in] cosine Cosine value of the electrical angle
 * @param[out] pDQ Pointer to the structure holding DQ currents
 */
__STATIC_FORCEINLINE void mcFoc_ParkTransformation( const tmcTypes_AlphaBeta_s * const pAlphaBeta,
                                                                                    const float32_t sine, const float32_t cosine,
                                                                                    tmcTypes_DQ_s * const pDQ )
{
    pDQ->d =  pAlphaBeta->alpha * cosine  + pAlphaBeta->beta * sine;
    pDQ->q = -pAlphaBeta->alpha * sine  + pAlphaBeta->beta * cosine;
}

/**
 * @brief Inverse Park Transformation
 *
 * @details
 * This function performs the inverse Park transformation to convert DQ currents
 * back into alpha-beta currents using the provided sine and cosine values.
 *
 * @param[in] pDQ Pointer to the structure holding DQ currents
 * @param[in] sine Sine value of the electrical angle
 * @param[in] cosine Cosine value of the electrical angle
 * @param[out] pAlphaBeta Pointer to the structure holding alpha-beta currents
 */
__STATIC_FORCEINLINE void mcFoc_InverseParkTransformation( const tmcTypes_DQ_s * const pDQ,
                                                                                                const float32_t sine, const float32_t cosine,
                                                                                                tmcTypes_AlphaBeta_s * const pAlphaBeta )
{
    pAlphaBeta->alpha =  pDQ->d * cosine - pDQ->q * sine;
    pAlphaBeta->beta  =  pDQ->d * sine  + pDQ->q * cosine;
}

/*******************************************************************************
 * Interface Functions
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
void  mcFocI_FieldOrientedControlInit( tmcFocI_ModuleData_s * const pModule )
{
    /** Link state variable structure to the module */
    pModule->pStatePointer = (void *)&mcFoc_State_mds;

    /** Set parameters */
    mcFocI_ParametersSet(&pModule->dParameters);

    /** ToDO: Set default direction */
    mcFoc_State_mds.commandDirection = 1.0f;

    /** Update state variables */
    mcFoc_State_mds.ratedSpeedInRpm = pModule->dParameters.pMotorParameters->NratedInRpm;
    /** Initialize open loop start-up module */
    mcSupI_OpenLoopStartupInit( &mcFoc_State_mds.bOpenLoopStartup );

    /** Initialize reference control module */
    mcRefI_ReferenceControlInit( &mcFoc_State_mds.bReferenceController);

    /** Initialize speed control module */
    mcSpeI_SpeedControlInit( &mcFoc_State_mds.bSpeedController);

    /** Initialize torque control module */
    mcTorI_TorqueControlInit( &mcFoc_State_mds.bTorqueController);

    /** Initialize flux control module */
    mcFlxI_FluxControlInit( &mcFoc_State_mds.bFluxController);

    /** Initialize rotor position estimation  */
    mcRpeI_RotorPositionEstimInit( &mcFoc_State_mds.bPositionEstimation);

    /** Initialize PWM  module */
    mcPwmI_PulseWidthModulationInit( &mcFoc_State_mds.bPwmModulator );

    /** Set initialization flag */
    mcFoc_State_mds.initDone = true;

}

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
void  mcFocI_FieldOrientedControlEnable( tmcFocI_ModuleData_s * const pParameters )
{
    /** Get the linked state variable */
    tmcFoc_State_s * pState;
    pState = (tmcFoc_State_s *)pParameters->pStatePointer;

    if( ( NULL == pState ) || ( !pState->initDone ))
    {
         /** Initialize parameters */
        mcFocI_FieldOrientedControlInit(pParameters);
    }
    else
    {
         /** For MISRA Compliance */
    }

    /** Enable open loop start-up module */
    mcSupI_OpenLoopStartupEnable( &mcFoc_State_mds.bOpenLoopStartup );

    /** Enable reference control module */
    mcRefI_ReferenceControlEnable( &mcFoc_State_mds.bReferenceController);

    /** Enable speed control module */
    mcSpeI_SpeedControlEnable( &mcFoc_State_mds.bSpeedController);


    /** Enable torque control module */
    mcTorI_TorqueControlEnable( &mcFoc_State_mds.bTorqueController);

    /** Enable flux control module */
    mcFlxI_FluxControlEnable( &mcFoc_State_mds.bFluxController);

    /** Enable rotor position estimation  */
    mcRpeI_RotorPositionEstimEnable( &mcFoc_State_mds.bPositionEstimation);

    /** Enable PWM  module */
    mcPwmI_PulseWidthModulationEnable( &mcFoc_State_mds.bPwmModulator );

    /** Set FOC state machine */
    mcFoc_State_mds.FocState = FocState_Startup;

    /** Set enable flag as true */
    pState->enable = true;
}

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
void  mcFocI_FieldOrientedControlDisable( tmcFocI_ModuleData_s * const pParameters )
{
    /** Get the linked state variable */
    tmcFoc_State_s * pState;
    pState = (tmcFoc_State_s *)pParameters->pStatePointer;

    if( NULL != pState)
    {
        /** Reset state variables  */
        mcFocI_FieldOrientedControlReset(pParameters);
    }
    else
    {
        /** For MISRA Compliance */
    }

    /** Disable open loop start-up module */
    mcSupI_OpenLoopStartupDisable( &mcFoc_State_mds.bOpenLoopStartup );

    /** Disable reference control module */
    mcRefI_ReferenceControlDisable( &mcFoc_State_mds.bReferenceController);

    /** Disable speed control module */
    mcSpeI_SpeedControlDisable( &mcFoc_State_mds.bSpeedController);

    /** Disable torque control module */
    mcTorI_TorqueControlDisable( &mcFoc_State_mds.bTorqueController);

    /** Disable flux control module */
    mcFlxI_FluxControlDisable( &mcFoc_State_mds.bFluxController);

    /** Disable rotor position estimation  */
    mcRpeI_RotorPositionEstimDisable( &mcFoc_State_mds.bPositionEstimation);

    /** Disable PWM  module */
    mcPwmI_PulseWidthModulationDisable( &mcFoc_State_mds.bPwmModulator );

    /** Set enable flag as true */
    pState->enable = false;

}

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
void mcFocI_FieldOrientedControlFast( tmcFocI_ModuleData_s * const pModule )
{
    /** Intermediate variables */
    float32_t sine = 0.0f;
    float32_t cosine = 0.0f;

    /** Get the linked state variable */
    tmcFoc_State_s * pState;
    pState = (tmcFoc_State_s *)pModule->pStatePointer;

    /** Get the output structure pointer */
    tmcFoc_Input_s * pInput;
    pInput = &pModule->dInput;

    /** Get the output structure pointer */
    tmcFoc_Output_s * pOutput;
    pOutput = &pModule->dOutput;

    /** Read FOC inputs  */
    mcFocI_InputsRead( pModule );

    /** Clarke transformation */
    mcFoc_ClarkeTransformation( &pInput->iABC, &pOutput->iAlphaBeta);

    /** Rotor position estimation */
    tmcTypes_AlphaBeta_s eAlphaBeta;
    mcRpeI_RotorPositionEstim( &pState->bPositionEstimation, &pOutput->iAlphaBeta, &pOutput->uAlphaBeta,
                               &eAlphaBeta, &pOutput->elecAngle, &pOutput->elecSpeed );

    switch(pState->FocState )
    {

        case FocState_Startup:
        {
            tmcTypes_StdReturn_e startupStatus;
            startupStatus = mcSupI_OpenLoopStartup( &pState->bOpenLoopStartup, pState->commandDirection, &pState->iQref,
                                                    &pState->iDref, &pState->openLoopAngle, &pState->openLoopSpeed );

            pState->nRef = pState->openLoopSpeed;

            if( StdReturn_Complete == startupStatus )
            {
                /** Set speed controller state */
                mcSpeI_SpeedControlManual( &pState->bSpeedController, pState->iQref );

                /** Calculate angle difference */
                pState->angleDifference = UTIL_AngleDifferenceCalc( pState->openLoopAngle, pOutput->elecAngle );

                /** Set FOC state machine to ClosingLoop */
                pState->FocState = FocState_ClosingLoop;
            }

            /** Sine-cosine calculation */
            mcUtils_SineCosineCalculation( pState->openLoopAngle, &sine, &cosine );

            break;
        }
            case FocState_ClosingLoop:
            {
                float32_t angle = pOutput->elecAngle + pState->angleDifference;
                mcUtils_TruncateAngle0To2Pi(&angle);

                /** Ramp-down angle difference */
                UTIL_LinearRampFloat(&pState->angleDifference, ROTOR_ANGLE_RAMP_RATE, 0.0f );

                if( UTIL_IS_ZERO( pState->angleDifference ))
                {
                    pState->angleDifference = 0.0f;
                    pState->FocState = FocState_CloseLoop;
                }

                /** Sine-cosine calculation */
                mcUtils_SineCosineCalculation( angle, &sine, &cosine );

                /** Execute speed controller */
                mcSpeI_SpeedControlAuto( &pState->bSpeedController, pState->nRef, pOutput->elecSpeed,  &pState->iQref );
                break;
            }

            case FocState_CloseLoop:
            {

                /** Sine-cosine calculation */
                mcUtils_SineCosineCalculation( pOutput->elecAngle, &sine, &cosine );
                /** Reference Control */
                mcRefI_ReferenceControl( &mcFoc_State_mds.bReferenceController, pModule->dInput.reference, &pState->nRef );

                /** Execute speed controller */
                pState->nRef *=  pState->commandDirection;
                mcSpeI_SpeedControlAuto(&pState->bSpeedController,  pState->nRef, pOutput->elecSpeed,
                                        &pState->iQref );

                break;
            }

            default:
            {
                /** For MISRA Compliance */
            }
            break;
    }

    /** Park Transformation */
    mcFoc_ParkTransformation( &pOutput->iAlphaBeta, sine, cosine, &pOutput->iDQ );

    /** Compute Q-axis controller output limit */
    float32_t ydLimit = pModule->dInput.uBus * TWO_BY_PI;

    /** Execute flux control */
    mcFlxI_FluxControlAuto( &pState->bFluxController, pState->iDref, pOutput->iDQ.d, ydLimit, &pState->uDQ.d );

    /** Apply circle limit for Q-axis reference current clamping  */
    float32_t yqLimit = UTIL_SquareRootFloat( UTIL_SquareFloat( ydLimit )  - UTIL_SquareFloat( pState->uDQ.d ));


    /** Execute torque control */
    mcTorI_TorqueControlAuto( &pState->bTorqueController, pState->iQref, pOutput->iDQ.q, yqLimit, &pState->uDQ.q );

    /** Inverse Park transformation */
    mcFoc_InverseParkTransformation( &pState->uDQ, sine, cosine, &pOutput->uAlphaBeta );

     /** Space vector modulation */
    mcPwmI_PulseWidthModulation(&pState->bPwmModulator, pModule->dInput.uBus, &pOutput->uAlphaBeta, mcPwmI_Duty_gau16 );

}


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
void mcFocI_FieldOrientedControlSlow( const tmcFocI_ModuleData_s * const pParameters )
{
    /** ToDO: Put appropriate tasks */
}

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
void mcFocI_MotorDirectionChange(const tmcFocI_ModuleData_s * const pParameters)
{
    /** Get the linked state variable */
    tmcFoc_State_s * pState;
    pState = (tmcFoc_State_s *)pParameters->pStatePointer;

    pState->commandDirection = - pState->commandDirection;

}

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
void mcFocI_FieldOrientedControlReset( const tmcFocI_ModuleData_s * const pParameters )
{
    /** Reset reference control module */
    mcRefI_ReferenceControlReset( &mcFoc_State_mds.bReferenceController);

    /** Reset speed control module */
    mcSpeI_SpeedControlReset( &mcFoc_State_mds.bSpeedController);

    /** Reset torque control module */
    mcTorI_TorqueControlReset( &mcFoc_State_mds.bTorqueController);

    /** Reset flux control module */
    mcFlxI_FluxControlReset( &mcFoc_State_mds.bFluxController);

    /** Reset rotor position estimation  */
    mcRpeI_RotorPositionEstimReset( &mcFoc_State_mds.bPositionEstimation);

    /** Reset PWM  module */
    mcPwmI_PulseWidthModulationReset( &mcFoc_State_mds.bPwmModulator );
}

/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    mc_app.c

  Summary:
    This file contains the state machine and control loop of the motor control
   algorithm.

  Description:
    This file contains the control loop of the pmsm_foc_pll_estimator.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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
#include "mc_app.h"
#include "userparams.h"
#include "X2CScope.h"
#include "X2CScopeCommunication.h"
#include "math.h"


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

/* Motor phase current offset calibration limits. */
#define CURRENT_OFFSET_MAX                (2200) /* current offset max limit in terms of ADC count*/
#define CURRENT_OFFSET_MIN                (1900) /* current offset min limit in terms of ADC count*/
/** Phase Current Offset calibration samples */
#define CURRENTS_OFFSET_SAMPLES                             (100U)

/******************************************************************************/
/* Local Function Prototype                                                   */
/******************************************************************************/
__STATIC_INLINE void MCAPP_MotorAngleCalc(void);
__STATIC_INLINE void MCAPP_MotorCurrentControl( void );
static void MCAPP_ADCOffsetCalibration(void);
static void MCAPP_MotorControlParamInit(void);
__STATIC_INLINE bool MCAPP_SlowLoopTimeIsFinished(void);
__STATIC_INLINE void MCAPP_SlowControlLoop(void);
static void MCAPP_SwitchDebounce(MC_APP_STATE state);
static uintptr_t dummyforMisra;
#if(TORQUE_MODE == 0)
__STATIC_INLINE void MCAPP_SpeedRamp(void);
#endif

/******************************************************************************/
/*                   Global Variables                                         */
/******************************************************************************/
static MC_APP_DATA gMCAPPData;
static MCAPP_CONTROL_PARAM gCtrlParam;
static MCAPP_FOC_PARAM gfocParam;
static float phaseCurrentUOffset;
static float phaseCurrentVOffset;
static bool adc_calib_done = false;

static float speed_ref_filtered = 0.0f;


/******************************************************************************/
/* Function name: MCAPP_MotorCurrentControl                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Executes one PI iteration for each of the three loops Id,Iq                */
/******************************************************************************/
__STATIC_INLINE void MCAPP_MotorCurrentControl( void )
{
    if( gCtrlParam.openLoop == true )
    {
        /* OPENLOOP:  force rotating angle,Vd,Vq */
        if( gCtrlParam.changeMode == true )
        {
            /* just changed to open loop */
            gCtrlParam.changeMode = false;

            /* IqRef & idRef not used */
            gCtrlParam.iqRef = 0.0f;
            gCtrlParam.idRef = 0.0f;

			/* re-init vars for initial speed ramp */
			gCtrlParam.startup_lock_count = 0;
			gCtrlParam.startup_angle_ramp_rads_per_sec = 0.0f;
        }

        /* q current reference is equal to the velocity reference
         * while d current reference is equal to 0
         * for maximum startup torque, set the q current to maximum acceptable
         * value represents the maximum peak value 	 */
        gCtrlParam.iqRef = Q_CURRENT_REF_OPENLOOP;

        /* PI control for Iq torque control loop */
        gPIParmQ.inMeas = gMCLIBCurrentDQ.iq;
        gPIParmQ.inRef  = gCtrlParam.iqRef;
        MCLIB_PIControl(&gPIParmQ);
        gMCLIBVoltageDQ.vq = gPIParmQ.out;

        /* PI control for Id flux control loop */
        gPIParmD.inMeas = gMCLIBCurrentDQ.id;
        gPIParmD.inRef  = gCtrlParam.idRef;
        MCLIB_PIControl(&gPIParmD);
        gMCLIBVoltageDQ.vd = gPIParmD.out;
    }
    else
	{
		/* Closed Loop Vector Control */
        if( gCtrlParam.changeMode == true )
        {
            /* switched from open loop to close loop */
            gCtrlParam.changeMode = false;
			/* Load velocity control loop with Iq reference for smooth transition */
			gPIParmQref.dSum = gCtrlParam.iqRef;
            gCtrlParam.velRef = OPNLP_END_SPEED_RDPS_ELEC;
            gPIParmD.inRef = 0.0f;
            gCtrlParam.idRef = 0.0f;
			gCtrlParam.sync_cnt = 0;
	    }


#if(TORQUE_MODE == 1)
		/* If TORQUE MODE is enabled then skip the velocity control loop */
        gCtrlParam.iqRef = Q_CURRENT_REF_OPENLOOP;
#endif

        /* PI control for Id flux control loop */
        gPIParmD.inMeas = gMCLIBCurrentDQ.id;          /* This is in Amps */
        gPIParmD.inRef  = gCtrlParam.idRef;       /* This is in Amps */
		MCLIB_PIControl(&gPIParmD);

		gfocParam.lastVd = gMCLIBVoltageDQ.vd;
        gMCLIBVoltageDQ.vd    = gPIParmD.out;          /* This is in %. It should be converted to volts, multiply with (DC/sqrt(3)) */

        /* PI control for Iq torque control */
        gPIParmQ.inMeas = gMCLIBCurrentDQ.iq;          /* This is in Amps */
        gPIParmQ.inRef  = gCtrlParam.iqRef;       /* This is in Amps */
        MCLIB_PIControl(&gPIParmQ);
        gMCLIBVoltageDQ.vq    = gPIParmQ.out;          /* This is in %. If should be converted to volts, multiply with (VDC/sqrt(3))  */
	}
}


/******************************************************************************/
/* Function name: MCAPP_MotorAngleCalc                                          */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Generate the start sin waves feeding the motor's terminals    */
/* in open loop control, forcing the motor to align and to start speeding up  */
/******************************************************************************/
__STATIC_INLINE void MCAPP_MotorAngleCalc(void)
{
	if(gCtrlParam.openLoop == true)
	{
		/* begin with the lock sequence, for field alignment */
		if (gCtrlParam.startup_lock_count < LOCK_COUNT_FOR_LOCK_TIME)
		{
			gCtrlParam.startup_lock_count++;
            gfocParam.angle = (3.0f*(float)M_PI_2); // Since during lock time, the current
                                          //is injected on Q axis i.e. leads
                                          //gfocParam.Angle by PI/2, setting angle to 3PI/2 would cause the rotor to align at 0 degrees.

        }

	    /* then ramp up till the open loop end speed */
		else if (gCtrlParam.startup_angle_ramp_rads_per_sec < OPNLP_END_SPEED_RDPS_ELEC_IN_LOOPTIME)
		{
            gCtrlParam.startup_angle_ramp_rads_per_sec += OPEN_LOOP_RAMPSPEED_INCREASERATE;
            gCtrlParam.velRef = gCtrlParam.startup_angle_ramp_rads_per_sec / FAST_LOOP_TIME_SEC;

		}
		else
		{
			if(gCtrlParam.open_loop_stab_counter < 0xFFFU)
			{
				gCtrlParam.open_loop_stab_counter++;
			}
			else
			{
				/* switch to close loop */
				#if(OPEN_LOOP_FUNCTIONING == false)
				gCtrlParam.changeMode = true;
				gCtrlParam.openLoop = false;
				gCtrlParam.open_loop_stab_counter = 0;
				#endif
			}
		}

		/* the angle set depends on startup ramp */
		gfocParam.angle += gCtrlParam.startup_angle_ramp_rads_per_sec;

        if(gfocParam.angle >= SINGLE_ELEC_ROT_RADS_PER_SEC)
		{
            gfocParam.angle = gfocParam.angle - SINGLE_ELEC_ROT_RADS_PER_SEC;
		}
	}
	else
	{
		/* Switched to closed loop..
   	       In closed loop slowly decrease the offset which is present in
   	       the estimated angle during open loop */
   	    if(gMCLIBEstimParam.rhoOffset > ANGLE_OFFSET_MIN)
		{
           gMCLIBEstimParam.rhoOffset = gMCLIBEstimParam.rhoOffset - ANGLE_OFFSET_MIN;
		}
		else if(gMCLIBEstimParam.rhoOffset < ANGLE_OFFSET_MIN)
		{
			gMCLIBEstimParam.rhoOffset = gMCLIBEstimParam.rhoOffset + ANGLE_OFFSET_MIN;
		}
		else
		{
		   gMCLIBEstimParam.rhoOffset = 0.0f;
		}
	}

}

/******************************************************************************/
/* Function name: MCAPP_ADCOffsetCalibration                                     */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Measures motor phase current offsets.Limits the offset if it  */
/*              exceeds the predetermined range.                              */
/******************************************************************************/


static void MCAPP_ADCOffsetCalibration(void)
{
	uint32_t AdcSampleCounter = 0;
    int32_t delayCounter = 0xFFFF;
	uint32_t phaseUOffsetBuffer = 0;
    uint32_t phaseVOffsetBuffer = 0;

	/* Disable interrupt generation */
	AFEC0_ChannelsInterruptDisable(AFEC_INTERRUPT_EOC_7_MASK);

	/* Software trigger used for ADC conversion. */
	//afec_set_trigger(AFEC0, AFEC_TRIG_SW);
    AFEC0_REGS->AFEC_MR &= ~(AFEC_MR_TRGEN_Msk);

	for(AdcSampleCounter = 0; AdcSampleCounter < CURRENTS_OFFSET_SAMPLES; AdcSampleCounter++)
	{
		/* Software trigger for phase current measurements */
		AFEC0_ConversionStart();

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

		phaseUOffsetBuffer += AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)PH_U_CURRENT_ADC_CH);
		phaseVOffsetBuffer += AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)PH_V_CURRENT_ADC_CH);
	}

	phaseCurrentUOffset = (float)((float)phaseUOffsetBuffer/(float)CURRENTS_OFFSET_SAMPLES);
	phaseCurrentVOffset = (float)((float)phaseVOffsetBuffer/(float)CURRENTS_OFFSET_SAMPLES);

	/* Limit motor phase A current offset calibration to configured Min/Max levels. */
	if(phaseCurrentUOffset >  (float)CURRENT_OFFSET_MAX)
	{
		phaseCurrentUOffset = (float)CURRENT_OFFSET_MAX;
	}
	else if(phaseCurrentUOffset <  (float)CURRENT_OFFSET_MIN)
	{
		phaseCurrentUOffset = (float)CURRENT_OFFSET_MIN;
	}
    else
    {
        /* No operation dummy for MISRAC */
    }

	/* Limit motor phase B current offset calibration to configured Min/Max levels. */
	if(phaseCurrentVOffset >  (float)CURRENT_OFFSET_MAX)
	{
		phaseCurrentVOffset = (float)CURRENT_OFFSET_MAX;
	}
	else if(phaseCurrentVOffset <  (float)CURRENT_OFFSET_MIN)
	{
		phaseCurrentVOffset = (float)CURRENT_OFFSET_MIN;
	}
    else
    {
        /* No operation dummy for MISRAC */
    }
	/* Enable adc end of conversion interrupt generation to execute FOC loop */
	AFEC0_ChannelsInterruptEnable(AFEC_INTERRUPT_EOC_7_MASK);

    /* Set adc trigger from PWM event lines */
    AFEC0_REGS->AFEC_MR |= (AFEC_MR_TRGEN_Msk);
    /*Set ADC Calibration Done Flag*/
    adc_calib_done = true;
}

/******************************************************************************/
/* Function name: MCAPP_MotorPLLEstimInit                                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initializes sensorless MCLIB_PLLEstimator parameters.                  */
/******************************************************************************/
void MCAPP_MotorPLLEstimInit(void)
{
    /* Constants are defined in usreparms.h */
	gMCLIBEstimParam.lsDt = (float)(MOTOR_PER_PHASE_INDUCTANCE / FAST_LOOP_TIME_SEC);
	gMCLIBEstimParam.rs = MOTOR_PER_PHASE_RESISTANCE;

    gMCLIBEstimParam.invKFi = (float)(1.0f / BEMF_CNST_Vpk_PH_RAD_PER_SEC_ELEC);
   	gMCLIBEstimParam.rhoStateVar = 0.0f;
	gMCLIBEstimParam.omegaMr = 0.0f;

    gMCLIBEstimParam.kFilterEsdq = KFILTER_ESDQ;
	gMCLIBEstimParam.kFilterBEMFAmp = KFILTER_BEMF_AMPLITUDE;
    gMCLIBEstimParam.velEstimFilterK = KFILTER_VELESTIM;

    gMCLIBEstimParam.deltaT = FAST_LOOP_TIME_SEC;
    gMCLIBEstimParam.rhoOffset = (45.0f * ((float)M_PI/180.0f));

	gMCLIBEstimParam.bemfAmplitudeFilt = 0.0f;
	gMCLIBEstimParam.velEstim = 0.0f;
	gMCLIBEstimParam.lastIalpha = 0.0f;
	gMCLIBEstimParam.lastIbeta = 0.0f;
	gMCLIBEstimParam.lastVbeta = 0.0f;
	gMCLIBEstimParam.lastValpha = 0.0f;
	gMCLIBEstimParam.esdf = 0.0f;
	gMCLIBEstimParam.esqf = 0.0f;
	gMCLIBEstimParam.rho = 0.0f;
}

/******************************************************************************/
/* Function name: MCAPP_MotorPIParamInit                                     */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Initialize control parameters: PI coefficients for D,Q and                 */
/* Velocity control loop.                                                     */
/******************************************************************************/
void MCAPP_MotorPIParamInit(void)
{
	/**************** PI D Term ***********************************************/
	gPIParmD.kp = D_CURRCNTR_PTERM;
	gPIParmD.ki = D_CURRCNTR_ITERM;
	gPIParmD.kc = D_CURRCNTR_CTERM;
	gPIParmD.outMax = D_CURRCNTR_OUTMAX;
	gPIParmD.outMin = -gPIParmD.outMax;
    gPIParmD.dSum = 0.0f;
    gPIParmD.out = 0.0f;

	/**************** PI Q Term ************************************************/
	gPIParmQ.kp = Q_CURRCNTR_PTERM;
	gPIParmQ.ki = Q_CURRCNTR_ITERM;
	gPIParmQ.kc = Q_CURRCNTR_CTERM;
	gPIParmQ.outMax = Q_CURRCNTR_OUTMAX;
	gPIParmQ.outMin = -gPIParmQ.outMax;
    gPIParmQ.dSum = 0.0f;
    gPIParmQ.out = 0.0f;

	/**************** PI Velocity Control **************************************/
	gPIParmQref.kp = SPEEDCNTR_PTERM;
	gPIParmQref.ki = SPEEDCNTR_ITERM;
	gPIParmQref.kc = SPEEDCNTR_CTERM;
	gPIParmQref.outMax = SPEEDCNTR_OUTMAX;
	gPIParmQref.outMin = -gPIParmQref.outMax;
    gPIParmQref.dSum = 0.0f;
    gPIParmQref.out = 0.0f;
}

/******************************************************************************/
/* Function name: MCAPP_MotorControlParamInit                                            */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initialize control loop variables.                            */
/******************************************************************************/
static void MCAPP_MotorControlParamInit(void)
{
	/* Parameter initialization for FOC */
	MCAPP_MotorPIParamInit();
	MCAPP_MotorPLLEstimInit();

	gCtrlParam.openLoop = true;
	gCtrlParam.open_loop_stab_counter = 0;
	gCtrlParam.startup_angle_ramp_rads_per_sec = 0.0f;
	gCtrlParam.changeMode = false;
	gCtrlParam.startup_lock_count = 0;
	gMCLIBPosition.angle = 0.0f;
	gMCLIBSVPWM.period = (float)((uint32_t)PWM0_ChannelPeriodGet(PWM_CHANNEL_0));
	gCtrlParam.motorStatus = MOTOR_STATUS_STOPPED;
	gMCLIBCurrentDQ.id = 0.0f;
	gMCLIBCurrentDQ.iq = 0.0f;
	gCtrlParam.rampIncStep = SPEED_RAMP_INC_SLOW_LOOP;
}

/******************************************************************************/
/* Function name: MCAPP_SlowLoopTimeIsFinished                                             */
/* Function parameters: None                                                  */
/* Function return: Bool( True - executes slow control loop )                 */
/* Description: To be used in a state machine to decide whether to execute       */
/* slow control loop                                                          */
/******************************************************************************/
__STATIC_INLINE bool MCAPP_SlowLoopTimeIsFinished(void)
{
	bool retval = false;
	if((uint32_t)SLOW_LOOP_TIME_PWM_COUNT <= gCtrlParam.sync_cnt)
	{
		gCtrlParam.sync_cnt = 0;
		retval = true;
	}
	return( retval );
}

#if(TORQUE_MODE == 0)
/******************************************************************************/
/* Function name: MCAPP_SpeedRamp                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Increment/Decrements speed reference based upon ramp          */
/*              configuration in "userparams.h" file.                         */
/******************************************************************************/
__STATIC_INLINE void MCAPP_SpeedRamp(void)
{
	if(gCtrlParam.endSpeed > (gCtrlParam.velRef + gCtrlParam.rampIncStep))
	{
		gCtrlParam.velRef += gCtrlParam.rampIncStep;
	}
	else if(gCtrlParam.endSpeed < (gCtrlParam.velRef - gCtrlParam.rampIncStep))
	{
		gCtrlParam.velRef -= gCtrlParam.rampIncStep;
	}
	else
	{
		gCtrlParam.velRef = gCtrlParam.endSpeed;
	}
}
#endif




/******************************************************************************/
/* Function name: MCAPP_PWMDutyUpdate                                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Change the duty cycle of PWMs                                 */
/******************************************************************************/
__STATIC_INLINE void MCAPP_PWMDutyUpdate(uint32_t duty_PhU,uint32_t duty_PhV,uint32_t duty_PhW)
{
	PWM0_ChannelDutySet(PWM_CHANNEL_0, (uint16_t)duty_PhU);
    PWM0_ChannelDutySet(PWM_CHANNEL_1, (uint16_t)duty_PhV);
    PWM0_ChannelDutySet(PWM_CHANNEL_2, (uint16_t)duty_PhW);
}


/******************************************************************************/
/* Function name: MCAPP_ControlLoopISR                                      *
 * Function parameters: None                                                  *
 * Function return: None                                                      *
 * Description: ADC end of conversion interrupt is used for executing fast    *
 *              control loop.It estimates speed and rotor angle based upon    *
 *              the phase current measurements and updates duty.              *
 ******************************************************************************/

 void MCAPP_ControlLoopISR(uint32_t status, uintptr_t context)
{
	float phaseCurrentU;
	float phaseCurrentV;
	float dcBusVoltage;

	X2CScope_Update();

	/* PC23 GPIO is used for timing measurement. - Set High*/
	LED_Set();

 	/* Motor currents.  */
    phaseCurrentU = (float)((uint32_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)PH_U_CURRENT_ADC_CH));
    phaseCurrentV = (float)((uint32_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)PH_V_CURRENT_ADC_CH));

   /* Remove the offset from measured motor currents */
    phaseCurrentU = phaseCurrentU - phaseCurrentUOffset;
    phaseCurrentV = phaseCurrentV - phaseCurrentVOffset;

	/* Non Inverting amplifiers for current sensing */
    gMCLIBCurrentABC.ia = phaseCurrentU * ADC_CURRENT_SCALE;
    gMCLIBCurrentABC.ib = phaseCurrentV * ADC_CURRENT_SCALE;

    /* Clarke transform */
    MCLIB_ClarkeTransform(&gMCLIBCurrentABC, &gMCLIBCurrentAlphaBeta);

	/* Park transform */
    MCLIB_ParkTransform(&gMCLIBCurrentAlphaBeta, &gMCLIBPosition, &gMCLIBCurrentDQ);

	/* Read DC bus voltage */
	dcBusVoltage = (float)((uint32_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)DC_BUS_VOLTAGE_ADC_CH));
	gfocParam.dcBusVoltage = (float)(dcBusVoltage) * VOLTAGE_ADC_TO_PHY_RATIO;
    gfocParam.dcBusVoltageBySqrt3 = (float)(gfocParam.dcBusVoltage/SQRT3);
    gMCLIBEstimParam.dcBusVoltageBySqrt3 = gfocParam.dcBusVoltageBySqrt3;

    /* Estimates the rotor angle and speed. */
    MCLIB_PLLEstimator(&gMCLIBEstimParam, &gMCLIBPosition);

    /* Calculate control values  */
    MCAPP_MotorCurrentControl();

    /* Calculate park angle */
    MCAPP_MotorAngleCalc();

    /* Check if open loop is active */
    if(gCtrlParam.openLoop == true)
    {
        /* Then angle is manually incremented based upon open loop speed and ramp rate */
        gMCLIBPosition.angle = gfocParam.angle;
    }
    else
    {
        /* In close loop rotor angle is estimated by MCLIB_PLLEstimator. */
        gMCLIBPosition.angle = gMCLIBEstimParam.rho;
    }

	/* Calculate qSin,qCos from qAngle  */
    MCLIB_SinCosCalc(&gMCLIBPosition );

    /* Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq */
    MCLIB_InvParkTransform(&gMCLIBVoltageDQ, &gMCLIBPosition, &gMCLIBVoltageAlphaBeta);

	/* Calculate and set PWM duty cycles from Vr1,Vr2,Vr3 */
    MCLIB_SVPWMGen(&gMCLIBVoltageAlphaBeta, &gMCLIBSVPWM);

    MCAPP_PWMDutyUpdate(gMCLIBSVPWM.dPWM1, gMCLIBSVPWM.dPWM2, gMCLIBSVPWM.dPWM3);

    /* sync count for slow control loop execution */
	gCtrlParam.sync_cnt++;

	/* PC23 GPIO is used for timing measurement. - Set Low*/
	LED_Clear();
}

/******************************************************************************/
/* Function name: SlowControlLoop                                             */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Slow control loop is in sync with fast control loop           */
/*              MCAPP_ControlLoopISR().Slow loop execution frequency should           */
/*			    be in multiple of PWM frequency and be configured by          */
/*				"SLOW_LOOP_TIME_SEC" MACRO in "userparms.h file.              */
/*               Speed ramp and speed control loop is executed from this loop */
/******************************************************************************/
__STATIC_INLINE void MCAPP_SlowControlLoop(void)
{

#if(TORQUE_MODE == 0)
	if(gCtrlParam.openLoop == false)
	{
		/* Velocity reference will be taken from potentiometer if configured. */
		float PotReading;


		PotReading = (float)((uint32_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)POT_ADC_CH));

        speed_ref_filtered = speed_ref_filtered + ((PotReading - speed_ref_filtered) * KFILTER_POT );
		gCtrlParam.velRef = (float)((float)speed_ref_filtered * POT_ADC_COUNT_FW_SPEED_RATIO);

		/* Restrict velocity reference so motor will be spinning in closed loop. */
		if(gCtrlParam.velRef < OPNLP_END_SPEED_RDPS_ELEC)
		{
			gCtrlParam.velRef = OPNLP_END_SPEED_RDPS_ELEC;
		}

		/* Speed Ramp */
		MCAPP_SpeedRamp();


		/* Check if velocity reference is lower than minimum speed requirement */
		if(gCtrlParam.velRef >= OPNLP_END_SPEED_RDPS_ELEC)
		{
			/* Execute the velocity control loop */
			gPIParmQref.inMeas = gMCLIBEstimParam.velEstim;
			gPIParmQref.inRef  = gCtrlParam.velRef;
			MCLIB_PIControl(&gPIParmQref);
			gCtrlParam.iqRef = gPIParmQref.out;
		}
		else
		{
			MCAPP_MotorStop();
		}
	}
#endif	// End of #if(TORQUE_MODE == false)
}

/******************************************************************************/
/* Function name: MotorStart                                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Enables fast control loop and starts the PWMs.               */
/******************************************************************************/
void MCAPP_MotorStart(void)
{
	/* ADC end of conversion interrupt generation for FOC control */
	NVIC_DisableIRQ(AFEC0_IRQn);
	NVIC_ClearPendingIRQ(AFEC0_IRQn);
    MCAPP_MotorControlParamInit();
	NVIC_SetPriority(AFEC0_IRQn, 0);
	AFEC0_CallbackRegister(MCAPP_ControlLoopISR, (uintptr_t)dummyforMisra);
	NVIC_EnableIRQ(AFEC0_IRQn);
	AFEC0_ChannelsInterruptEnable(AFEC_INTERRUPT_EOC_7_MASK);
	((pio_registers_t*)PIO_PORT_D)->PIO_PDR = ~0xF8FFFFFFu; // Enable PWML output.

	gCtrlParam.motorStatus = MOTOR_STATUS_RUNNING;
	gCtrlParam.endSpeed = OPNLP_END_SPEED_RDPS_ELEC;
	/* Clear fault before start */
    PWM0_FaultStatusClear(PWM_FAULT_ID_2);

	/* Enables PWM channels. */
    PWM0_ChannelsStart(PWM_CHANNEL_0_MASK);

    gMCAPPData.mcState = MC_APP_STATE_RUNNING;
}

/******************************************************************************/
/* Function name: MotorStop                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Stops PWM and disables fast control loop.                    */
/******************************************************************************/
void MCAPP_MotorStop(void)
{
	((pio_registers_t*)PIO_PORT_D)->PIO_PER = ~0xF8FFFFFFu; // Disable PWML output.
    /* Disables PWM channels. */
    PWM0_ChannelsStop((PWM_CHANNEL_MASK)((uint8_t)PWM_CHANNEL_0_MASK | (uint8_t)PWM_CHANNEL_1_MASK | (uint8_t)PWM_CHANNEL_2_MASK));

	/* Reset algorithm specific variables for next iteration.*/
	MCAPP_MotorControlParamInit();
	/* ADC end of conversion interrupt generation disabled */
	AFEC0_ChannelsInterruptDisable(AFEC_INTERRUPT_EOC_7_MASK);
	NVIC_DisableIRQ(AFEC0_IRQn);
	NVIC_ClearPendingIRQ(AFEC0_IRQn);

	gCtrlParam.endSpeed = 0.0f;
	gCtrlParam.velRef = 0.0f;
	gCtrlParam.motorStatus = MOTOR_STATUS_STOPPED;
    gMCAPPData.mcState = MC_APP_STATE_STOP;
    speed_ref_filtered = 0.0f;
    adc_calib_done = false;
}

/******************************************************************************/
/* Function name: MCAPP_SwitchDebounce                                                   */
/* Function parameters: state                                                  */
/* Function return: None                                                      */
/* Description: Switch button debounce logic                                         */
/******************************************************************************/
static void MCAPP_SwitchDebounce(MC_APP_STATE state)
{
    if (!(bool)SWITCH_Get())
    {
        gMCAPPData.switchCount++;
        if (gMCAPPData.switchCount >= 0xFFU)
        {
           gMCAPPData.switchCount = 0;
           gMCAPPData.switchState = MC_APP_SWITCH_PRESSED;
        }
    }
    if (gMCAPPData.switchState == MC_APP_SWITCH_PRESSED)
    {
        if ((bool)SWITCH_Get())
        {
            gMCAPPData.switchCount = 0;
            gMCAPPData.switchState = MC_APP_SWITCH_RELEASED;
            gMCAPPData.mcState = state;

        }
    }

}

/******************************************************************************/
/* Function name: MCAPP_Tasks                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Motor State Machine                                           */
/******************************************************************************/
void MCAPP_Tasks(void)
{
switch (gMCAPPData.mcState)
{
    case MC_APP_STATE_INIT:
        ((pio_registers_t*)PIO_PORT_D)->PIO_PER = ~0xF8FFFFFFu; // Disable PWML output.
        NVIC_DisableIRQ(AFEC0_IRQn);
        NVIC_ClearPendingIRQ(AFEC0_IRQn);
        AFEC0_ChannelsInterruptDisable(AFEC_INTERRUPT_EOC_7_MASK);
        if(adc_calib_done == false)
        {
            MCAPP_ADCOffsetCalibration(); // Perform ADC Offset Calibration only once in this state.
        }
        else
        {
            NOP();
        }
        MCAPP_MotorControlParamInit();
        gMCAPPData.switchCount = 0xFF;
        MCAPP_SwitchDebounce(MC_APP_STATE_START);
        break;

    case MC_APP_STATE_START:
        MCAPP_MotorStart();
        gMCAPPData.mcState = MC_APP_STATE_RUNNING;
        break;

    case MC_APP_STATE_RUNNING:
        if(MCAPP_SlowLoopTimeIsFinished())
        {
          MCAPP_SlowControlLoop();
        }
        MCAPP_SwitchDebounce(MC_APP_STATE_STOP);
        break;

    case MC_APP_STATE_STOP:
        MCAPP_MotorStop();
        MCAPP_SwitchDebounce(MC_APP_STATE_START);
        break;

    default:
        /* Undefined state: Should never come here */
        break;
  }


}


/*******************************************************************************
 End of File
*/

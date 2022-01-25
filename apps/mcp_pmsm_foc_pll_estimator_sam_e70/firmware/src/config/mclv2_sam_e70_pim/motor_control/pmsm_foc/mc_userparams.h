/*******************************************************************************
  System Definitions

  File Name:
    userparams.h

  Summary:
    Header file which defines Motor Specific and Board Specific constants 

  Description:
    This file contains the motor and board specific constants. It also defines
 * switches which allows algorithm to be run in debug modes like Open Loop Mode,
 * Torque mode, etc. 

 *******************************************************************************/

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

#ifndef USERPARAMS_H 
#define USERPARAMS_H

/*******************************************************************************
 * Application Configuration
*******************************************************************************/
#define ENABLE                      1u
#define DISABLE                     0u

/* Alignment methods */
#define Q_AXIS                          (0u)
#define D_AXIS                          (1u)

/* Control Loop */
#define OPEN_LOOP                        (0u)
#define SPEED_LOOP                       (1u)
#define TORQUE_LOOP                      (2u)
#define POSITION_LOOP                    (3u)



/** 
 *  Defining DEVELOPER_MODE allows the programmer to monitor debug signals via X2C Scope 
 *  Note: X2C library has to be linked for the DEVELOPER_MODE to function                    
 */

#define DEVELOPER_MODE   ENABLE

#define CONTROL_LOOP                     (SPEED_LOOP)

/**
 *  Defining Q_AXIS_STARTUP causes the open loop startup by injecting current in Q axis  
 *  Un-defining Q_AXIS_STARTUP cause the open loop startup by injecting current in D axis       
 */
#define Q_AXIS_STARTUP    1u
#define ALIGNMENT_METHOD                 (Q_AXIS)  /* alignment method  */

#define POTENTIOMETER_INPUT_ENABLED       ENABLE


/**
 *  Defining BIDIRECTION_CONTROL enables bi-directional control of the motor                            
 */ 
#define BIDIRECTION_CONTROL  ENABLE

/**
 *  Defining RAMP_PROFILE sets the ramp profile of speed in UI input mode                      
 */ 
#define STEP     0u
#define LINEAR   1u
#define RAMP_PROFILE   Step

/**
 *  Field Weakening                     
 */ 
#define FIELD_WEAKENING (DISABLE)

/* Field weakening - Limit for -ve Idref */
#if(FIELD_WEAKENING == ENABLE)
#define MAX_FW_NEGATIVE_ID_REF              (float)(0)
#endif

/*******************************************************************************
 * System Configuration
*******************************************************************************/
/**
 *  PWM frequency in Hz                            
 */ 
#define PWM_FREQUENCY   (20000u)

/**
 *  Current control Frequency                           
 */ 
#define CURRENT_CONTROL_FREQUENCY      ( 1.0f * PWM_FREQUENCY ) 

/**
 * Current control sampling time                            
 */ 
#define CURRENT_CONTROL_SAMPLING_TIME    (float)(1.0f/(float)CURRENT_CONTROL_FREQUENCY)

/*******************************************************************************
 * Motor Parameters 
*******************************************************************************/
/**
 *  Winding Connection Type 
 */
#define WINDING_CONNECTION_TYPE  (STAR)

/**
  *  Check for motor isotropy                  
  */
#define NON_ISOTROPIC_MOTOR

/**
 * Rated electrical speed in RPM
 */
#define RATED_SPEED_IN_RPM   ((int16_t)2804)

/**
 * Maximum electrical speed  with Field Weakening enabled in RPM
 */
#define MAXIMUM_SPEED_IN_RPM    ((int16_t)3500)

/**
 * Minimum close loop electrical speed of the motor in RPM
 */
#define MINIMUM_SPEED_IN_RPM   (500) 

/**
 * Number of pole pairs  
 */
#define NUM_POLE_PAIRS  ((int16_t)5)  

 /**
  *  Stator resistance in Ohm                      
  */
#define MOTOR_PER_PHASE_RESISTANCE_IN_OHM   ((float)0.285)    

 /**
  *  Direct axis inductance in Henry                 
  */
#define DIRECT_AXIS_INDUCTANCE_IN_HENRY  ((float)0.00032)     

/**
  *  Quadrature axis inductance in Henry                 
  */
#define QUADRATURE_AXIS_INDUCTANCE_IN_HENRY  ((float)0.00032) 

 /** 
  * Back EMF Constant in Vpeak/kRPM                   
  */
#define MOTOR_BEMF_CONST_VOLTS_PER_KRPM_MECH  ((float)7.24)

 /** 
  * Air gap flux in Weber                   
  */
#define  AIR_GAP_FLUX    (float)(  60 * MOTOR_BEMF_CONST_VOLTS_PER_KRPM_MECH / ( 1.414 * 1000 * M_PI ))

 /** 
  * Maximum peak current of the motor in amperes                     
  */
#define MAXIMUM_MOTOR_CURRENT_IN_AMPERE   ((float)4)     


/*******************************************************************************
 * Board Parameters 
*******************************************************************************/
/**
 * Current sensing shunt resistance in Ohms
 */ 
#define CURRENT_SENSE_SHUNT_RESISTANCE_IN_OHM (float)0.025 

/**
 * Current sensing amplifier type
 */ 
#define INVERTING_CURRENT_GAIN_AMPLIFIER

/**
 * Current sensing amplifier gain
 */ 
#define CURRENT_SENSE_AMPLIFIER_GAIN (float)15 

/**
 * Voltage divider ratio for bus voltage measurement
 */
#define VOLTAGE_SENSE_DIVIDER_RATIO  (float)0.0625 

/*******************************************************************************
 * PI Controller Parameters  
*******************************************************************************/
/**
 * Speed PI Controller Parameters 
 */
#define SPEED_TS        (float)CURRENT_CONTROL_SAMPLING_TIME
#define SPEED_KP        (float)(0.005)
#define SPEED_KI        (float)(0.002 * CURRENT_CONTROL_SAMPLING_TIME)
#define SPEED_KC        (float)(0.5)
#define SPEED_YMAX      (float)(3)

/**
 * Direct axis current PI Controller Parameters 
 */
#define D_CURRENT_TS    (float)CURRENT_CONTROL_SAMPLING_TIME
#define D_CURRENT_KP    (float)(0.01)
#define D_CURRENT_KI    (float)(1 * CURRENT_CONTROL_SAMPLING_TIME)
#define D_CURRENT_KC    (float)(0.5)
#define D_CURRENT_YMAX  (float)(0.98)

/**
 * Quadrature axis current PI Controller Parameters 
 */
#define Q_CURRENT_TS     (float)CURRENT_CONTROL_SAMPLING_TIME
#define Q_CURRENT_KP     (float)(0.01)
#define Q_CURRENT_KI     (float)(1 * CURRENT_CONTROL_SAMPLING_TIME)
#define Q_CURRENT_KC    (float)(0.5)
#define Q_CURRENT_YMAX   (float)(0.98)



/*******************************************************************************
 * Torque Mode Parameters  
*******************************************************************************/


/*******************************************************************************
 * Start-up Parameters  
*******************************************************************************/

/**
 * Forced alignment lock time 
 */ 
#define START_UP_LOCK_TIME_IN_SECOND   (2) 

/**
  * Start-up current in amperes                          
  */
#define START_UP_CURRENT_IN_AMPERE   (0.4) 


/**
 * Start-up ramp time
 */
#define START_UP_RAMP_TIME_IN_SECOND   (5)


/**
 * Start-up stabilization time
 */
#define START_UP_STAB_TIME_IN_SECOND   (2)


/**
 * Start-up transition speed 
 */
#define START_UP_TRANS_SPEED_IN_RPM    (500)

#endif // USERPARAMS_H


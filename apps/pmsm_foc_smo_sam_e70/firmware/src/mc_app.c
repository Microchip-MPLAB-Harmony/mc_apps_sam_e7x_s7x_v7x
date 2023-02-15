/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mc_app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "definitions.h"                // SYS function prototypes
#include "userparams.h"
#include "mc_app.h"
#include "positionEstimator.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

static MC_APP_DATA mc_appData = 
{
    .direction = 1,
};

/* MC Core Variables */
#define 	SQRT3_BY2     			(float)0.866025403788
#define 	ONE_BY_SQRT3     		(float)0.5773502691
tMC_APP_MC_CONTROL                      MC_APP_MC_CONTROL;
static tPIParm     						PIParmQ;        						/* Parameters for Q axis Currrent PI Controller */
static tPIParm     						PIParmD;        						/* Parameters for D axis Currrent PI Controller */
static tPIParm     						PIParmQref;     						/* Parameters for Speed PI Controller */
static tParkParm							ParkParm;   
static tSincosParm							SincosParm;
static tSVGenParm 							SVGenParm;
static tCtrlParm 						    CtrlParm;

static float 								T1, T2, Ta, Tb, Tc;
static float 								dPWM1, dPWM2, dPWM3;
static float 								Startup_Ramp_Angle_Rads_Per_Sec = 0.0f; 	/* ramp angle variable for initial ramp */
static unsigned int 						Startup_Lock_Count = 0; 				/* lock variable for initial ramp */ 
static unsigned int                        Speed_Maintain_Cnt1 = 0;

static short        						potReading;
static short                               phaseCurrentA;
static short                               phaseCurrentB;

static float								DoControl_Temp1, DoControl_Temp2;
static float                               VelRefRaw;
static float                               deviationTH;
static float                               decayRateTH;
static float                               closeLoopTHI;
static float                               closeLoopTHO;
static float                               finalTHI;
static float                               finalTHO;
static float                               lastValpha;
static float                               lastVbeta;


static tagStateCtrl closeLoopTHstate;
/* Instruction to BEMF observer:
 * 0: BEMF observer keeps the natural state flow;
 * 1: reset BEMF observer;
 * 2: start BEMF observer. */
static uint16_t flagStartObs = 0u;

/* Motion estimator: global variable definitions */
tagPosition positionData;
tagSpeed speedData;
static tagInputPara inputpara;
tagObserverInput observerInput;
static uintptr_t dummyforMisra;
//Cumulative value is initalized to (estimated offset value * 2^MOVING_AVG_WINDOW_SIZE) which helps in expedited tracking of offset value without
//waiting for all the 2^MOVING_AVG_WINDOW_SIZE samples.

static unsigned int cumulative_sum_phaseA = (CURRENT_OFFSET_INIT << MOVING_AVG_WINDOW_SIZE); 
static unsigned int cumulative_sum_phaseB = (CURRENT_OFFSET_INIT << MOVING_AVG_WINDOW_SIZE);
static int32_t moving_average_phaseA = 0;
static int32_t moving_average_phaseB = 0;

#define 							DECIMATE_NOMINAL_SPEED      ((NOMINAL_SPEED_RPM *((float)M_PI/30.0f))*NOPOLESPAIRS/10.0f)                                            

static const float sineTable[TABLE_SIZE] = 
// <editor-fold defaultstate="collapsed" desc="Sine Table">
{
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f,
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f
};
// </editor-fold>
static const float cosineTable[TABLE_SIZE] = 
// <editor-fold defaultstate="collapsed" desc="Cosine Table">
{
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f,
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f
};
// </editor-fold>


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static inline void MC_APP_MC_ParkParaCal(float angle)
{
    SincosParm.Angle = angle;
    ParkParm.Angle = angle;
    MC_APP_MC_SinCos();
    ParkParm.Sin = SincosParm.Sin;
    ParkParm.Cos = SincosParm.Cos;    
}

static inline void MC_APP_MC_UpdateFeedbackSignals(void)
{
    
    phaseCurrentA = (int16_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)PH_U_CURRENT_ADC_CH); // Phase Current A
    phaseCurrentB = (int16_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)PH_V_CURRENT_ADC_CH);// Phase Current B 
    potReading = (int16_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)POT_ADC_CH);   // Speed Potentiometer  
    ParkParm.DCBusVoltage = (float)((uint32_t)AFEC0_ChannelResultGet((AFEC_CHANNEL_NUM)DC_BUS_VOLTAGE_ADC_CH)) * VOLTAGE_ADC_TO_PHY_RATIO; // Reads and translates to actual bus voltage
    ParkParm.MaxPhaseVoltage = (float)(ParkParm.DCBusVoltage*ONE_BY_SQRT3); 
	
    /* Moving Average Filter is implemented to calculate the current offset. Window size of the moving average filter = 2^MOVING_AVG_WINDOW_SIZE
    cumulative_sum_phaseX(n) = cumulative_sum_phaseX(n-1) + phaseCurrentX(n) - moving_average_phaseX(n-1)
    moving_average_phaseX(n) =  cumulative_sum_phaseX(n)/(2^MOVING_AVG_WINDOW_SIZE) */	
    cumulative_sum_phaseA  =  (uint32_t)((int32_t)((int32_t)cumulative_sum_phaseA + phaseCurrentA - moving_average_phaseA));
    moving_average_phaseA  = (int32_t)((uint32_t)(cumulative_sum_phaseA >> MOVING_AVG_WINDOW_SIZE));
    
    /*Bounding the offset value */
    if(moving_average_phaseA > CURRENT_OFFSET_MAX)
    {
        moving_average_phaseA = CURRENT_OFFSET_MAX;
    } 
    else if(moving_average_phaseA < CURRENT_OFFSET_MIN)
    {
        moving_average_phaseA = CURRENT_OFFSET_MIN;
    } 
    else 
    {
         /* Dummy branch for MISRAC compliance*/
    }
		
    cumulative_sum_phaseB  = (uint32_t)((int32_t)((int32_t)cumulative_sum_phaseB + phaseCurrentB - moving_average_phaseB));
    moving_average_phaseB  = (int32_t)((uint32_t)(cumulative_sum_phaseB >> MOVING_AVG_WINDOW_SIZE));
	
	/*Bounding the offset value */
    if(moving_average_phaseB > CURRENT_OFFSET_MAX)
    {
        moving_average_phaseB = CURRENT_OFFSET_MAX;
    } else if(moving_average_phaseB < CURRENT_OFFSET_MIN)
    {
        moving_average_phaseB = CURRENT_OFFSET_MIN;
    } else {
         /* Dummy branch for MISRAC compliance*/
    }
	
    phaseCurrentA = (phaseCurrentA - (int16_t)moving_average_phaseA); // Removing the offset
    phaseCurrentB = (phaseCurrentB - (int16_t)moving_average_phaseB);

    ParkParm.Ia = (float)phaseCurrentA * ADC_CURRENT_SCALE; 
    ParkParm.Ib = (float)phaseCurrentB * ADC_CURRENT_SCALE;    

}

static void MC_APP_MC_UpdatePWMdutyCycle(void)
{   
    PWM0_ChannelDutySet(PWM_CHANNEL_0, (uint16_t) dPWM1);  
    PWM0_ChannelDutySet(PWM_CHANNEL_1, (uint16_t) dPWM2);  
    PWM0_ChannelDutySet(PWM_CHANNEL_2, (uint16_t) dPWM3);    
}

static inline void MC_APP_MC_LoadMotionEstimator(tagObserverInput * observerInputP)
{
    observerInputP->Ialpha = ParkParm.Ialpha;
    observerInputP->Ibeta = ParkParm.Ibeta;
    observerInputP->Ualpha = ParkParm.MaxPhaseVoltage * lastValpha;
    observerInputP->Ubeta = ParkParm.MaxPhaseVoltage * lastVbeta;
    observerInputP->WeRef = CtrlParm.VelRef;
    observerInputP->startObs = flagStartObs;
    lastValpha = ParkParm.Valpha;
    lastVbeta = ParkParm.Vbeta;
}

static inline void MC_APP_MC_Clarke(void)
{
    ParkParm.Ialpha = ParkParm.Ia;
    ParkParm.Ibeta = (ParkParm.Ia * ONE_BY_SQRT3) + (ParkParm.Ib * 2.0f * ONE_BY_SQRT3);
}

static inline void MC_APP_MC_Park(void)
{
    ParkParm.Id =  ParkParm.Ialpha*ParkParm.Cos + ParkParm.Ibeta*ParkParm.Sin;
    ParkParm.Iq = -ParkParm.Ialpha*ParkParm.Sin + ParkParm.Ibeta*ParkParm.Cos;
}

static inline void MC_APP_MC_InvPark(void)
{
    ParkParm.Valpha =  ParkParm.Vd*ParkParm.Cos - ParkParm.Vq*ParkParm.Sin;
    ParkParm.Vbeta  =  ParkParm.Vd*ParkParm.Sin + ParkParm.Vq*ParkParm.Cos;  
}

void MC_APP_MC_SinCos(void)
{
    // IMPORTANT:
    // DO NOT PASS "SincosParm.angle" > 2*PI. There is no software check
    
    // Since we are using "float", it is not possible to get an index of array
    // directly. Almost every time, we will need to do interpolation, as per
    // following equation: -
    // y = y0 + (y1 - y0)*((x - x0)/(x1 - x0))
    
    int16_t y0_Index;
    int16_t y0_IndexNext;
    float x0, y0, y1, temp;
    
    y0_Index = (int16_t)((float)(SincosParm.Angle * ONE_OVER_ANGLE_STEP));
    y0_IndexNext = y0_Index + 1;
    
    if((TABLE_SIZE - 1) <= y0_Index)
    { 
        y0_Index = TABLE_SIZE - 1;  
        y0_IndexNext = 0;    
    } 
    else if(0 > y0_Index)
    {
        y0_Index = 0;
        y0_IndexNext = 1;
    } 
    else{
         /* Dummy branch for MISRAC compliance*/
    }

    x0 = ((float)y0_Index * ANGLE_STEP);        
	temp = ((SincosParm.Angle - x0) * ONE_OVER_ANGLE_STEP);
   
	// Find Sine 
    y0 = sineTable[y0_Index];
    y1 = sineTable[y0_IndexNext];     
    SincosParm.Sin = y0 + ((y1 - y0)*temp);
	
    // Find Cosine 
    y0 = cosineTable[y0_Index];
    y1 = cosineTable[y0_IndexNext];
    SincosParm.Cos = y0 + ((y1 - y0)*temp);
}


// *****************************************************************************
// *****************************************************************************
// Section: MC FOC Control Routine
// *****************************************************************************
// *****************************************************************************
void MC_APP_MC_DoControl( void )
{  	  
    if( MC_APP_MC_CONTROL.bit.OpenLoop==1U )
    {
        // OPENLOOP:  force rotating angle,Vd,Vq
        if( MC_APP_MC_CONTROL.bit.ChangeMode ==1U)
        {
            // just changed to openloop
            MC_APP_MC_CONTROL.bit.ChangeMode = 0U;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.IqRef = 0.0f;
            CtrlParm.IdRef = 0.0f;

            // reinit vars for initial speed ramp
            Startup_Lock_Count = 0u;
            Startup_Ramp_Angle_Rads_Per_Sec = 0.0f;
        }
        
        // q current reference is equal to the vel reference
        // while d current reference is equal to 0
        // for maximum startup torque, set the q current to maximum acceptable
        // value represents the maximum peak value

        CtrlParm.IqRef    = Q_CURRENT_REF_OPENLOOP * (float)mc_appData.direction;
     	
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.Iq;
        PIParmQ.qInRef  = CtrlParm.IqRef;
        MC_APP_MC_CalcPI(&PIParmQ);
        ParkParm.Vq = PIParmQ.qOut;
       
        // PI control for D
        PIParmD.qInMeas = ParkParm.Id;
        PIParmD.qInRef  = CtrlParm.IdRef;
        MC_APP_MC_CalcPI(&PIParmD);
        ParkParm.Vd = PIParmD.qOut;
    } 
    else  // Closed Loop Vector Control
    { 
        if( MC_APP_MC_CONTROL.bit.ChangeMode==1u )
        {
            // just changed from openloop
            MC_APP_MC_CONTROL.bit.ChangeMode = 0u;
            PIParmQref.qdSum = CtrlParm.IqRef;
            CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
            PIParmD.qInRef = 0.0f;
            CtrlParm.IdRef = 0.0f;
        }             
      
        VelRefRaw = (float)((float)potReading * POT_ADC_COUNT_FW_SPEED_RATIO);
        /* LPF */
        CtrlParm.VelRef = (RL_1MINUS_WCTS_VELREF * (CtrlParm.VelRef)) + (RL_WCTS_VELREF * (VelRefRaw));
        
        if(CtrlParm.VelRef < END_SPEED_RADS_PER_SEC_ELEC)
        {
            CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
        }
               
    
        CtrlParm.IqRefmax = MAX_MOTOR_CURRENT;

        // Execute the velocity control loop
        PIParmQref.qInMeas = speedData.WeHat;
        PIParmQref.qInRef  = CtrlParm.VelRef * (float)mc_appData.direction;
        MC_APP_MC_CalcPI(&PIParmQref);
        CtrlParm.IqRef = PIParmQref.qOut;

        CtrlParm.IdRef = 0.0f;
		
        // PI control for D
        PIParmD.qInMeas = ParkParm.Id;          // This is in Amps
        PIParmD.qInRef  = CtrlParm.IdRef;      // This is in Amps
        MC_APP_MC_CalcPI(&PIParmD);
        ParkParm.Vd    =  PIParmD.qOut;          // This is in %. If should be converted to volts, multiply with (DC/2)

        // dynamic d-q adjustment
        // with d component priority
        // vq=sqrt (vs^2 - vd^2)
        // limit vq maximum to the one resulting from the calculation above
        DoControl_Temp2 = PIParmD.qOut * PIParmD.qOut;
        DoControl_Temp1 = 0.98f - DoControl_Temp2;
        if(DoControl_Temp1>=0.0f){
        PIParmQ.qOutMax = (float)sqrt(DoControl_Temp1); 
        }else{
            PIParmQ.qOutMax = (float)sqrt(-DoControl_Temp1);
        }
		
        //Limit Q axis current
        if(CtrlParm.IqRef>CtrlParm.IqRefmax)
        {
            CtrlParm.IqRef = CtrlParm.IqRefmax;
        }
		
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.Iq;          // This is in Amps
        PIParmQ.qInRef  = CtrlParm.IqRef;      // This is in Amps
        MC_APP_MC_CalcPI(&PIParmQ);
        ParkParm.Vq    = PIParmQ.qOut;          // This is in %. If should be converted to volts, multiply with (DC/2)       
    }  /* end of Closed Loop Vector Control */
}

static inline float MC_APP_MC_WrapFrom0To2Pi(float raw){
    float tmp;
    
    if(0.0 > raw)
    { 
        tmp = raw + SINGLE_ELEC_ROT_RADS_PER_SEC;
    } 
    else if(SINGLE_ELEC_ROT_RADS_PER_SEC < raw)
    {
        tmp = raw - SINGLE_ELEC_ROT_RADS_PER_SEC;
    } 
    else
    {
        tmp = raw;
    }    
    
    return tmp;
}

static inline void MC_APP_MC_CloseLoopTHcal(void){
    float tmp1;
    
    tmp1 = ParkParm.Angle;
    
    switch(closeLoopTHstate.state)
    {
        case 0u:  /* Just switched to close-loop. */
            /* Latch the angular deviation between open-loop angle and estimated angle. */
            deviationTH = tmp1 - positionData.TH;
            decayRateTH = deviationTH * RL_1D_2SCNT;     // Enoch - Redundant?
            closeLoopTHI = tmp1;
            closeLoopTHO = tmp1;
            closeLoopTHstate.state = 1u;
            break;
        case 1u:  /* Decrease the deviation between open-loop angle and estimated angle. */
            if(((0.0<decayRateTH) && (deviationTH>decayRateTH)) || 
            ((0.0>decayRateTH) && (deviationTH<decayRateTH)))
            {
                deviationTH = deviationTH - decayRateTH;
                tmp1 = deviationTH + positionData.THI;
                closeLoopTHI = MC_APP_MC_WrapFrom0To2Pi(tmp1);
                tmp1 = deviationTH + positionData.THO;
                closeLoopTHO = MC_APP_MC_WrapFrom0To2Pi(tmp1);
            } 
            else
            {
                closeLoopTHI =  positionData.THI;
                closeLoopTHO =  positionData.THO;
                closeLoopTHstate.state = 2u;
            }     
            break;
        case 2u:  /* Use estimated angle as close-loop angle. */
            closeLoopTHI =  positionData.THI;
            closeLoopTHO =  positionData.THO;
            break;
        default:
            /* Undefined state: Should never come here */
            break;
    }
}

void MC_APP_MC_CalculateParkAngle(void)
{
	if(MC_APP_MC_CONTROL.bit.OpenLoop==1u)
    {   // If open loop  
        if (mc_appData.direction == -1)
        {
            ParkParm.Angle -= Startup_Ramp_Angle_Rads_Per_Sec;        
            if(ParkParm.Angle < 0.0f)
            {
                ParkParm.Angle = ParkParm.Angle + SINGLE_ELEC_ROT_RADS_PER_SEC;      
            }
        }
        else
        {
            ParkParm.Angle += Startup_Ramp_Angle_Rads_Per_Sec;        
            if(ParkParm.Angle >= SINGLE_ELEC_ROT_RADS_PER_SEC)
            {
                ParkParm.Angle = ParkParm.Angle - SINGLE_ELEC_ROT_RADS_PER_SEC;      
            }
        }
        		// begin with the lock sequence, for field alignment. The rotor is locked at angle = 0 for LOCK_COUNT_FOR_LOCK_TIME ~ 2 seconds
		if (Startup_Lock_Count < ((unsigned int)((float)LOCK_COUNT_FOR_LOCK_TIME)))
        {
            Startup_Lock_Count++;  
            Startup_Ramp_Angle_Rads_Per_Sec = 0.0f;
            ParkParm.Angle = 0.0f;
            CtrlParm.VelRef = Startup_Ramp_Angle_Rads_Per_Sec/PWM_TS;
        } 
        else if (Startup_Ramp_Angle_Rads_Per_Sec < END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME)
        {  // then ramp up till the end speed
			Startup_Ramp_Angle_Rads_Per_Sec += OPENLOOP_RAMPSPEED_INCREASERATE;
        } 
        else if(Speed_Maintain_Cnt1 < (uint32_t)RL_CNT_1S)
        {  /* Maintain the open-loop end-speed for certain time. */
            Speed_Maintain_Cnt1++;
            CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
            flagStartObs = 2u;  /* Start BEMF observer. */
        } 
        else 
        {  // switch to closed loop            
            #if (OPEN_LOOP_FUNCTIONING == 0U)
                float tmp1;
                /* Latch the angular deviation between open-loop angle and estimated angle. */
                tmp1 = ParkParm.Angle - positionData.TH;
                if((float)-M_PI > tmp1)
                {
                    deviationTH = tmp1 + RL_2PI;
                } 
                else if((float)M_PI < tmp1)
                {
                    deviationTH = tmp1 - RL_2PI;
                } 
                else
                {
                    deviationTH = tmp1;
                }
                decayRateTH = deviationTH * RL_1D_2SCNT;
                closeLoopTHstate.state = 1u;
                closeLoopTHstate.cnt1 = 0u;
                MC_APP_MC_CONTROL.bit.ChangeMode = 1u;
                MC_APP_MC_CONTROL.bit.OpenLoop = 0u;
            #endif
		}  
        
        finalTHI = ParkParm.Angle;
        finalTHO = ParkParm.Angle;
	} 
    else
    { // closed loop
        MC_APP_MC_CloseLoopTHcal();
        finalTHI = closeLoopTHI;
        finalTHO = closeLoopTHO;
	}
    
	return;
}

// *****************************************************************************
// *****************************************************************************
// Section: MC PI Controller Routines
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_InitControlParameters(void)
{
	// PI D Term     
    PIParmD.qKp = D_CURRCNTR_PTERM;       
    PIParmD.qKi = D_CURRCNTR_ITERM;              
    PIParmD.qKc = D_CURRCNTR_CTERM;       
    PIParmD.qOutMax = D_CURRCNTR_OUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    MC_APP_MC_InitPI(&PIParmD);

    // PI Q Term 
    PIParmQ.qKp = Q_CURRCNTR_PTERM;    
    PIParmQ.qKi = Q_CURRCNTR_ITERM;
    PIParmQ.qKc = Q_CURRCNTR_CTERM;
    PIParmQ.qdSum = 0.0f;
    PIParmQ.qOutMax = Q_CURRCNTR_OUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    MC_APP_MC_InitPI(&PIParmQ);

    // PI Qref Term
    PIParmQref.qKp = SPEEDCNTR_PTERM;       
    PIParmQref.qKi = SPEEDCNTR_ITERM;       
    PIParmQref.qKc = SPEEDCNTR_CTERM;       
    PIParmQref.qOutMax = SPEEDCNTR_OUTMAX;   
    PIParmQref.qOutMin = -PIParmQref.qOutMax;

    MC_APP_MC_InitPI(&PIParmQref);

	return;
}

void MC_APP_MC_InitPI( tPIParm *pParm)
{
    pParm->qdSum = 0.0f;
    pParm->qOut = 0.0f;
}

void MC_APP_MC_CalcPI( tPIParm *pParm)
{
    float Err;
    float U;
    float Exc;
    
    Err  = pParm->qInRef - pParm->qInMeas;
    pParm->qErr =  Err; 
    U  = pParm->qdSum + pParm->qKp * Err;
   
    if( U > pParm->qOutMax )
    {
        pParm->qOut = pParm->qOutMax;
    }    
    else if( U < pParm->qOutMin )
    {
        pParm->qOut = pParm->qOutMin;
    }
    else        
    {
        pParm->qOut = U;  
    }
     
    Exc = U - pParm->qOut;

    pParm->qdSum = pParm->qdSum + pParm->qKi * Err - pParm->qKc * Exc;
}


// *****************************************************************************
// *****************************************************************************
// Section: MC Space Vector Modulation Routines
// *****************************************************************************
// *****************************************************************************
static inline void MC_APP_MC_CalcRefVec(void) 
{
    SVGenParm.Vr1 = ParkParm.Vbeta;
    SVGenParm.Vr2 = (-ParkParm.Vbeta/2.0f + SQRT3_BY2 * ParkParm.Valpha);
    SVGenParm.Vr3 = (-ParkParm.Vbeta/2.0f - SQRT3_BY2 * ParkParm.Valpha);       
} 

static inline void MC_APP_MC_CalcSVGen( void )
{
    if( SVGenParm.Vr1 >= 0.0f )
    {       
		// (xx1)
        if( SVGenParm.Vr2 >= 0.0f )
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
            T1 = SVGenParm.Vr2;
            T2 = SVGenParm.Vr1;
            MC_APP_MC_CalcTimes();
            dPWM1 = Ta;
            dPWM2 = Tb;
            dPWM3 = Tc;
        }
        else
        {            
            // (x01)
            if( SVGenParm.Vr3 >= 0.0f )
            {
                // Sector 5: (1,0,1)  120-180 degrees
                T1 = SVGenParm.Vr1;
                T2 = SVGenParm.Vr3;
                MC_APP_MC_CalcTimes();
                dPWM1 = Tc;
                dPWM2 = Ta;
                dPWM3 = Tb;
            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                T1 = -SVGenParm.Vr2;
                T2 = -SVGenParm.Vr3;
                MC_APP_MC_CalcTimes();
                dPWM1 = Tb;
                dPWM2 = Ta;
                dPWM3 = Tc;
            }
        }
    }
    else
    {
        // (xx0)
        if( SVGenParm.Vr2 >= 0.0f )
        {
			// (x10)
            if( SVGenParm.Vr3 >= 0.0f )
            {
                // Sector 6: (1,1,0)  240-300 degrees
                T1 = SVGenParm.Vr3;
                T2 = SVGenParm.Vr2;
                MC_APP_MC_CalcTimes();
                dPWM1 = Tb;
                dPWM2 = Tc;
                dPWM3 = Ta;
            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                T1 = -SVGenParm.Vr3;
                T2 = -SVGenParm.Vr1;
                MC_APP_MC_CalcTimes();
                dPWM1 = Ta;
                dPWM2 = Tc;
                dPWM3 = Tb;
            }
        }
        else
        {            
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            T1 = -SVGenParm.Vr1;
            T2 = -SVGenParm.Vr2;
            MC_APP_MC_CalcTimes();
            dPWM1 = Tc;
            dPWM2 = Tb;
            dPWM3 = Ta;

        }
    }
}

void MC_APP_MC_CalcTimes(void)
{
    T1 = SVGenParm.PWMPeriod * T1;
    T2 = SVGenParm.PWMPeriod * T2;
    Tc = (SVGenParm.PWMPeriod-T1-T2)/2.0f;
    Tb = Tc + T2;
    Ta = Tb + T1;    
}  

// *****************************************************************************
// *****************************************************************************
// Init routines
// *****************************************************************************
// *****************************************************************************

static inline void MC_APP_MC_InitMotionEstimator(tagObserverInput * observerInputP)
{ 
    inputpara.boundaryI = RL_BOUNDARY_I;
    inputpara.m = RL_M;
    inputpara.lambda = RL_LAMBDA;
    inputpara.wcSpeedFil = RL_WC_SPEED_FIL;
    inputpara.pwmFreq = RL_PWM_FREQUENCY;
    inputpara.rs = RL_RS;
    inputpara.ls = RL_LS;
    inputpara.P = RL_P;
    inputpara.speedRefTime = RL_SPEEDREF_TIME;
    inputpara.expRsLsTs = RL_EXP_MINUS_RSLST;
    
    observerInputP->para = &inputpara;
    observerInputP->positionDataP = &positionData;
    observerInputP->speedDataP = &speedData;
}

void MC_APP_MC_InitMotorParameters(void)
{
    
    MC_APP_MC_CONTROL.bit.OpenLoop = 1u;
    MC_APP_MC_CONTROL.bit.ChangeMode = 1u;
    Startup_Ramp_Angle_Rads_Per_Sec = 0.0f;
    Startup_Lock_Count = 0u;
    Speed_Maintain_Cnt1 = 0;
    flagStartObs = 0;
    ParkParm.Angle = 0.0f;
    SincosParm.Angle = 0.0f;
    closeLoopTHstate.state = 0u;
    closeLoopTHstate.cnt1 = 0u;
    finalTHI = 0.0f;
    finalTHO = 0.0f;
    closeLoopTHI = 0.0f;
    closeLoopTHO = 0.0f;
    CtrlParm.VelRef = 0.0f;
    CtrlParm.IdRef = 0.0f;
    CtrlParm.IqRef = 0.0f;

    positionData.TH = 0.0f;
    positionData.THI = 0.0f;
    positionData.THO = 0.0f;
    positionData.dTH = 0.0f;
    positionData.zTH = 0.0f;

    speedData.SUMdTH = 0.0f;
    speedData.WeHat = 0.0f;
    speedData.WeRaw = 0.0f;
    speedData.WmHat = 0.0f;
    speedData.cntFIFO = 0u;
    
    ParkParm.Ialpha =0.0f;
    ParkParm.Ibeta = 0.0f;
    ParkParm.MaxPhaseVoltage =0.0f;
}

// *****************************************************************************
// *****************************************************************************
// Section: MC ADC ISR TASKS
// *****************************************************************************
// *****************************************************************************
void MC_APP_MC_ControlLoopISR(uint32_t status, uintptr_t context)
{
    X2CScope_Update();
    MC_APP_MC_UpdateFeedbackSignals();
    
    MC_APP_MC_Clarke();
 
    MC_APP_MC_LoadMotionEstimator(&observerInput);
    motionEstimator(&observerInput);  /* BEMF SMO */  
    
    MC_APP_MC_CalculateParkAngle();
 
    MC_APP_MC_ParkParaCal(finalTHI);    
    MC_APP_MC_Park();

    MC_APP_MC_DoControl();

    MC_APP_MC_ParkParaCal(finalTHO);   
    MC_APP_MC_InvPark();

    MC_APP_MC_CalcRefVec();
  
    MC_APP_MC_CalcSVGen();
    MC_APP_MC_UpdatePWMdutyCycle();

}

/******************************************************************************/
/* Function name: MCAPP_SwitchDebounce                                                   */
/* Function parameters: state                                                  */
/* Function return: None                                                      */
/* Description: Switch button debounce logic                                         */
/******************************************************************************/
static void MC_APP_MC_SwitchDebounce(MC_APP_MC_STATES state)
{
    if (START_STOP_BUTTON_Get()!=1U)
    {
        mc_appData.switchCount++;
        if (mc_appData.switchCount >= 0xFFu)
        {
           mc_appData.switchCount = 0u;
           mc_appData.switchState = MC_APP_SWITCH_PRESSED;
        }
    }
    if (mc_appData.switchState == MC_APP_SWITCH_PRESSED)
    {
        if (START_STOP_BUTTON_Get()==1U)
        {
            mc_appData.switchCount = 0u;
            mc_appData.switchState = MC_APP_SWITCH_RELEASED;
            mc_appData.mcState = state;

        }
    }

}

#ifdef MCLV2
/******************************************************************************/
/* Function name: MCAPP_MotorDirectionToggle                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Motor Direction reversal switch                               */
/******************************************************************************/
static void MC_APP_MC_MotorDirectionToggle( void )
{
   static uint16_t switchCount = 0xFFu;
   static MC_APP_SWITCH_STATE switchState = MC_APP_SWITCH_RELEASED;

   /* Check if the push button for motor direction toggling is pressed */
    if (switchState == MC_APP_SWITCH_RELEASED)
    {
        if (DIRECTION_TOGGLE_BUTTON_Get()!=1U)
        {
            switchCount++;
            if (switchCount >= 0xFFu)
            {
               switchCount = 0u;
               switchState = MC_APP_SWITCH_PRESSED;
            }
        }
    }
   if (switchState == MC_APP_SWITCH_PRESSED)
   {
       if (DIRECTION_TOGGLE_BUTTON_Get()==1U)
       {
           switchCount = 0u;
           switchState = MC_APP_SWITCH_RELEASED;

           /* Set LED to indicate the direction reversal */
           LED_Toggle();

           mc_appData.direction = mc_appData.direction * -1;
       }
   }

}
#endif


void MC_APP_MC_Tasks (void )
{
    /* Check the application's current state. */
    switch (mc_appData.mcState )
    {
        /* Application's initial state. */
        case MC_APP_MC_STATE_INIT:
        {
            PWM0_REGS->PWM_OOV = 0x00;
            PWM0_REGS->PWM_OS = 0xF000F;
            mc_appData.switchCount = 0xFF;
            MC_APP_MC_SwitchDebounce(MC_APP_MC_STATE_START);
            break;
        }
		
        case MC_APP_MC_STATE_START:
            MC_APP_MC_CONTROL.bit.OpenLoop = 1u;
            MC_APP_MC_CONTROL.bit.ChangeMode = 1u;
            MC_APP_MC_InitControlParameters();
            MC_APP_MC_InitMotionEstimator(&observerInput);
            resetEstimator(&observerInput);
            MC_APP_MC_InitMotorParameters();
            SincosParm.Angle = 0.0f;
            ParkParm.Angle = 0.0f;;
            SVGenParm.PWMPeriod = (float)((uint32_t)PWM0_ChannelPeriodGet(PWM_CHANNEL_0));

            /* ADC end of conversion interrupt generation for FOC control */
            NVIC_DisableIRQ(AFEC0_IRQn);
            NVIC_ClearPendingIRQ(AFEC0_IRQn);
            NVIC_SetPriority(AFEC0_IRQn, 0);
            AFEC0_CallbackRegister(MC_APP_MC_ControlLoopISR, (uintptr_t)dummyforMisra);
            NVIC_EnableIRQ(AFEC0_IRQn);
            AFEC0_ChannelsInterruptEnable(AFEC_INTERRUPT_EOC_7_MASK);
            ((pio_registers_t*)PIO_PORT_D)->PIO_PDR = ~0xF8FFFFFFu; // Enable PWML output. 

            /* Clear fault before start */
            PWM0_FaultStatusClear(PWM_FAULT_ID_2);
            /* Enable PWM outputs */
            PWM0_REGS->PWM_OS = 0x0;                
            PWM0_ChannelsStart(PWM_CHANNEL_0_MASK);
            mc_appData.mcState = MC_APP_MC_STATE_RUNNING;
            break;
               
        case MC_APP_MC_STATE_RUNNING:
            MC_APP_MC_SwitchDebounce(MC_APP_MC_STATE_STOP);
            break;
               
        case MC_APP_MC_STATE_STOP:
        {
            NVIC_DisableIRQ(AFEC0_IRQn);
            PWM0_REGS->PWM_OS = 0xF000F;
            /* Disables PWM channels. */
            PWM0_ChannelsStop((PWM_CHANNEL_MASK)((uint8_t)PWM_CHANNEL_0_MASK | (uint8_t)PWM_CHANNEL_1_MASK | (uint8_t)PWM_CHANNEL_2_MASK));
            /* ADC end of conversion interrupt generation disabled */
            AFEC0_ChannelsInterruptDisable(AFEC_INTERRUPT_EOC_7_MASK);
            
            NVIC_ClearPendingIRQ(AFEC0_IRQn);
            MC_APP_MC_SwitchDebounce(MC_APP_MC_STATE_START);
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* Undefined state: Should never come here */
            break;
        }
    }
    
#ifdef MCLV2
    if( MC_APP_MC_STATE_RUNNING != mc_appData.mcState )
    {
        MC_APP_MC_MotorDirectionToggle();
    }
#endif        
}

 

/*******************************************************************************
 End of File
 */

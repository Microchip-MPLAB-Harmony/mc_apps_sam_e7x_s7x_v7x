/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    mclib_generic_float.c

  Summary:
    This file contains the motor control algorithm functions.

  Description:
    This file contains the motor control algorithm functions like clarke transform,
    park transform. This library is implemented with float data type.
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
#include "mclib_generic_float.h"
#include "userparams.h"
#include "X2CScope.h"
#include "X2CScopeCommunication.h"
#include "math.h"
/******************************************************************************/
/* Local Function Prototype                                                   */
/******************************************************************************/

__STATIC_INLINE void MCLIB_SVPWMTimeCalc(MCLIB_SVPWM* svm);

/******************************************************************************/
/*                   Global Variables                                         */
/******************************************************************************/

MCLIB_PI                gPIParmQ;        /* Iq PI controllers */
MCLIB_PI                gPIParmD;        /* Id PI controllers */
MCLIB_PI                gPIParmQref;     /* Speed PI controllers */
MCLIB_I_ABC             gMCLIBCurrentABC;
MCLIB_I_ALPHA_BETA      gMCLIBCurrentAlphaBeta;
MCLIB_I_DQ              gMCLIBCurrentDQ;
MCLIB_POSITION          gMCLIBPosition;
MCLIB_V_DQ              gMCLIBVoltageDQ;
MCLIB_V_ALPHA_BETA      gMCLIBVoltageAlphaBeta;
MCLIB_SVPWM             gMCLIBSVPWM;
MCLIB_ESTIMATOR         gMCLIBEstimParam;

/******************************************************************************/
/*                   SIN Table  256  -  0.0244rad resolution                  */
/******************************************************************************/
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
/******************************************************************************/
/*                   COS Table  -  0.0244rad resolution                       */
/******************************************************************************/
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
0.788346,
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
/******************************************************************************/
/* Function name: MCLIB_ClarkeTransform                                                      */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Clarke Transformation                                         */
/******************************************************************************/
 void MCLIB_ClarkeTransform(MCLIB_I_ABC* input, MCLIB_I_ALPHA_BETA* output)
{
    output->iAlpha = input->ia;
    output->iBeta = (input->ia * ONE_BY_SQRT3) + (input->ib * TWO_BY_SQRT3);
}

/******************************************************************************/
/* Function name: MCLIB_ParkTransform                                                        */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Park Transformation.                                          */
/******************************************************************************/
 void MCLIB_ParkTransform(MCLIB_I_ALPHA_BETA* input, MCLIB_POSITION* position, MCLIB_I_DQ* output)
{
    output->id =  gMCLIBCurrentAlphaBeta.iAlpha * position->cosAngle
                        + gMCLIBCurrentAlphaBeta.iBeta * position->sineAngle;
	output->iq = -gMCLIBCurrentAlphaBeta.iAlpha * position->sineAngle
                        + gMCLIBCurrentAlphaBeta.iBeta * position->cosAngle;
}

/******************************************************************************/
/* Function name: MCLIB_InvParkTransform                                                     */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Inverse Park Transformation.                                  */
/******************************************************************************/
 void MCLIB_InvParkTransform(MCLIB_V_DQ* input, MCLIB_POSITION* position, MCLIB_V_ALPHA_BETA* output)
{
    output->vAlpha =  input->vd * position->cosAngle - input->vq * position->sineAngle;
    output->vBeta  =  input->vd * position->sineAngle + input->vq * position->cosAngle;
}

/******************************************************************************/
/* Function name: MCLIB_PLLEstimator                                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Estimation of the speed of the motor and rotor angle based on */
/* inverter voltages and motor currents.                                      */
/******************************************************************************/
 void MCLIB_PLLEstimator(MCLIB_ESTIMATOR* estimParam, MCLIB_POSITION* position)
{
    float tempqVelEstim;
    if(estimParam->velEstim < 0.0f)
    {
        tempqVelEstim = estimParam->velEstim * (-1.0f);
    }
    else
    {
        tempqVelEstim = estimParam->velEstim;
    }

    /* dIalpha = Ialpha - oldIalpha,  dIbeta  = Ibeta - oldIbeta
       difference is made between 2 sampled values @PWM period match.*/

    estimParam->dIalpha	= (gMCLIBCurrentAlphaBeta.iAlpha - estimParam->lastIalpha);
    estimParam->vIndalpha = (estimParam->lsDt * estimParam->dIalpha);

    estimParam->dIbeta	= (gMCLIBCurrentAlphaBeta.iBeta - estimParam->lastIbeta);
    estimParam->vIndbeta  = (estimParam->lsDt * estimParam->dIbeta);

    /* Update  LastIalpha and LastIbeta */
    estimParam->lastIalpha	=	gMCLIBCurrentAlphaBeta.iAlpha;
    estimParam->lastIbeta 	=	gMCLIBCurrentAlphaBeta.iBeta;

    /* Stator voltage equations
       Ualpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
       BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt
	*/

	estimParam->esa		= 	estimParam->lastValpha -
							((estimParam->rs  * gMCLIBCurrentAlphaBeta.iAlpha))
							-estimParam->vIndalpha;

    /* Ubeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
       BEMF = Ubeta - Rs Ibeta - Ls dIbeta/dt
	*/
	estimParam->esb		= 	estimParam->lastVbeta -
							((estimParam->rs  * gMCLIBCurrentAlphaBeta.iBeta ))
							- estimParam->vIndbeta;


    /* Update LastValpha and LastVbeta. Convert per unit representation to volts  */
	estimParam->lastValpha = estimParam->dcBusVoltageBySqrt3 * gMCLIBVoltageAlphaBeta.vAlpha;
	estimParam->lastVbeta = estimParam->dcBusVoltageBySqrt3 * gMCLIBVoltageAlphaBeta.vBeta;


    /* Calculate Sin(Angle) and Cos(Angle) */
    position->angle 	=	estimParam->rho + estimParam->rhoOffset;

    if(position->angle >= SINGLE_ELEC_ROT_RADS_PER_SEC)
	{
        position->angle = position->angle - SINGLE_ELEC_ROT_RADS_PER_SEC;
	}

	/* Determine sin and cos values of the angle from the lookup table. */
	MCLIB_SinCosCalc(position);

    /*    Esd =  Esa*cos(Angle) + Esb*sin(Angle) */
	estimParam->esd		=	((estimParam->esa * position->cosAngle))
							+
							((estimParam->esb * position->sineAngle));

    /*   Esq = -Esa*sin(Angle) + Esb*cos(Rho)  */
	estimParam->esq		=	(( estimParam->esb * position->cosAngle))
							-
							((estimParam->esa * position->sineAngle));

    /* Filter first order for Esd and Esq
       EsdFilter = 1/TFilterd * Intergal{ (Esd-EsdFilter).dt } */

	estimParam->esdf		= estimParam->esdf +
							( (estimParam->esd - estimParam->esdf) * estimParam->kFilterEsdq) ;

	estimParam->esqf		= estimParam->esqf +
							( (estimParam->esq - estimParam->esqf) * estimParam->kFilterEsdq) ;

	/* OmegaMr= InvKfi * (Esqf -sgn(Esqf) * Esdf) */
    /* For stability the condition for low speed */
    if (tempqVelEstim > DECIMATE_RATED_SPEED)
    {
		/* Estimated speed is greater than 10% of rated speed */
    	if(estimParam->esqf > 0.0f)
    	{
    		estimParam->omegaMr = estimParam->invKFi * (estimParam->esqf - estimParam->esdf);
    	}
		else
    	{
    		estimParam->omegaMr = estimParam->invKFi * (estimParam->esqf + estimParam->esdf);
    	}
    }
	else
    {
		/* Estimated speed is less than 10% of rated speed */
    	if(estimParam->velEstim > 0.0f)
    	{
    		estimParam->omegaMr	=	((estimParam->invKFi * (estimParam->esqf - estimParam->esdf))) ;
    	}
		else
    	{
    		estimParam->omegaMr	=	((estimParam->invKFi * (estimParam->esqf + estimParam->esdf))) ;
    	}
    }

    /* the integral of the estimated speed(OmegaMr) is the estimated angle */
	estimParam->rho	= 	estimParam->rho +
							(estimParam->omegaMr) * (estimParam->deltaT);

    if(estimParam->rho >= SINGLE_ELEC_ROT_RADS_PER_SEC)
    {
        estimParam->rho = estimParam->rho - SINGLE_ELEC_ROT_RADS_PER_SEC;
    }else
    {
        /* No operation */
    }


    /* the estimated speed is a filter value of the above calculated OmegaMr. The filter implementation */
    /* is the same as for BEMF d-q components filtering */
	estimParam->velEstim = estimParam->velEstim +
						( (estimParam->omegaMr - estimParam->velEstim) * estimParam->velEstimFilterK );

}	/* End of Estim() */

/******************************************************************************/
/* Function name: MCLIB_SinCosCalc                                                      */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Calculates the sin and cosine of angle based upon             */
/*              interpolation technique from the table.                       */
/******************************************************************************/
 void MCLIB_SinCosCalc(MCLIB_POSITION* position )
{
    /* IMPORTANT:
       DO NOT PASS "SincosParm.angle" > 2*PI. There is no software check

       Since we are using "float", it is not possible to get an index of array
       directly. Almost every time, we will need to do interpolation, as per
       following equation: -
       y = y0 + (y1 - y0)*((x - x0)/(x1 - x0)) */

    uint32_t y0_Index;
    uint32_t y0_IndexNext;
    float x0, x1, y0, y1, temp;

    y0_Index = (uint32_t)((float)(position->angle / ANGLE_STEP));
    y0_IndexNext = y0_Index + 1U;

    if(y0_IndexNext >= TABLE_SIZE )
    {
        y0_IndexNext = 0U;
        x1 = TOTAL_SINE_TABLE_ANGLE;
    }
    else
    {
        x1 = (((float)y0_IndexNext) * ANGLE_STEP);
    }

    x0 = ((float)y0_Index * ANGLE_STEP);

    /* Since below calculation is same for sin & cosine, we can do it once and reuse. */
    temp = ((position->angle - x0) / (x1 - x0));
    /*==============  Find Sine now  =============================================*/
    y0 = sineTable[y0_Index];
    y1 = sineTable[y0_IndexNext];
    position->sineAngle = y0 + ((y1 - y0) * temp);
   /*==============  Find Cosine now  =============================================*/
    y0 = cosineTable[y0_Index];
    y1 = cosineTable[y0_IndexNext];
    position->cosAngle = y0 + ((y1 - y0) * temp);
}

/******************************************************************************/
/* Function name: MCLIB_PIControl                                                   */
/* Function parameters: pParm - PI parameter structure                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Execute PI control                                                         */
/******************************************************************************/
 void MCLIB_PIControl( MCLIB_PI *pParm)
{
	float Err;
	float Out;
	float Exc;

	Err  = pParm->inRef - pParm->inMeas;
	Out  = pParm->dSum + pParm->kp * Err;

	/* Limit checking for PI output */
	if( Out > pParm->outMax )
    {
        pParm->out = pParm->outMax;
    }
	else if( Out < pParm->outMin )
    {
        pParm->out = pParm->outMin;
    }
	else
    {
        pParm->out = Out;
    }

	Exc = Out - pParm->out;
	pParm->dSum = pParm->dSum + pParm->ki * Err - pParm->kc * Exc;
}



/******************************************************************************/
/* Function name: MCLIB_SVPWMTimeCalc                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Calculates time to apply vector a,b,c                         */
/******************************************************************************/
__STATIC_INLINE void MCLIB_SVPWMTimeCalc(MCLIB_SVPWM* svm)
{
    svm->t1 = (gMCLIBSVPWM.period) * svm->t1;
    svm->t2 = (gMCLIBSVPWM.period) * svm->t2;
    svm->t_c = (gMCLIBSVPWM.period - svm->t1 - svm->t2)/2.0f;
    svm->t_b = svm->t_c + svm->t2;
    svm->t_a = svm->t_b + svm->t1;
}

/******************************************************************************/
/* Function name: MCLIB_SVPWMGen                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Determines sector based upon three reference vectors amplitude*/
/*              and updates duty.                                             */
/******************************************************************************/
 void MCLIB_SVPWMGen( MCLIB_V_ALPHA_BETA* vAlphaBeta, MCLIB_SVPWM* svm )
{
    svm->vr1 = vAlphaBeta->vBeta;
    svm->vr2 = (-vAlphaBeta->vBeta/2.0f + SQRT3_BY2 * vAlphaBeta->vAlpha);
    svm->vr3 = (-vAlphaBeta->vBeta/2.0f - SQRT3_BY2 * vAlphaBeta->vAlpha);

	if( svm->vr1 >= 0.0f )
	{
		// (xx1)
		if( svm->vr2 >= 0.0f )
		{
			// (x11)
			// Must be Sector 3 since Sector 7 not allowed
			// Sector 3: (0,1,1)  0-60 degrees
			svm->t1 = svm->vr2;
			svm->t2 = svm->vr1;
			MCLIB_SVPWMTimeCalc(svm);
			svm->dPWM1 = (uint32_t)svm->t_a;
			svm->dPWM2 = (uint32_t)svm->t_b;
			svm->dPWM3 = (uint32_t)svm->t_c;
		}
		else
		{
			// (x01)
			if( svm->vr3 >= 0.0f )
			{
				// Sector 5: (1,0,1)  120-180 degrees
				svm->t1 = svm->vr1;
				svm->t2 = svm->vr3;
				MCLIB_SVPWMTimeCalc(svm);
				svm->dPWM1 = (uint32_t)svm->t_c;
				svm->dPWM2 = (uint32_t)svm->t_a;
				svm->dPWM3 = (uint32_t)svm->t_b;

			}
			else
			{
				// Sector 1: (0,0,1)  60-120 degrees
				svm->t1 = -svm->vr2;
				svm->t2 = -svm->vr3;
				MCLIB_SVPWMTimeCalc(svm);
				svm->dPWM1 = (uint32_t)svm->t_b;
				svm->dPWM2 = (uint32_t)svm->t_a;
				svm->dPWM3 = (uint32_t)svm->t_c;
			}
		}
	}
	else
	{
		// (xx0)
		if( svm->vr2 >= 0.0f )
		{
			// (x10)
			if( svm->vr3 >= 0.0f )
			{
				// Sector 6: (1,1,0)  240-300 degrees
				svm->t1 = svm->vr3;
				svm->t2 = svm->vr2;
				MCLIB_SVPWMTimeCalc(svm);
				svm->dPWM1 = (uint32_t)svm->t_b;
				svm->dPWM2 = (uint32_t)svm->t_c;
				svm->dPWM3 = (uint32_t)svm->t_a;
			}
			else
			{
				// Sector 2: (0,1,0)  300-0 degrees
				svm->t1 = -svm->vr3;
				svm->t2 = -svm->vr1;
				MCLIB_SVPWMTimeCalc(svm);
				svm->dPWM1 = (uint32_t)svm->t_a;
				svm->dPWM2 = (uint32_t)svm->t_c;
				svm->dPWM3 = (uint32_t)svm->t_b;
			}
		}
		else
		{
			// (x00)
			// Must be Sector 4 since Sector 0 not allowed
			// Sector 4: (1,0,0)  180-240 degrees
			svm->t1 = -svm->vr1;
			svm->t2 = -svm->vr2;
			MCLIB_SVPWMTimeCalc(svm);
			svm->dPWM1 = (uint32_t)svm->t_c;
			svm->dPWM2 = (uint32_t)svm->t_b;
			svm->dPWM3 = (uint32_t)svm->t_a;
		}
	}
}


/*******************************************************************************
 End of File
*/

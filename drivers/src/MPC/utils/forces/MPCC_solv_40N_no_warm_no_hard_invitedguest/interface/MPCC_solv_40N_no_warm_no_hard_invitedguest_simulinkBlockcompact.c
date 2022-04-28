/*
MPCC_solv_40N_no_warm_no_hard_invitedguest : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME MPCC_solv_40N_no_warm_no_hard_invitedguest_simulinkBlockcompact

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/MPCC_solv_40N_no_warm_no_hard_invitedguest.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef MPCC_solv_40N_no_warm_no_hard_invitedguestinterface_float MPCC_solv_40N_no_warm_no_hard_invitedguestnmpc_float;





/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		return; /* Parameter mismatch will be reported by Simulink */
    }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 7 in total */
    if (!ssSetNumInputPorts(S, 7)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 10, 40);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 10, 560);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 14, 574);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 14, 41);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 2, 40);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 2, 560);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/
	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 8, 1);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/
 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 492, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType( S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	solver_int32_default i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *c = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *C = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *H = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *f = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *b = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *A = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *c_1 = (const real_T*) ssGetInputPortSignal(S,6);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static MPCC_solv_40N_no_warm_no_hard_invitedguest_params params;
	static MPCC_solv_40N_no_warm_no_hard_invitedguest_output output;
	static MPCC_solv_40N_no_warm_no_hard_invitedguest_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<8; i++)
	{ 
		params.c_1[i] = (double) c_1[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_2[i] = (double) c[i]; 
	}

	j=10; 
	for( i=0; i<10; i++)
	{ 
		params.c_3[i] = (double) c[j++]; 
	}

	j=20; 
	for( i=0; i<10; i++)
	{ 
		params.c_4[i] = (double) c[j++]; 
	}

	j=30; 
	for( i=0; i<10; i++)
	{ 
		params.c_5[i] = (double) c[j++]; 
	}

	j=40; 
	for( i=0; i<10; i++)
	{ 
		params.c_6[i] = (double) c[j++]; 
	}

	j=50; 
	for( i=0; i<10; i++)
	{ 
		params.c_7[i] = (double) c[j++]; 
	}

	j=60; 
	for( i=0; i<10; i++)
	{ 
		params.c_8[i] = (double) c[j++]; 
	}

	j=70; 
	for( i=0; i<10; i++)
	{ 
		params.c_9[i] = (double) c[j++]; 
	}

	j=80; 
	for( i=0; i<10; i++)
	{ 
		params.c_10[i] = (double) c[j++]; 
	}

	j=90; 
	for( i=0; i<10; i++)
	{ 
		params.c_11[i] = (double) c[j++]; 
	}

	j=100; 
	for( i=0; i<10; i++)
	{ 
		params.c_12[i] = (double) c[j++]; 
	}

	j=110; 
	for( i=0; i<10; i++)
	{ 
		params.c_13[i] = (double) c[j++]; 
	}

	j=120; 
	for( i=0; i<10; i++)
	{ 
		params.c_14[i] = (double) c[j++]; 
	}

	j=130; 
	for( i=0; i<10; i++)
	{ 
		params.c_15[i] = (double) c[j++]; 
	}

	j=140; 
	for( i=0; i<10; i++)
	{ 
		params.c_16[i] = (double) c[j++]; 
	}

	j=150; 
	for( i=0; i<10; i++)
	{ 
		params.c_17[i] = (double) c[j++]; 
	}

	j=160; 
	for( i=0; i<10; i++)
	{ 
		params.c_18[i] = (double) c[j++]; 
	}

	j=170; 
	for( i=0; i<10; i++)
	{ 
		params.c_19[i] = (double) c[j++]; 
	}

	j=180; 
	for( i=0; i<10; i++)
	{ 
		params.c_20[i] = (double) c[j++]; 
	}

	j=190; 
	for( i=0; i<10; i++)
	{ 
		params.c_21[i] = (double) c[j++]; 
	}

	j=200; 
	for( i=0; i<10; i++)
	{ 
		params.c_22[i] = (double) c[j++]; 
	}

	j=210; 
	for( i=0; i<10; i++)
	{ 
		params.c_23[i] = (double) c[j++]; 
	}

	j=220; 
	for( i=0; i<10; i++)
	{ 
		params.c_24[i] = (double) c[j++]; 
	}

	j=230; 
	for( i=0; i<10; i++)
	{ 
		params.c_25[i] = (double) c[j++]; 
	}

	j=240; 
	for( i=0; i<10; i++)
	{ 
		params.c_26[i] = (double) c[j++]; 
	}

	j=250; 
	for( i=0; i<10; i++)
	{ 
		params.c_27[i] = (double) c[j++]; 
	}

	j=260; 
	for( i=0; i<10; i++)
	{ 
		params.c_28[i] = (double) c[j++]; 
	}

	j=270; 
	for( i=0; i<10; i++)
	{ 
		params.c_29[i] = (double) c[j++]; 
	}

	j=280; 
	for( i=0; i<10; i++)
	{ 
		params.c_30[i] = (double) c[j++]; 
	}

	j=290; 
	for( i=0; i<10; i++)
	{ 
		params.c_31[i] = (double) c[j++]; 
	}

	j=300; 
	for( i=0; i<10; i++)
	{ 
		params.c_32[i] = (double) c[j++]; 
	}

	j=310; 
	for( i=0; i<10; i++)
	{ 
		params.c_33[i] = (double) c[j++]; 
	}

	j=320; 
	for( i=0; i<10; i++)
	{ 
		params.c_34[i] = (double) c[j++]; 
	}

	j=330; 
	for( i=0; i<10; i++)
	{ 
		params.c_35[i] = (double) c[j++]; 
	}

	j=340; 
	for( i=0; i<10; i++)
	{ 
		params.c_36[i] = (double) c[j++]; 
	}

	j=350; 
	for( i=0; i<10; i++)
	{ 
		params.c_37[i] = (double) c[j++]; 
	}

	j=360; 
	for( i=0; i<10; i++)
	{ 
		params.c_38[i] = (double) c[j++]; 
	}

	j=370; 
	for( i=0; i<10; i++)
	{ 
		params.c_39[i] = (double) c[j++]; 
	}

	j=380; 
	for( i=0; i<10; i++)
	{ 
		params.c_40[i] = (double) c[j++]; 
	}

	j=390; 
	for( i=0; i<10; i++)
	{ 
		params.c_41[i] = (double) c[j++]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_1[i] = (double) H[i]; 
	}

	j=196; 
	for( i=0; i<196; i++)
	{ 
		params.H_2[i] = (double) H[j++]; 
	}

	j=392; 
	for( i=0; i<196; i++)
	{ 
		params.H_3[i] = (double) H[j++]; 
	}

	j=588; 
	for( i=0; i<196; i++)
	{ 
		params.H_4[i] = (double) H[j++]; 
	}

	j=784; 
	for( i=0; i<196; i++)
	{ 
		params.H_5[i] = (double) H[j++]; 
	}

	j=980; 
	for( i=0; i<196; i++)
	{ 
		params.H_6[i] = (double) H[j++]; 
	}

	j=1176; 
	for( i=0; i<196; i++)
	{ 
		params.H_7[i] = (double) H[j++]; 
	}

	j=1372; 
	for( i=0; i<196; i++)
	{ 
		params.H_8[i] = (double) H[j++]; 
	}

	j=1568; 
	for( i=0; i<196; i++)
	{ 
		params.H_9[i] = (double) H[j++]; 
	}

	j=1764; 
	for( i=0; i<196; i++)
	{ 
		params.H_10[i] = (double) H[j++]; 
	}

	j=1960; 
	for( i=0; i<196; i++)
	{ 
		params.H_11[i] = (double) H[j++]; 
	}

	j=2156; 
	for( i=0; i<196; i++)
	{ 
		params.H_12[i] = (double) H[j++]; 
	}

	j=2352; 
	for( i=0; i<196; i++)
	{ 
		params.H_13[i] = (double) H[j++]; 
	}

	j=2548; 
	for( i=0; i<196; i++)
	{ 
		params.H_14[i] = (double) H[j++]; 
	}

	j=2744; 
	for( i=0; i<196; i++)
	{ 
		params.H_15[i] = (double) H[j++]; 
	}

	j=2940; 
	for( i=0; i<196; i++)
	{ 
		params.H_16[i] = (double) H[j++]; 
	}

	j=3136; 
	for( i=0; i<196; i++)
	{ 
		params.H_17[i] = (double) H[j++]; 
	}

	j=3332; 
	for( i=0; i<196; i++)
	{ 
		params.H_18[i] = (double) H[j++]; 
	}

	j=3528; 
	for( i=0; i<196; i++)
	{ 
		params.H_19[i] = (double) H[j++]; 
	}

	j=3724; 
	for( i=0; i<196; i++)
	{ 
		params.H_20[i] = (double) H[j++]; 
	}

	j=3920; 
	for( i=0; i<196; i++)
	{ 
		params.H_21[i] = (double) H[j++]; 
	}

	j=4116; 
	for( i=0; i<196; i++)
	{ 
		params.H_22[i] = (double) H[j++]; 
	}

	j=4312; 
	for( i=0; i<196; i++)
	{ 
		params.H_23[i] = (double) H[j++]; 
	}

	j=4508; 
	for( i=0; i<196; i++)
	{ 
		params.H_24[i] = (double) H[j++]; 
	}

	j=4704; 
	for( i=0; i<196; i++)
	{ 
		params.H_25[i] = (double) H[j++]; 
	}

	j=4900; 
	for( i=0; i<196; i++)
	{ 
		params.H_26[i] = (double) H[j++]; 
	}

	j=5096; 
	for( i=0; i<196; i++)
	{ 
		params.H_27[i] = (double) H[j++]; 
	}

	j=5292; 
	for( i=0; i<196; i++)
	{ 
		params.H_28[i] = (double) H[j++]; 
	}

	j=5488; 
	for( i=0; i<196; i++)
	{ 
		params.H_29[i] = (double) H[j++]; 
	}

	j=5684; 
	for( i=0; i<196; i++)
	{ 
		params.H_30[i] = (double) H[j++]; 
	}

	j=5880; 
	for( i=0; i<196; i++)
	{ 
		params.H_31[i] = (double) H[j++]; 
	}

	j=6076; 
	for( i=0; i<196; i++)
	{ 
		params.H_32[i] = (double) H[j++]; 
	}

	j=6272; 
	for( i=0; i<196; i++)
	{ 
		params.H_33[i] = (double) H[j++]; 
	}

	j=6468; 
	for( i=0; i<196; i++)
	{ 
		params.H_34[i] = (double) H[j++]; 
	}

	j=6664; 
	for( i=0; i<196; i++)
	{ 
		params.H_35[i] = (double) H[j++]; 
	}

	j=6860; 
	for( i=0; i<196; i++)
	{ 
		params.H_36[i] = (double) H[j++]; 
	}

	j=7056; 
	for( i=0; i<196; i++)
	{ 
		params.H_37[i] = (double) H[j++]; 
	}

	j=7252; 
	for( i=0; i<196; i++)
	{ 
		params.H_38[i] = (double) H[j++]; 
	}

	j=7448; 
	for( i=0; i<196; i++)
	{ 
		params.H_39[i] = (double) H[j++]; 
	}

	j=7644; 
	for( i=0; i<196; i++)
	{ 
		params.H_40[i] = (double) H[j++]; 
	}

	j=7840; 
	for( i=0; i<196; i++)
	{ 
		params.H_41[i] = (double) H[j++]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_1[i] = (double) f[i]; 
	}

	j=14; 
	for( i=0; i<14; i++)
	{ 
		params.f_2[i] = (double) f[j++]; 
	}

	j=28; 
	for( i=0; i<14; i++)
	{ 
		params.f_3[i] = (double) f[j++]; 
	}

	j=42; 
	for( i=0; i<14; i++)
	{ 
		params.f_4[i] = (double) f[j++]; 
	}

	j=56; 
	for( i=0; i<14; i++)
	{ 
		params.f_5[i] = (double) f[j++]; 
	}

	j=70; 
	for( i=0; i<14; i++)
	{ 
		params.f_6[i] = (double) f[j++]; 
	}

	j=84; 
	for( i=0; i<14; i++)
	{ 
		params.f_7[i] = (double) f[j++]; 
	}

	j=98; 
	for( i=0; i<14; i++)
	{ 
		params.f_8[i] = (double) f[j++]; 
	}

	j=112; 
	for( i=0; i<14; i++)
	{ 
		params.f_9[i] = (double) f[j++]; 
	}

	j=126; 
	for( i=0; i<14; i++)
	{ 
		params.f_10[i] = (double) f[j++]; 
	}

	j=140; 
	for( i=0; i<14; i++)
	{ 
		params.f_11[i] = (double) f[j++]; 
	}

	j=154; 
	for( i=0; i<14; i++)
	{ 
		params.f_12[i] = (double) f[j++]; 
	}

	j=168; 
	for( i=0; i<14; i++)
	{ 
		params.f_13[i] = (double) f[j++]; 
	}

	j=182; 
	for( i=0; i<14; i++)
	{ 
		params.f_14[i] = (double) f[j++]; 
	}

	j=196; 
	for( i=0; i<14; i++)
	{ 
		params.f_15[i] = (double) f[j++]; 
	}

	j=210; 
	for( i=0; i<14; i++)
	{ 
		params.f_16[i] = (double) f[j++]; 
	}

	j=224; 
	for( i=0; i<14; i++)
	{ 
		params.f_17[i] = (double) f[j++]; 
	}

	j=238; 
	for( i=0; i<14; i++)
	{ 
		params.f_18[i] = (double) f[j++]; 
	}

	j=252; 
	for( i=0; i<14; i++)
	{ 
		params.f_19[i] = (double) f[j++]; 
	}

	j=266; 
	for( i=0; i<14; i++)
	{ 
		params.f_20[i] = (double) f[j++]; 
	}

	j=280; 
	for( i=0; i<14; i++)
	{ 
		params.f_21[i] = (double) f[j++]; 
	}

	j=294; 
	for( i=0; i<14; i++)
	{ 
		params.f_22[i] = (double) f[j++]; 
	}

	j=308; 
	for( i=0; i<14; i++)
	{ 
		params.f_23[i] = (double) f[j++]; 
	}

	j=322; 
	for( i=0; i<14; i++)
	{ 
		params.f_24[i] = (double) f[j++]; 
	}

	j=336; 
	for( i=0; i<14; i++)
	{ 
		params.f_25[i] = (double) f[j++]; 
	}

	j=350; 
	for( i=0; i<14; i++)
	{ 
		params.f_26[i] = (double) f[j++]; 
	}

	j=364; 
	for( i=0; i<14; i++)
	{ 
		params.f_27[i] = (double) f[j++]; 
	}

	j=378; 
	for( i=0; i<14; i++)
	{ 
		params.f_28[i] = (double) f[j++]; 
	}

	j=392; 
	for( i=0; i<14; i++)
	{ 
		params.f_29[i] = (double) f[j++]; 
	}

	j=406; 
	for( i=0; i<14; i++)
	{ 
		params.f_30[i] = (double) f[j++]; 
	}

	j=420; 
	for( i=0; i<14; i++)
	{ 
		params.f_31[i] = (double) f[j++]; 
	}

	j=434; 
	for( i=0; i<14; i++)
	{ 
		params.f_32[i] = (double) f[j++]; 
	}

	j=448; 
	for( i=0; i<14; i++)
	{ 
		params.f_33[i] = (double) f[j++]; 
	}

	j=462; 
	for( i=0; i<14; i++)
	{ 
		params.f_34[i] = (double) f[j++]; 
	}

	j=476; 
	for( i=0; i<14; i++)
	{ 
		params.f_35[i] = (double) f[j++]; 
	}

	j=490; 
	for( i=0; i<14; i++)
	{ 
		params.f_36[i] = (double) f[j++]; 
	}

	j=504; 
	for( i=0; i<14; i++)
	{ 
		params.f_37[i] = (double) f[j++]; 
	}

	j=518; 
	for( i=0; i<14; i++)
	{ 
		params.f_38[i] = (double) f[j++]; 
	}

	j=532; 
	for( i=0; i<14; i++)
	{ 
		params.f_39[i] = (double) f[j++]; 
	}

	j=546; 
	for( i=0; i<14; i++)
	{ 
		params.f_40[i] = (double) f[j++]; 
	}

	j=560; 
	for( i=0; i<14; i++)
	{ 
		params.f_41[i] = (double) f[j++]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_1[i] = (double) C[i]; 
	}

	j=140; 
	for( i=0; i<140; i++)
	{ 
		params.C_2[i] = (double) C[j++]; 
	}

	j=280; 
	for( i=0; i<140; i++)
	{ 
		params.C_3[i] = (double) C[j++]; 
	}

	j=420; 
	for( i=0; i<140; i++)
	{ 
		params.C_4[i] = (double) C[j++]; 
	}

	j=560; 
	for( i=0; i<140; i++)
	{ 
		params.C_5[i] = (double) C[j++]; 
	}

	j=700; 
	for( i=0; i<140; i++)
	{ 
		params.C_6[i] = (double) C[j++]; 
	}

	j=840; 
	for( i=0; i<140; i++)
	{ 
		params.C_7[i] = (double) C[j++]; 
	}

	j=980; 
	for( i=0; i<140; i++)
	{ 
		params.C_8[i] = (double) C[j++]; 
	}

	j=1120; 
	for( i=0; i<140; i++)
	{ 
		params.C_9[i] = (double) C[j++]; 
	}

	j=1260; 
	for( i=0; i<140; i++)
	{ 
		params.C_10[i] = (double) C[j++]; 
	}

	j=1400; 
	for( i=0; i<140; i++)
	{ 
		params.C_11[i] = (double) C[j++]; 
	}

	j=1540; 
	for( i=0; i<140; i++)
	{ 
		params.C_12[i] = (double) C[j++]; 
	}

	j=1680; 
	for( i=0; i<140; i++)
	{ 
		params.C_13[i] = (double) C[j++]; 
	}

	j=1820; 
	for( i=0; i<140; i++)
	{ 
		params.C_14[i] = (double) C[j++]; 
	}

	j=1960; 
	for( i=0; i<140; i++)
	{ 
		params.C_15[i] = (double) C[j++]; 
	}

	j=2100; 
	for( i=0; i<140; i++)
	{ 
		params.C_16[i] = (double) C[j++]; 
	}

	j=2240; 
	for( i=0; i<140; i++)
	{ 
		params.C_17[i] = (double) C[j++]; 
	}

	j=2380; 
	for( i=0; i<140; i++)
	{ 
		params.C_18[i] = (double) C[j++]; 
	}

	j=2520; 
	for( i=0; i<140; i++)
	{ 
		params.C_19[i] = (double) C[j++]; 
	}

	j=2660; 
	for( i=0; i<140; i++)
	{ 
		params.C_20[i] = (double) C[j++]; 
	}

	j=2800; 
	for( i=0; i<140; i++)
	{ 
		params.C_21[i] = (double) C[j++]; 
	}

	j=2940; 
	for( i=0; i<140; i++)
	{ 
		params.C_22[i] = (double) C[j++]; 
	}

	j=3080; 
	for( i=0; i<140; i++)
	{ 
		params.C_23[i] = (double) C[j++]; 
	}

	j=3220; 
	for( i=0; i<140; i++)
	{ 
		params.C_24[i] = (double) C[j++]; 
	}

	j=3360; 
	for( i=0; i<140; i++)
	{ 
		params.C_25[i] = (double) C[j++]; 
	}

	j=3500; 
	for( i=0; i<140; i++)
	{ 
		params.C_26[i] = (double) C[j++]; 
	}

	j=3640; 
	for( i=0; i<140; i++)
	{ 
		params.C_27[i] = (double) C[j++]; 
	}

	j=3780; 
	for( i=0; i<140; i++)
	{ 
		params.C_28[i] = (double) C[j++]; 
	}

	j=3920; 
	for( i=0; i<140; i++)
	{ 
		params.C_29[i] = (double) C[j++]; 
	}

	j=4060; 
	for( i=0; i<140; i++)
	{ 
		params.C_30[i] = (double) C[j++]; 
	}

	j=4200; 
	for( i=0; i<140; i++)
	{ 
		params.C_31[i] = (double) C[j++]; 
	}

	j=4340; 
	for( i=0; i<140; i++)
	{ 
		params.C_32[i] = (double) C[j++]; 
	}

	j=4480; 
	for( i=0; i<140; i++)
	{ 
		params.C_33[i] = (double) C[j++]; 
	}

	j=4620; 
	for( i=0; i<140; i++)
	{ 
		params.C_34[i] = (double) C[j++]; 
	}

	j=4760; 
	for( i=0; i<140; i++)
	{ 
		params.C_35[i] = (double) C[j++]; 
	}

	j=4900; 
	for( i=0; i<140; i++)
	{ 
		params.C_36[i] = (double) C[j++]; 
	}

	j=5040; 
	for( i=0; i<140; i++)
	{ 
		params.C_37[i] = (double) C[j++]; 
	}

	j=5180; 
	for( i=0; i<140; i++)
	{ 
		params.C_38[i] = (double) C[j++]; 
	}

	j=5320; 
	for( i=0; i<140; i++)
	{ 
		params.C_39[i] = (double) C[j++]; 
	}

	j=5460; 
	for( i=0; i<140; i++)
	{ 
		params.C_40[i] = (double) C[j++]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_2[i] = (double) A[i]; 
	}

	j=28; 
	for( i=0; i<28; i++)
	{ 
		params.A_3[i] = (double) A[j++]; 
	}

	j=56; 
	for( i=0; i<28; i++)
	{ 
		params.A_4[i] = (double) A[j++]; 
	}

	j=84; 
	for( i=0; i<28; i++)
	{ 
		params.A_5[i] = (double) A[j++]; 
	}

	j=112; 
	for( i=0; i<28; i++)
	{ 
		params.A_6[i] = (double) A[j++]; 
	}

	j=140; 
	for( i=0; i<28; i++)
	{ 
		params.A_7[i] = (double) A[j++]; 
	}

	j=168; 
	for( i=0; i<28; i++)
	{ 
		params.A_8[i] = (double) A[j++]; 
	}

	j=196; 
	for( i=0; i<28; i++)
	{ 
		params.A_9[i] = (double) A[j++]; 
	}

	j=224; 
	for( i=0; i<28; i++)
	{ 
		params.A_10[i] = (double) A[j++]; 
	}

	j=252; 
	for( i=0; i<28; i++)
	{ 
		params.A_11[i] = (double) A[j++]; 
	}

	j=280; 
	for( i=0; i<28; i++)
	{ 
		params.A_12[i] = (double) A[j++]; 
	}

	j=308; 
	for( i=0; i<28; i++)
	{ 
		params.A_13[i] = (double) A[j++]; 
	}

	j=336; 
	for( i=0; i<28; i++)
	{ 
		params.A_14[i] = (double) A[j++]; 
	}

	j=364; 
	for( i=0; i<28; i++)
	{ 
		params.A_15[i] = (double) A[j++]; 
	}

	j=392; 
	for( i=0; i<28; i++)
	{ 
		params.A_16[i] = (double) A[j++]; 
	}

	j=420; 
	for( i=0; i<28; i++)
	{ 
		params.A_17[i] = (double) A[j++]; 
	}

	j=448; 
	for( i=0; i<28; i++)
	{ 
		params.A_18[i] = (double) A[j++]; 
	}

	j=476; 
	for( i=0; i<28; i++)
	{ 
		params.A_19[i] = (double) A[j++]; 
	}

	j=504; 
	for( i=0; i<28; i++)
	{ 
		params.A_20[i] = (double) A[j++]; 
	}

	j=532; 
	for( i=0; i<28; i++)
	{ 
		params.A_21[i] = (double) A[j++]; 
	}

	j=560; 
	for( i=0; i<28; i++)
	{ 
		params.A_22[i] = (double) A[j++]; 
	}

	j=588; 
	for( i=0; i<28; i++)
	{ 
		params.A_23[i] = (double) A[j++]; 
	}

	j=616; 
	for( i=0; i<28; i++)
	{ 
		params.A_24[i] = (double) A[j++]; 
	}

	j=644; 
	for( i=0; i<28; i++)
	{ 
		params.A_25[i] = (double) A[j++]; 
	}

	j=672; 
	for( i=0; i<28; i++)
	{ 
		params.A_26[i] = (double) A[j++]; 
	}

	j=700; 
	for( i=0; i<28; i++)
	{ 
		params.A_27[i] = (double) A[j++]; 
	}

	j=728; 
	for( i=0; i<28; i++)
	{ 
		params.A_28[i] = (double) A[j++]; 
	}

	j=756; 
	for( i=0; i<28; i++)
	{ 
		params.A_29[i] = (double) A[j++]; 
	}

	j=784; 
	for( i=0; i<28; i++)
	{ 
		params.A_30[i] = (double) A[j++]; 
	}

	j=812; 
	for( i=0; i<28; i++)
	{ 
		params.A_31[i] = (double) A[j++]; 
	}

	j=840; 
	for( i=0; i<28; i++)
	{ 
		params.A_32[i] = (double) A[j++]; 
	}

	j=868; 
	for( i=0; i<28; i++)
	{ 
		params.A_33[i] = (double) A[j++]; 
	}

	j=896; 
	for( i=0; i<28; i++)
	{ 
		params.A_34[i] = (double) A[j++]; 
	}

	j=924; 
	for( i=0; i<28; i++)
	{ 
		params.A_35[i] = (double) A[j++]; 
	}

	j=952; 
	for( i=0; i<28; i++)
	{ 
		params.A_36[i] = (double) A[j++]; 
	}

	j=980; 
	for( i=0; i<28; i++)
	{ 
		params.A_37[i] = (double) A[j++]; 
	}

	j=1008; 
	for( i=0; i<28; i++)
	{ 
		params.A_38[i] = (double) A[j++]; 
	}

	j=1036; 
	for( i=0; i<28; i++)
	{ 
		params.A_39[i] = (double) A[j++]; 
	}

	j=1064; 
	for( i=0; i<28; i++)
	{ 
		params.A_40[i] = (double) A[j++]; 
	}

	j=1092; 
	for( i=0; i<28; i++)
	{ 
		params.A_41[i] = (double) A[j++]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_2[i] = (double) b[i]; 
	}

	j=2; 
	for( i=0; i<2; i++)
	{ 
		params.b_3[i] = (double) b[j++]; 
	}

	j=4; 
	for( i=0; i<2; i++)
	{ 
		params.b_4[i] = (double) b[j++]; 
	}

	j=6; 
	for( i=0; i<2; i++)
	{ 
		params.b_5[i] = (double) b[j++]; 
	}

	j=8; 
	for( i=0; i<2; i++)
	{ 
		params.b_6[i] = (double) b[j++]; 
	}

	j=10; 
	for( i=0; i<2; i++)
	{ 
		params.b_7[i] = (double) b[j++]; 
	}

	j=12; 
	for( i=0; i<2; i++)
	{ 
		params.b_8[i] = (double) b[j++]; 
	}

	j=14; 
	for( i=0; i<2; i++)
	{ 
		params.b_9[i] = (double) b[j++]; 
	}

	j=16; 
	for( i=0; i<2; i++)
	{ 
		params.b_10[i] = (double) b[j++]; 
	}

	j=18; 
	for( i=0; i<2; i++)
	{ 
		params.b_11[i] = (double) b[j++]; 
	}

	j=20; 
	for( i=0; i<2; i++)
	{ 
		params.b_12[i] = (double) b[j++]; 
	}

	j=22; 
	for( i=0; i<2; i++)
	{ 
		params.b_13[i] = (double) b[j++]; 
	}

	j=24; 
	for( i=0; i<2; i++)
	{ 
		params.b_14[i] = (double) b[j++]; 
	}

	j=26; 
	for( i=0; i<2; i++)
	{ 
		params.b_15[i] = (double) b[j++]; 
	}

	j=28; 
	for( i=0; i<2; i++)
	{ 
		params.b_16[i] = (double) b[j++]; 
	}

	j=30; 
	for( i=0; i<2; i++)
	{ 
		params.b_17[i] = (double) b[j++]; 
	}

	j=32; 
	for( i=0; i<2; i++)
	{ 
		params.b_18[i] = (double) b[j++]; 
	}

	j=34; 
	for( i=0; i<2; i++)
	{ 
		params.b_19[i] = (double) b[j++]; 
	}

	j=36; 
	for( i=0; i<2; i++)
	{ 
		params.b_20[i] = (double) b[j++]; 
	}

	j=38; 
	for( i=0; i<2; i++)
	{ 
		params.b_21[i] = (double) b[j++]; 
	}

	j=40; 
	for( i=0; i<2; i++)
	{ 
		params.b_22[i] = (double) b[j++]; 
	}

	j=42; 
	for( i=0; i<2; i++)
	{ 
		params.b_23[i] = (double) b[j++]; 
	}

	j=44; 
	for( i=0; i<2; i++)
	{ 
		params.b_24[i] = (double) b[j++]; 
	}

	j=46; 
	for( i=0; i<2; i++)
	{ 
		params.b_25[i] = (double) b[j++]; 
	}

	j=48; 
	for( i=0; i<2; i++)
	{ 
		params.b_26[i] = (double) b[j++]; 
	}

	j=50; 
	for( i=0; i<2; i++)
	{ 
		params.b_27[i] = (double) b[j++]; 
	}

	j=52; 
	for( i=0; i<2; i++)
	{ 
		params.b_28[i] = (double) b[j++]; 
	}

	j=54; 
	for( i=0; i<2; i++)
	{ 
		params.b_29[i] = (double) b[j++]; 
	}

	j=56; 
	for( i=0; i<2; i++)
	{ 
		params.b_30[i] = (double) b[j++]; 
	}

	j=58; 
	for( i=0; i<2; i++)
	{ 
		params.b_31[i] = (double) b[j++]; 
	}

	j=60; 
	for( i=0; i<2; i++)
	{ 
		params.b_32[i] = (double) b[j++]; 
	}

	j=62; 
	for( i=0; i<2; i++)
	{ 
		params.b_33[i] = (double) b[j++]; 
	}

	j=64; 
	for( i=0; i<2; i++)
	{ 
		params.b_34[i] = (double) b[j++]; 
	}

	j=66; 
	for( i=0; i<2; i++)
	{ 
		params.b_35[i] = (double) b[j++]; 
	}

	j=68; 
	for( i=0; i<2; i++)
	{ 
		params.b_36[i] = (double) b[j++]; 
	}

	j=70; 
	for( i=0; i<2; i++)
	{ 
		params.b_37[i] = (double) b[j++]; 
	}

	j=72; 
	for( i=0; i<2; i++)
	{ 
		params.b_38[i] = (double) b[j++]; 
	}

	j=74; 
	for( i=0; i<2; i++)
	{ 
		params.b_39[i] = (double) b[j++]; 
	}

	j=76; 
	for( i=0; i<2; i++)
	{ 
		params.b_40[i] = (double) b[j++]; 
	}

	j=78; 
	for( i=0; i<2; i++)
	{ 
		params.b_41[i] = (double) b[j++]; 
	}

	

	

    #if SET_PRINTLEVEL_MPCC_solv_40N_no_warm_no_hard_invitedguest > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = MPCC_solv_40N_no_warm_no_hard_invitedguest_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPCC_solv_40N_no_warm_no_hard_invitedguest > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<492; i++)
	{ 
		outputs[i] = (real_T) output.X[i]; 
	}

	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



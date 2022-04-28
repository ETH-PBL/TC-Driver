/*
MPCC_solv_80N_no_warm_no_hard_invitedguest : A fast customized optimization solver.

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
#define S_FUNCTION_NAME MPCC_solv_80N_no_warm_no_hard_invitedguest_simulinkBlockcompact

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/MPCC_solv_80N_no_warm_no_hard_invitedguest.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef MPCC_solv_80N_no_warm_no_hard_invitedguestinterface_float MPCC_solv_80N_no_warm_no_hard_invitedguestnmpc_float;





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
    ssSetInputPortMatrixDimensions(S,  0, 10, 80);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 10, 1120);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 14, 1134);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 14, 81);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 2, 80);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 2, 1120);
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
    ssSetOutputPortMatrixDimensions(S,  0, 972, 1);
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
	static MPCC_solv_80N_no_warm_no_hard_invitedguest_params params;
	static MPCC_solv_80N_no_warm_no_hard_invitedguest_output output;
	static MPCC_solv_80N_no_warm_no_hard_invitedguest_info info;	
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

	j=400; 
	for( i=0; i<10; i++)
	{ 
		params.c_42[i] = (double) c[j++]; 
	}

	j=410; 
	for( i=0; i<10; i++)
	{ 
		params.c_43[i] = (double) c[j++]; 
	}

	j=420; 
	for( i=0; i<10; i++)
	{ 
		params.c_44[i] = (double) c[j++]; 
	}

	j=430; 
	for( i=0; i<10; i++)
	{ 
		params.c_45[i] = (double) c[j++]; 
	}

	j=440; 
	for( i=0; i<10; i++)
	{ 
		params.c_46[i] = (double) c[j++]; 
	}

	j=450; 
	for( i=0; i<10; i++)
	{ 
		params.c_47[i] = (double) c[j++]; 
	}

	j=460; 
	for( i=0; i<10; i++)
	{ 
		params.c_48[i] = (double) c[j++]; 
	}

	j=470; 
	for( i=0; i<10; i++)
	{ 
		params.c_49[i] = (double) c[j++]; 
	}

	j=480; 
	for( i=0; i<10; i++)
	{ 
		params.c_50[i] = (double) c[j++]; 
	}

	j=490; 
	for( i=0; i<10; i++)
	{ 
		params.c_51[i] = (double) c[j++]; 
	}

	j=500; 
	for( i=0; i<10; i++)
	{ 
		params.c_52[i] = (double) c[j++]; 
	}

	j=510; 
	for( i=0; i<10; i++)
	{ 
		params.c_53[i] = (double) c[j++]; 
	}

	j=520; 
	for( i=0; i<10; i++)
	{ 
		params.c_54[i] = (double) c[j++]; 
	}

	j=530; 
	for( i=0; i<10; i++)
	{ 
		params.c_55[i] = (double) c[j++]; 
	}

	j=540; 
	for( i=0; i<10; i++)
	{ 
		params.c_56[i] = (double) c[j++]; 
	}

	j=550; 
	for( i=0; i<10; i++)
	{ 
		params.c_57[i] = (double) c[j++]; 
	}

	j=560; 
	for( i=0; i<10; i++)
	{ 
		params.c_58[i] = (double) c[j++]; 
	}

	j=570; 
	for( i=0; i<10; i++)
	{ 
		params.c_59[i] = (double) c[j++]; 
	}

	j=580; 
	for( i=0; i<10; i++)
	{ 
		params.c_60[i] = (double) c[j++]; 
	}

	j=590; 
	for( i=0; i<10; i++)
	{ 
		params.c_61[i] = (double) c[j++]; 
	}

	j=600; 
	for( i=0; i<10; i++)
	{ 
		params.c_62[i] = (double) c[j++]; 
	}

	j=610; 
	for( i=0; i<10; i++)
	{ 
		params.c_63[i] = (double) c[j++]; 
	}

	j=620; 
	for( i=0; i<10; i++)
	{ 
		params.c_64[i] = (double) c[j++]; 
	}

	j=630; 
	for( i=0; i<10; i++)
	{ 
		params.c_65[i] = (double) c[j++]; 
	}

	j=640; 
	for( i=0; i<10; i++)
	{ 
		params.c_66[i] = (double) c[j++]; 
	}

	j=650; 
	for( i=0; i<10; i++)
	{ 
		params.c_67[i] = (double) c[j++]; 
	}

	j=660; 
	for( i=0; i<10; i++)
	{ 
		params.c_68[i] = (double) c[j++]; 
	}

	j=670; 
	for( i=0; i<10; i++)
	{ 
		params.c_69[i] = (double) c[j++]; 
	}

	j=680; 
	for( i=0; i<10; i++)
	{ 
		params.c_70[i] = (double) c[j++]; 
	}

	j=690; 
	for( i=0; i<10; i++)
	{ 
		params.c_71[i] = (double) c[j++]; 
	}

	j=700; 
	for( i=0; i<10; i++)
	{ 
		params.c_72[i] = (double) c[j++]; 
	}

	j=710; 
	for( i=0; i<10; i++)
	{ 
		params.c_73[i] = (double) c[j++]; 
	}

	j=720; 
	for( i=0; i<10; i++)
	{ 
		params.c_74[i] = (double) c[j++]; 
	}

	j=730; 
	for( i=0; i<10; i++)
	{ 
		params.c_75[i] = (double) c[j++]; 
	}

	j=740; 
	for( i=0; i<10; i++)
	{ 
		params.c_76[i] = (double) c[j++]; 
	}

	j=750; 
	for( i=0; i<10; i++)
	{ 
		params.c_77[i] = (double) c[j++]; 
	}

	j=760; 
	for( i=0; i<10; i++)
	{ 
		params.c_78[i] = (double) c[j++]; 
	}

	j=770; 
	for( i=0; i<10; i++)
	{ 
		params.c_79[i] = (double) c[j++]; 
	}

	j=780; 
	for( i=0; i<10; i++)
	{ 
		params.c_80[i] = (double) c[j++]; 
	}

	j=790; 
	for( i=0; i<10; i++)
	{ 
		params.c_81[i] = (double) c[j++]; 
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

	j=8036; 
	for( i=0; i<196; i++)
	{ 
		params.H_42[i] = (double) H[j++]; 
	}

	j=8232; 
	for( i=0; i<196; i++)
	{ 
		params.H_43[i] = (double) H[j++]; 
	}

	j=8428; 
	for( i=0; i<196; i++)
	{ 
		params.H_44[i] = (double) H[j++]; 
	}

	j=8624; 
	for( i=0; i<196; i++)
	{ 
		params.H_45[i] = (double) H[j++]; 
	}

	j=8820; 
	for( i=0; i<196; i++)
	{ 
		params.H_46[i] = (double) H[j++]; 
	}

	j=9016; 
	for( i=0; i<196; i++)
	{ 
		params.H_47[i] = (double) H[j++]; 
	}

	j=9212; 
	for( i=0; i<196; i++)
	{ 
		params.H_48[i] = (double) H[j++]; 
	}

	j=9408; 
	for( i=0; i<196; i++)
	{ 
		params.H_49[i] = (double) H[j++]; 
	}

	j=9604; 
	for( i=0; i<196; i++)
	{ 
		params.H_50[i] = (double) H[j++]; 
	}

	j=9800; 
	for( i=0; i<196; i++)
	{ 
		params.H_51[i] = (double) H[j++]; 
	}

	j=9996; 
	for( i=0; i<196; i++)
	{ 
		params.H_52[i] = (double) H[j++]; 
	}

	j=10192; 
	for( i=0; i<196; i++)
	{ 
		params.H_53[i] = (double) H[j++]; 
	}

	j=10388; 
	for( i=0; i<196; i++)
	{ 
		params.H_54[i] = (double) H[j++]; 
	}

	j=10584; 
	for( i=0; i<196; i++)
	{ 
		params.H_55[i] = (double) H[j++]; 
	}

	j=10780; 
	for( i=0; i<196; i++)
	{ 
		params.H_56[i] = (double) H[j++]; 
	}

	j=10976; 
	for( i=0; i<196; i++)
	{ 
		params.H_57[i] = (double) H[j++]; 
	}

	j=11172; 
	for( i=0; i<196; i++)
	{ 
		params.H_58[i] = (double) H[j++]; 
	}

	j=11368; 
	for( i=0; i<196; i++)
	{ 
		params.H_59[i] = (double) H[j++]; 
	}

	j=11564; 
	for( i=0; i<196; i++)
	{ 
		params.H_60[i] = (double) H[j++]; 
	}

	j=11760; 
	for( i=0; i<196; i++)
	{ 
		params.H_61[i] = (double) H[j++]; 
	}

	j=11956; 
	for( i=0; i<196; i++)
	{ 
		params.H_62[i] = (double) H[j++]; 
	}

	j=12152; 
	for( i=0; i<196; i++)
	{ 
		params.H_63[i] = (double) H[j++]; 
	}

	j=12348; 
	for( i=0; i<196; i++)
	{ 
		params.H_64[i] = (double) H[j++]; 
	}

	j=12544; 
	for( i=0; i<196; i++)
	{ 
		params.H_65[i] = (double) H[j++]; 
	}

	j=12740; 
	for( i=0; i<196; i++)
	{ 
		params.H_66[i] = (double) H[j++]; 
	}

	j=12936; 
	for( i=0; i<196; i++)
	{ 
		params.H_67[i] = (double) H[j++]; 
	}

	j=13132; 
	for( i=0; i<196; i++)
	{ 
		params.H_68[i] = (double) H[j++]; 
	}

	j=13328; 
	for( i=0; i<196; i++)
	{ 
		params.H_69[i] = (double) H[j++]; 
	}

	j=13524; 
	for( i=0; i<196; i++)
	{ 
		params.H_70[i] = (double) H[j++]; 
	}

	j=13720; 
	for( i=0; i<196; i++)
	{ 
		params.H_71[i] = (double) H[j++]; 
	}

	j=13916; 
	for( i=0; i<196; i++)
	{ 
		params.H_72[i] = (double) H[j++]; 
	}

	j=14112; 
	for( i=0; i<196; i++)
	{ 
		params.H_73[i] = (double) H[j++]; 
	}

	j=14308; 
	for( i=0; i<196; i++)
	{ 
		params.H_74[i] = (double) H[j++]; 
	}

	j=14504; 
	for( i=0; i<196; i++)
	{ 
		params.H_75[i] = (double) H[j++]; 
	}

	j=14700; 
	for( i=0; i<196; i++)
	{ 
		params.H_76[i] = (double) H[j++]; 
	}

	j=14896; 
	for( i=0; i<196; i++)
	{ 
		params.H_77[i] = (double) H[j++]; 
	}

	j=15092; 
	for( i=0; i<196; i++)
	{ 
		params.H_78[i] = (double) H[j++]; 
	}

	j=15288; 
	for( i=0; i<196; i++)
	{ 
		params.H_79[i] = (double) H[j++]; 
	}

	j=15484; 
	for( i=0; i<196; i++)
	{ 
		params.H_80[i] = (double) H[j++]; 
	}

	j=15680; 
	for( i=0; i<196; i++)
	{ 
		params.H_81[i] = (double) H[j++]; 
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

	j=574; 
	for( i=0; i<14; i++)
	{ 
		params.f_42[i] = (double) f[j++]; 
	}

	j=588; 
	for( i=0; i<14; i++)
	{ 
		params.f_43[i] = (double) f[j++]; 
	}

	j=602; 
	for( i=0; i<14; i++)
	{ 
		params.f_44[i] = (double) f[j++]; 
	}

	j=616; 
	for( i=0; i<14; i++)
	{ 
		params.f_45[i] = (double) f[j++]; 
	}

	j=630; 
	for( i=0; i<14; i++)
	{ 
		params.f_46[i] = (double) f[j++]; 
	}

	j=644; 
	for( i=0; i<14; i++)
	{ 
		params.f_47[i] = (double) f[j++]; 
	}

	j=658; 
	for( i=0; i<14; i++)
	{ 
		params.f_48[i] = (double) f[j++]; 
	}

	j=672; 
	for( i=0; i<14; i++)
	{ 
		params.f_49[i] = (double) f[j++]; 
	}

	j=686; 
	for( i=0; i<14; i++)
	{ 
		params.f_50[i] = (double) f[j++]; 
	}

	j=700; 
	for( i=0; i<14; i++)
	{ 
		params.f_51[i] = (double) f[j++]; 
	}

	j=714; 
	for( i=0; i<14; i++)
	{ 
		params.f_52[i] = (double) f[j++]; 
	}

	j=728; 
	for( i=0; i<14; i++)
	{ 
		params.f_53[i] = (double) f[j++]; 
	}

	j=742; 
	for( i=0; i<14; i++)
	{ 
		params.f_54[i] = (double) f[j++]; 
	}

	j=756; 
	for( i=0; i<14; i++)
	{ 
		params.f_55[i] = (double) f[j++]; 
	}

	j=770; 
	for( i=0; i<14; i++)
	{ 
		params.f_56[i] = (double) f[j++]; 
	}

	j=784; 
	for( i=0; i<14; i++)
	{ 
		params.f_57[i] = (double) f[j++]; 
	}

	j=798; 
	for( i=0; i<14; i++)
	{ 
		params.f_58[i] = (double) f[j++]; 
	}

	j=812; 
	for( i=0; i<14; i++)
	{ 
		params.f_59[i] = (double) f[j++]; 
	}

	j=826; 
	for( i=0; i<14; i++)
	{ 
		params.f_60[i] = (double) f[j++]; 
	}

	j=840; 
	for( i=0; i<14; i++)
	{ 
		params.f_61[i] = (double) f[j++]; 
	}

	j=854; 
	for( i=0; i<14; i++)
	{ 
		params.f_62[i] = (double) f[j++]; 
	}

	j=868; 
	for( i=0; i<14; i++)
	{ 
		params.f_63[i] = (double) f[j++]; 
	}

	j=882; 
	for( i=0; i<14; i++)
	{ 
		params.f_64[i] = (double) f[j++]; 
	}

	j=896; 
	for( i=0; i<14; i++)
	{ 
		params.f_65[i] = (double) f[j++]; 
	}

	j=910; 
	for( i=0; i<14; i++)
	{ 
		params.f_66[i] = (double) f[j++]; 
	}

	j=924; 
	for( i=0; i<14; i++)
	{ 
		params.f_67[i] = (double) f[j++]; 
	}

	j=938; 
	for( i=0; i<14; i++)
	{ 
		params.f_68[i] = (double) f[j++]; 
	}

	j=952; 
	for( i=0; i<14; i++)
	{ 
		params.f_69[i] = (double) f[j++]; 
	}

	j=966; 
	for( i=0; i<14; i++)
	{ 
		params.f_70[i] = (double) f[j++]; 
	}

	j=980; 
	for( i=0; i<14; i++)
	{ 
		params.f_71[i] = (double) f[j++]; 
	}

	j=994; 
	for( i=0; i<14; i++)
	{ 
		params.f_72[i] = (double) f[j++]; 
	}

	j=1008; 
	for( i=0; i<14; i++)
	{ 
		params.f_73[i] = (double) f[j++]; 
	}

	j=1022; 
	for( i=0; i<14; i++)
	{ 
		params.f_74[i] = (double) f[j++]; 
	}

	j=1036; 
	for( i=0; i<14; i++)
	{ 
		params.f_75[i] = (double) f[j++]; 
	}

	j=1050; 
	for( i=0; i<14; i++)
	{ 
		params.f_76[i] = (double) f[j++]; 
	}

	j=1064; 
	for( i=0; i<14; i++)
	{ 
		params.f_77[i] = (double) f[j++]; 
	}

	j=1078; 
	for( i=0; i<14; i++)
	{ 
		params.f_78[i] = (double) f[j++]; 
	}

	j=1092; 
	for( i=0; i<14; i++)
	{ 
		params.f_79[i] = (double) f[j++]; 
	}

	j=1106; 
	for( i=0; i<14; i++)
	{ 
		params.f_80[i] = (double) f[j++]; 
	}

	j=1120; 
	for( i=0; i<14; i++)
	{ 
		params.f_81[i] = (double) f[j++]; 
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

	j=5600; 
	for( i=0; i<140; i++)
	{ 
		params.C_41[i] = (double) C[j++]; 
	}

	j=5740; 
	for( i=0; i<140; i++)
	{ 
		params.C_42[i] = (double) C[j++]; 
	}

	j=5880; 
	for( i=0; i<140; i++)
	{ 
		params.C_43[i] = (double) C[j++]; 
	}

	j=6020; 
	for( i=0; i<140; i++)
	{ 
		params.C_44[i] = (double) C[j++]; 
	}

	j=6160; 
	for( i=0; i<140; i++)
	{ 
		params.C_45[i] = (double) C[j++]; 
	}

	j=6300; 
	for( i=0; i<140; i++)
	{ 
		params.C_46[i] = (double) C[j++]; 
	}

	j=6440; 
	for( i=0; i<140; i++)
	{ 
		params.C_47[i] = (double) C[j++]; 
	}

	j=6580; 
	for( i=0; i<140; i++)
	{ 
		params.C_48[i] = (double) C[j++]; 
	}

	j=6720; 
	for( i=0; i<140; i++)
	{ 
		params.C_49[i] = (double) C[j++]; 
	}

	j=6860; 
	for( i=0; i<140; i++)
	{ 
		params.C_50[i] = (double) C[j++]; 
	}

	j=7000; 
	for( i=0; i<140; i++)
	{ 
		params.C_51[i] = (double) C[j++]; 
	}

	j=7140; 
	for( i=0; i<140; i++)
	{ 
		params.C_52[i] = (double) C[j++]; 
	}

	j=7280; 
	for( i=0; i<140; i++)
	{ 
		params.C_53[i] = (double) C[j++]; 
	}

	j=7420; 
	for( i=0; i<140; i++)
	{ 
		params.C_54[i] = (double) C[j++]; 
	}

	j=7560; 
	for( i=0; i<140; i++)
	{ 
		params.C_55[i] = (double) C[j++]; 
	}

	j=7700; 
	for( i=0; i<140; i++)
	{ 
		params.C_56[i] = (double) C[j++]; 
	}

	j=7840; 
	for( i=0; i<140; i++)
	{ 
		params.C_57[i] = (double) C[j++]; 
	}

	j=7980; 
	for( i=0; i<140; i++)
	{ 
		params.C_58[i] = (double) C[j++]; 
	}

	j=8120; 
	for( i=0; i<140; i++)
	{ 
		params.C_59[i] = (double) C[j++]; 
	}

	j=8260; 
	for( i=0; i<140; i++)
	{ 
		params.C_60[i] = (double) C[j++]; 
	}

	j=8400; 
	for( i=0; i<140; i++)
	{ 
		params.C_61[i] = (double) C[j++]; 
	}

	j=8540; 
	for( i=0; i<140; i++)
	{ 
		params.C_62[i] = (double) C[j++]; 
	}

	j=8680; 
	for( i=0; i<140; i++)
	{ 
		params.C_63[i] = (double) C[j++]; 
	}

	j=8820; 
	for( i=0; i<140; i++)
	{ 
		params.C_64[i] = (double) C[j++]; 
	}

	j=8960; 
	for( i=0; i<140; i++)
	{ 
		params.C_65[i] = (double) C[j++]; 
	}

	j=9100; 
	for( i=0; i<140; i++)
	{ 
		params.C_66[i] = (double) C[j++]; 
	}

	j=9240; 
	for( i=0; i<140; i++)
	{ 
		params.C_67[i] = (double) C[j++]; 
	}

	j=9380; 
	for( i=0; i<140; i++)
	{ 
		params.C_68[i] = (double) C[j++]; 
	}

	j=9520; 
	for( i=0; i<140; i++)
	{ 
		params.C_69[i] = (double) C[j++]; 
	}

	j=9660; 
	for( i=0; i<140; i++)
	{ 
		params.C_70[i] = (double) C[j++]; 
	}

	j=9800; 
	for( i=0; i<140; i++)
	{ 
		params.C_71[i] = (double) C[j++]; 
	}

	j=9940; 
	for( i=0; i<140; i++)
	{ 
		params.C_72[i] = (double) C[j++]; 
	}

	j=10080; 
	for( i=0; i<140; i++)
	{ 
		params.C_73[i] = (double) C[j++]; 
	}

	j=10220; 
	for( i=0; i<140; i++)
	{ 
		params.C_74[i] = (double) C[j++]; 
	}

	j=10360; 
	for( i=0; i<140; i++)
	{ 
		params.C_75[i] = (double) C[j++]; 
	}

	j=10500; 
	for( i=0; i<140; i++)
	{ 
		params.C_76[i] = (double) C[j++]; 
	}

	j=10640; 
	for( i=0; i<140; i++)
	{ 
		params.C_77[i] = (double) C[j++]; 
	}

	j=10780; 
	for( i=0; i<140; i++)
	{ 
		params.C_78[i] = (double) C[j++]; 
	}

	j=10920; 
	for( i=0; i<140; i++)
	{ 
		params.C_79[i] = (double) C[j++]; 
	}

	j=11060; 
	for( i=0; i<140; i++)
	{ 
		params.C_80[i] = (double) C[j++]; 
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

	j=1120; 
	for( i=0; i<28; i++)
	{ 
		params.A_42[i] = (double) A[j++]; 
	}

	j=1148; 
	for( i=0; i<28; i++)
	{ 
		params.A_43[i] = (double) A[j++]; 
	}

	j=1176; 
	for( i=0; i<28; i++)
	{ 
		params.A_44[i] = (double) A[j++]; 
	}

	j=1204; 
	for( i=0; i<28; i++)
	{ 
		params.A_45[i] = (double) A[j++]; 
	}

	j=1232; 
	for( i=0; i<28; i++)
	{ 
		params.A_46[i] = (double) A[j++]; 
	}

	j=1260; 
	for( i=0; i<28; i++)
	{ 
		params.A_47[i] = (double) A[j++]; 
	}

	j=1288; 
	for( i=0; i<28; i++)
	{ 
		params.A_48[i] = (double) A[j++]; 
	}

	j=1316; 
	for( i=0; i<28; i++)
	{ 
		params.A_49[i] = (double) A[j++]; 
	}

	j=1344; 
	for( i=0; i<28; i++)
	{ 
		params.A_50[i] = (double) A[j++]; 
	}

	j=1372; 
	for( i=0; i<28; i++)
	{ 
		params.A_51[i] = (double) A[j++]; 
	}

	j=1400; 
	for( i=0; i<28; i++)
	{ 
		params.A_52[i] = (double) A[j++]; 
	}

	j=1428; 
	for( i=0; i<28; i++)
	{ 
		params.A_53[i] = (double) A[j++]; 
	}

	j=1456; 
	for( i=0; i<28; i++)
	{ 
		params.A_54[i] = (double) A[j++]; 
	}

	j=1484; 
	for( i=0; i<28; i++)
	{ 
		params.A_55[i] = (double) A[j++]; 
	}

	j=1512; 
	for( i=0; i<28; i++)
	{ 
		params.A_56[i] = (double) A[j++]; 
	}

	j=1540; 
	for( i=0; i<28; i++)
	{ 
		params.A_57[i] = (double) A[j++]; 
	}

	j=1568; 
	for( i=0; i<28; i++)
	{ 
		params.A_58[i] = (double) A[j++]; 
	}

	j=1596; 
	for( i=0; i<28; i++)
	{ 
		params.A_59[i] = (double) A[j++]; 
	}

	j=1624; 
	for( i=0; i<28; i++)
	{ 
		params.A_60[i] = (double) A[j++]; 
	}

	j=1652; 
	for( i=0; i<28; i++)
	{ 
		params.A_61[i] = (double) A[j++]; 
	}

	j=1680; 
	for( i=0; i<28; i++)
	{ 
		params.A_62[i] = (double) A[j++]; 
	}

	j=1708; 
	for( i=0; i<28; i++)
	{ 
		params.A_63[i] = (double) A[j++]; 
	}

	j=1736; 
	for( i=0; i<28; i++)
	{ 
		params.A_64[i] = (double) A[j++]; 
	}

	j=1764; 
	for( i=0; i<28; i++)
	{ 
		params.A_65[i] = (double) A[j++]; 
	}

	j=1792; 
	for( i=0; i<28; i++)
	{ 
		params.A_66[i] = (double) A[j++]; 
	}

	j=1820; 
	for( i=0; i<28; i++)
	{ 
		params.A_67[i] = (double) A[j++]; 
	}

	j=1848; 
	for( i=0; i<28; i++)
	{ 
		params.A_68[i] = (double) A[j++]; 
	}

	j=1876; 
	for( i=0; i<28; i++)
	{ 
		params.A_69[i] = (double) A[j++]; 
	}

	j=1904; 
	for( i=0; i<28; i++)
	{ 
		params.A_70[i] = (double) A[j++]; 
	}

	j=1932; 
	for( i=0; i<28; i++)
	{ 
		params.A_71[i] = (double) A[j++]; 
	}

	j=1960; 
	for( i=0; i<28; i++)
	{ 
		params.A_72[i] = (double) A[j++]; 
	}

	j=1988; 
	for( i=0; i<28; i++)
	{ 
		params.A_73[i] = (double) A[j++]; 
	}

	j=2016; 
	for( i=0; i<28; i++)
	{ 
		params.A_74[i] = (double) A[j++]; 
	}

	j=2044; 
	for( i=0; i<28; i++)
	{ 
		params.A_75[i] = (double) A[j++]; 
	}

	j=2072; 
	for( i=0; i<28; i++)
	{ 
		params.A_76[i] = (double) A[j++]; 
	}

	j=2100; 
	for( i=0; i<28; i++)
	{ 
		params.A_77[i] = (double) A[j++]; 
	}

	j=2128; 
	for( i=0; i<28; i++)
	{ 
		params.A_78[i] = (double) A[j++]; 
	}

	j=2156; 
	for( i=0; i<28; i++)
	{ 
		params.A_79[i] = (double) A[j++]; 
	}

	j=2184; 
	for( i=0; i<28; i++)
	{ 
		params.A_80[i] = (double) A[j++]; 
	}

	j=2212; 
	for( i=0; i<28; i++)
	{ 
		params.A_81[i] = (double) A[j++]; 
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

	j=80; 
	for( i=0; i<2; i++)
	{ 
		params.b_42[i] = (double) b[j++]; 
	}

	j=82; 
	for( i=0; i<2; i++)
	{ 
		params.b_43[i] = (double) b[j++]; 
	}

	j=84; 
	for( i=0; i<2; i++)
	{ 
		params.b_44[i] = (double) b[j++]; 
	}

	j=86; 
	for( i=0; i<2; i++)
	{ 
		params.b_45[i] = (double) b[j++]; 
	}

	j=88; 
	for( i=0; i<2; i++)
	{ 
		params.b_46[i] = (double) b[j++]; 
	}

	j=90; 
	for( i=0; i<2; i++)
	{ 
		params.b_47[i] = (double) b[j++]; 
	}

	j=92; 
	for( i=0; i<2; i++)
	{ 
		params.b_48[i] = (double) b[j++]; 
	}

	j=94; 
	for( i=0; i<2; i++)
	{ 
		params.b_49[i] = (double) b[j++]; 
	}

	j=96; 
	for( i=0; i<2; i++)
	{ 
		params.b_50[i] = (double) b[j++]; 
	}

	j=98; 
	for( i=0; i<2; i++)
	{ 
		params.b_51[i] = (double) b[j++]; 
	}

	j=100; 
	for( i=0; i<2; i++)
	{ 
		params.b_52[i] = (double) b[j++]; 
	}

	j=102; 
	for( i=0; i<2; i++)
	{ 
		params.b_53[i] = (double) b[j++]; 
	}

	j=104; 
	for( i=0; i<2; i++)
	{ 
		params.b_54[i] = (double) b[j++]; 
	}

	j=106; 
	for( i=0; i<2; i++)
	{ 
		params.b_55[i] = (double) b[j++]; 
	}

	j=108; 
	for( i=0; i<2; i++)
	{ 
		params.b_56[i] = (double) b[j++]; 
	}

	j=110; 
	for( i=0; i<2; i++)
	{ 
		params.b_57[i] = (double) b[j++]; 
	}

	j=112; 
	for( i=0; i<2; i++)
	{ 
		params.b_58[i] = (double) b[j++]; 
	}

	j=114; 
	for( i=0; i<2; i++)
	{ 
		params.b_59[i] = (double) b[j++]; 
	}

	j=116; 
	for( i=0; i<2; i++)
	{ 
		params.b_60[i] = (double) b[j++]; 
	}

	j=118; 
	for( i=0; i<2; i++)
	{ 
		params.b_61[i] = (double) b[j++]; 
	}

	j=120; 
	for( i=0; i<2; i++)
	{ 
		params.b_62[i] = (double) b[j++]; 
	}

	j=122; 
	for( i=0; i<2; i++)
	{ 
		params.b_63[i] = (double) b[j++]; 
	}

	j=124; 
	for( i=0; i<2; i++)
	{ 
		params.b_64[i] = (double) b[j++]; 
	}

	j=126; 
	for( i=0; i<2; i++)
	{ 
		params.b_65[i] = (double) b[j++]; 
	}

	j=128; 
	for( i=0; i<2; i++)
	{ 
		params.b_66[i] = (double) b[j++]; 
	}

	j=130; 
	for( i=0; i<2; i++)
	{ 
		params.b_67[i] = (double) b[j++]; 
	}

	j=132; 
	for( i=0; i<2; i++)
	{ 
		params.b_68[i] = (double) b[j++]; 
	}

	j=134; 
	for( i=0; i<2; i++)
	{ 
		params.b_69[i] = (double) b[j++]; 
	}

	j=136; 
	for( i=0; i<2; i++)
	{ 
		params.b_70[i] = (double) b[j++]; 
	}

	j=138; 
	for( i=0; i<2; i++)
	{ 
		params.b_71[i] = (double) b[j++]; 
	}

	j=140; 
	for( i=0; i<2; i++)
	{ 
		params.b_72[i] = (double) b[j++]; 
	}

	j=142; 
	for( i=0; i<2; i++)
	{ 
		params.b_73[i] = (double) b[j++]; 
	}

	j=144; 
	for( i=0; i<2; i++)
	{ 
		params.b_74[i] = (double) b[j++]; 
	}

	j=146; 
	for( i=0; i<2; i++)
	{ 
		params.b_75[i] = (double) b[j++]; 
	}

	j=148; 
	for( i=0; i<2; i++)
	{ 
		params.b_76[i] = (double) b[j++]; 
	}

	j=150; 
	for( i=0; i<2; i++)
	{ 
		params.b_77[i] = (double) b[j++]; 
	}

	j=152; 
	for( i=0; i<2; i++)
	{ 
		params.b_78[i] = (double) b[j++]; 
	}

	j=154; 
	for( i=0; i<2; i++)
	{ 
		params.b_79[i] = (double) b[j++]; 
	}

	j=156; 
	for( i=0; i<2; i++)
	{ 
		params.b_80[i] = (double) b[j++]; 
	}

	j=158; 
	for( i=0; i<2; i++)
	{ 
		params.b_81[i] = (double) b[j++]; 
	}

	

	

    #if SET_PRINTLEVEL_MPCC_solv_80N_no_warm_no_hard_invitedguest > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = MPCC_solv_80N_no_warm_no_hard_invitedguest_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPCC_solv_80N_no_warm_no_hard_invitedguest > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<972; i++)
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



/*
MPCC_solv_10N_no_warm_no_hard_gnone : A fast customized optimization solver.

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
#define S_FUNCTION_NAME MPCC_solv_10N_no_warm_no_hard_gnone_simulinkBlockcompact

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/MPCC_solv_10N_no_warm_no_hard_gnone.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef MPCC_solv_10N_no_warm_no_hard_gnoneinterface_float MPCC_solv_10N_no_warm_no_hard_gnonenmpc_float;





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
    ssSetInputPortMatrixDimensions(S,  0, 10, 10);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 10, 140);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 14, 154);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 14, 11);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 2, 10);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 2, 140);
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
    ssSetOutputPortMatrixDimensions(S,  0, 132, 1);
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
	static MPCC_solv_10N_no_warm_no_hard_gnone_params params;
	static MPCC_solv_10N_no_warm_no_hard_gnone_output output;
	static MPCC_solv_10N_no_warm_no_hard_gnone_info info;	
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

	

	

    #if SET_PRINTLEVEL_MPCC_solv_10N_no_warm_no_hard_gnone > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = MPCC_solv_10N_no_warm_no_hard_gnone_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPCC_solv_10N_no_warm_no_hard_gnone > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<132; i++)
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



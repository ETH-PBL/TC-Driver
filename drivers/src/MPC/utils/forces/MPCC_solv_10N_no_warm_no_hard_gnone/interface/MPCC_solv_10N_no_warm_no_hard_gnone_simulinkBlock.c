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
#define S_FUNCTION_NAME MPCC_solv_10N_no_warm_no_hard_gnone_simulinkBlock

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

	/* initialize input ports - there are 63 in total */
    if (!ssSetNumInputPorts(S, 63)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 8, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 10, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 10, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 10, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 10, 1);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 10, 1);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/
	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 10, 1);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/
	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 10, 1);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/
	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 10, 1);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/
	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 10, 1);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/
	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 10, 1);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 10, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/
	
	/* Input Port 11 */
    ssSetInputPortMatrixDimensions(S,  11, 14, 14);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/
	
	/* Input Port 12 */
    ssSetInputPortMatrixDimensions(S,  12, 14, 14);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 12, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/
	
	/* Input Port 13 */
    ssSetInputPortMatrixDimensions(S,  13, 14, 14);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 13, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/
	
	/* Input Port 14 */
    ssSetInputPortMatrixDimensions(S,  14, 14, 14);
    ssSetInputPortDataType(S, 14, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 14, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 14, 1); /*direct input signal access*/
	
	/* Input Port 15 */
    ssSetInputPortMatrixDimensions(S,  15, 14, 14);
    ssSetInputPortDataType(S, 15, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 15, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 15, 1); /*direct input signal access*/
	
	/* Input Port 16 */
    ssSetInputPortMatrixDimensions(S,  16, 14, 14);
    ssSetInputPortDataType(S, 16, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 16, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 16, 1); /*direct input signal access*/
	
	/* Input Port 17 */
    ssSetInputPortMatrixDimensions(S,  17, 14, 14);
    ssSetInputPortDataType(S, 17, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 17, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 17, 1); /*direct input signal access*/
	
	/* Input Port 18 */
    ssSetInputPortMatrixDimensions(S,  18, 14, 14);
    ssSetInputPortDataType(S, 18, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 18, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 18, 1); /*direct input signal access*/
	
	/* Input Port 19 */
    ssSetInputPortMatrixDimensions(S,  19, 14, 14);
    ssSetInputPortDataType(S, 19, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 19, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 19, 1); /*direct input signal access*/
	
	/* Input Port 20 */
    ssSetInputPortMatrixDimensions(S,  20, 14, 14);
    ssSetInputPortDataType(S, 20, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 20, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 20, 1); /*direct input signal access*/
	
	/* Input Port 21 */
    ssSetInputPortMatrixDimensions(S,  21, 14, 14);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/
	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 14, 1);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/
	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 14, 1);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/
	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 14, 1);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/
	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 14, 1);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/
	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 14, 1);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/
	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 14, 1);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/
	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 14, 1);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/
	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 14, 1);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/
	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 14, 1);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/
	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 14, 1);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/
	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 14, 1);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/
	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 10, 14);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/
	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 10, 14);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/
	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 10, 14);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/
	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 10, 14);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/
	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 10, 14);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/
	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 10, 14);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/
	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 10, 14);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/
	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 10, 14);
    ssSetInputPortDataType(S, 40, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 40, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 40, 1); /*direct input signal access*/
	
	/* Input Port 41 */
    ssSetInputPortMatrixDimensions(S,  41, 10, 14);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/
	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 10, 14);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/
	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 2, 14);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/
	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 2, 14);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/
	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 2, 14);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/
	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 2, 14);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/
	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 2, 14);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/
	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 2, 14);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/
	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 2, 14);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/
	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 2, 14);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/
	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 2, 14);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/
	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 2, 14);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/
	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 2, 1);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/
	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 2, 1);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/
	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 2, 1);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/
	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 2, 1);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/
	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 2, 1);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/
	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 2, 1);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/
	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 2, 1);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/
	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 2, 1);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/
	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 2, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/
	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 2, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/
 


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
	const real_T *c_1 = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *c_2 = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *c_3 = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *c_4 = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *c_5 = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *c_6 = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *c_7 = (const real_T*) ssGetInputPortSignal(S,6);
	const real_T *c_8 = (const real_T*) ssGetInputPortSignal(S,7);
	const real_T *c_9 = (const real_T*) ssGetInputPortSignal(S,8);
	const real_T *c_10 = (const real_T*) ssGetInputPortSignal(S,9);
	const real_T *c_11 = (const real_T*) ssGetInputPortSignal(S,10);
	const real_T *H_1 = (const real_T*) ssGetInputPortSignal(S,11);
	const real_T *H_2 = (const real_T*) ssGetInputPortSignal(S,12);
	const real_T *H_3 = (const real_T*) ssGetInputPortSignal(S,13);
	const real_T *H_4 = (const real_T*) ssGetInputPortSignal(S,14);
	const real_T *H_5 = (const real_T*) ssGetInputPortSignal(S,15);
	const real_T *H_6 = (const real_T*) ssGetInputPortSignal(S,16);
	const real_T *H_7 = (const real_T*) ssGetInputPortSignal(S,17);
	const real_T *H_8 = (const real_T*) ssGetInputPortSignal(S,18);
	const real_T *H_9 = (const real_T*) ssGetInputPortSignal(S,19);
	const real_T *H_10 = (const real_T*) ssGetInputPortSignal(S,20);
	const real_T *H_11 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *f_1 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *f_2 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *f_3 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *f_4 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *f_5 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *f_6 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *f_7 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *f_8 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *f_9 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *f_10 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *f_11 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *C_1 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *C_2 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *C_3 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *C_4 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *C_5 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *C_6 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *C_7 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *C_8 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *C_9 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *C_10 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *A_2 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *A_3 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *A_4 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *A_5 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *A_6 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *A_7 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *A_8 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *A_9 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *A_10 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *A_11 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *b_2 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *b_3 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *b_4 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *b_5 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *b_6 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *b_7 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *b_8 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *b_9 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *b_10 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *b_11 = (const real_T*) ssGetInputPortSignal(S,62);
	
    real_T *X = (real_T*) ssGetOutputPortSignal(S,0);
	
	

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
		params.c_2[i] = (double) c_2[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_3[i] = (double) c_3[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_4[i] = (double) c_4[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_5[i] = (double) c_5[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_6[i] = (double) c_6[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_7[i] = (double) c_7[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_8[i] = (double) c_8[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_9[i] = (double) c_9[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_10[i] = (double) c_10[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_11[i] = (double) c_11[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_1[i] = (double) H_1[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_2[i] = (double) H_2[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_3[i] = (double) H_3[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_4[i] = (double) H_4[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_5[i] = (double) H_5[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_6[i] = (double) H_6[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_7[i] = (double) H_7[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_8[i] = (double) H_8[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_9[i] = (double) H_9[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_10[i] = (double) H_10[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_11[i] = (double) H_11[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_1[i] = (double) f_1[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_2[i] = (double) f_2[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_3[i] = (double) f_3[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_4[i] = (double) f_4[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_5[i] = (double) f_5[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_6[i] = (double) f_6[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_7[i] = (double) f_7[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_8[i] = (double) f_8[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_9[i] = (double) f_9[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_10[i] = (double) f_10[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_11[i] = (double) f_11[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_1[i] = (double) C_1[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_2[i] = (double) C_2[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_3[i] = (double) C_3[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_4[i] = (double) C_4[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_5[i] = (double) C_5[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_6[i] = (double) C_6[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_7[i] = (double) C_7[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_8[i] = (double) C_8[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_9[i] = (double) C_9[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_10[i] = (double) C_10[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_2[i] = (double) A_2[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_3[i] = (double) A_3[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_4[i] = (double) A_4[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_5[i] = (double) A_5[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_6[i] = (double) A_6[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_7[i] = (double) A_7[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_8[i] = (double) A_8[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_9[i] = (double) A_9[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_10[i] = (double) A_10[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_11[i] = (double) A_11[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_2[i] = (double) b_2[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_3[i] = (double) b_3[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_4[i] = (double) b_4[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_5[i] = (double) b_5[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_6[i] = (double) b_6[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_7[i] = (double) b_7[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_8[i] = (double) b_8[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_9[i] = (double) b_9[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_10[i] = (double) b_10[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_11[i] = (double) b_11[i]; 
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
		X[i] = (real_T) output.X[i]; 
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



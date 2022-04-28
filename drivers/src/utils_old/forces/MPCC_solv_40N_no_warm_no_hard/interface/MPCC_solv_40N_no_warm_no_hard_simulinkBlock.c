/*
MPCC_solv_40N_no_warm_no_hard : A fast customized optimization solver.

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
#define S_FUNCTION_NAME MPCC_solv_40N_no_warm_no_hard_simulinkBlock

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/MPCC_solv_40N_no_warm_no_hard.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef MPCC_solv_40N_no_warm_no_hardinterface_float MPCC_solv_40N_no_warm_no_hardnmpc_float;





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

	/* initialize input ports - there are 244 in total */
    if (!ssSetNumInputPorts(S, 244)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 10, 1);
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
    ssSetInputPortMatrixDimensions(S,  11, 10, 1);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/
	
	/* Input Port 12 */
    ssSetInputPortMatrixDimensions(S,  12, 10, 1);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 12, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/
	
	/* Input Port 13 */
    ssSetInputPortMatrixDimensions(S,  13, 10, 1);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 13, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/
	
	/* Input Port 14 */
    ssSetInputPortMatrixDimensions(S,  14, 10, 1);
    ssSetInputPortDataType(S, 14, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 14, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 14, 1); /*direct input signal access*/
	
	/* Input Port 15 */
    ssSetInputPortMatrixDimensions(S,  15, 10, 1);
    ssSetInputPortDataType(S, 15, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 15, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 15, 1); /*direct input signal access*/
	
	/* Input Port 16 */
    ssSetInputPortMatrixDimensions(S,  16, 10, 1);
    ssSetInputPortDataType(S, 16, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 16, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 16, 1); /*direct input signal access*/
	
	/* Input Port 17 */
    ssSetInputPortMatrixDimensions(S,  17, 10, 1);
    ssSetInputPortDataType(S, 17, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 17, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 17, 1); /*direct input signal access*/
	
	/* Input Port 18 */
    ssSetInputPortMatrixDimensions(S,  18, 10, 1);
    ssSetInputPortDataType(S, 18, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 18, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 18, 1); /*direct input signal access*/
	
	/* Input Port 19 */
    ssSetInputPortMatrixDimensions(S,  19, 10, 1);
    ssSetInputPortDataType(S, 19, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 19, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 19, 1); /*direct input signal access*/
	
	/* Input Port 20 */
    ssSetInputPortMatrixDimensions(S,  20, 10, 1);
    ssSetInputPortDataType(S, 20, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 20, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 20, 1); /*direct input signal access*/
	
	/* Input Port 21 */
    ssSetInputPortMatrixDimensions(S,  21, 10, 1);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/
	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 10, 1);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/
	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 10, 1);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/
	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 10, 1);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/
	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 10, 1);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/
	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 10, 1);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/
	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 10, 1);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/
	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 10, 1);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/
	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 10, 1);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/
	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 10, 1);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/
	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 10, 1);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/
	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 10, 1);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/
	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 10, 1);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/
	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 10, 1);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/
	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 10, 1);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/
	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 10, 1);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/
	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 10, 1);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/
	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 10, 1);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/
	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 10, 1);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/
	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 10, 1);
    ssSetInputPortDataType(S, 40, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 40, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 40, 1); /*direct input signal access*/
	
	/* Input Port 41 */
    ssSetInputPortMatrixDimensions(S,  41, 14, 14);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/
	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 14, 14);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/
	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 14, 14);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/
	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 14, 14);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/
	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 14, 14);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/
	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 14, 14);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/
	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 14, 14);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/
	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 14, 14);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/
	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 14, 14);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/
	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 14, 14);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/
	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 14, 14);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/
	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 14, 14);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/
	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 14, 14);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/
	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 14, 14);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/
	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 14, 14);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/
	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 14, 14);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/
	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 14, 14);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/
	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 14, 14);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/
	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 14, 14);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/
	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 14, 14);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/
	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 14, 14);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/
	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 14, 14);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/
	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 14, 14);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/
	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 14, 14);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/
	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 14, 14);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/
	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 14, 14);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/
	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 14, 14);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/
	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 14, 14);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/
	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 14, 14);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/
	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 14, 14);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/
	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 14, 14);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/
	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 14, 14);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/
	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 14, 14);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/
	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 14, 14);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/
	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 14, 14);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/
	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 14, 14);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/
	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 14, 14);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/
	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 14, 14);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/
	
	/* Input Port 79 */
    ssSetInputPortMatrixDimensions(S,  79, 14, 14);
    ssSetInputPortDataType(S, 79, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 79, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 79, 1); /*direct input signal access*/
	
	/* Input Port 80 */
    ssSetInputPortMatrixDimensions(S,  80, 14, 14);
    ssSetInputPortDataType(S, 80, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 80, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 80, 1); /*direct input signal access*/
	
	/* Input Port 81 */
    ssSetInputPortMatrixDimensions(S,  81, 14, 14);
    ssSetInputPortDataType(S, 81, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 81, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 81, 1); /*direct input signal access*/
	
	/* Input Port 82 */
    ssSetInputPortMatrixDimensions(S,  82, 14, 1);
    ssSetInputPortDataType(S, 82, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 82, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 82, 1); /*direct input signal access*/
	
	/* Input Port 83 */
    ssSetInputPortMatrixDimensions(S,  83, 14, 1);
    ssSetInputPortDataType(S, 83, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 83, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 83, 1); /*direct input signal access*/
	
	/* Input Port 84 */
    ssSetInputPortMatrixDimensions(S,  84, 14, 1);
    ssSetInputPortDataType(S, 84, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 84, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 84, 1); /*direct input signal access*/
	
	/* Input Port 85 */
    ssSetInputPortMatrixDimensions(S,  85, 14, 1);
    ssSetInputPortDataType(S, 85, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 85, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 85, 1); /*direct input signal access*/
	
	/* Input Port 86 */
    ssSetInputPortMatrixDimensions(S,  86, 14, 1);
    ssSetInputPortDataType(S, 86, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 86, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 86, 1); /*direct input signal access*/
	
	/* Input Port 87 */
    ssSetInputPortMatrixDimensions(S,  87, 14, 1);
    ssSetInputPortDataType(S, 87, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 87, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 87, 1); /*direct input signal access*/
	
	/* Input Port 88 */
    ssSetInputPortMatrixDimensions(S,  88, 14, 1);
    ssSetInputPortDataType(S, 88, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 88, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 88, 1); /*direct input signal access*/
	
	/* Input Port 89 */
    ssSetInputPortMatrixDimensions(S,  89, 14, 1);
    ssSetInputPortDataType(S, 89, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 89, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 89, 1); /*direct input signal access*/
	
	/* Input Port 90 */
    ssSetInputPortMatrixDimensions(S,  90, 14, 1);
    ssSetInputPortDataType(S, 90, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 90, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 90, 1); /*direct input signal access*/
	
	/* Input Port 91 */
    ssSetInputPortMatrixDimensions(S,  91, 14, 1);
    ssSetInputPortDataType(S, 91, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 91, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 91, 1); /*direct input signal access*/
	
	/* Input Port 92 */
    ssSetInputPortMatrixDimensions(S,  92, 14, 1);
    ssSetInputPortDataType(S, 92, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 92, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 92, 1); /*direct input signal access*/
	
	/* Input Port 93 */
    ssSetInputPortMatrixDimensions(S,  93, 14, 1);
    ssSetInputPortDataType(S, 93, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 93, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 93, 1); /*direct input signal access*/
	
	/* Input Port 94 */
    ssSetInputPortMatrixDimensions(S,  94, 14, 1);
    ssSetInputPortDataType(S, 94, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 94, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 94, 1); /*direct input signal access*/
	
	/* Input Port 95 */
    ssSetInputPortMatrixDimensions(S,  95, 14, 1);
    ssSetInputPortDataType(S, 95, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 95, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 95, 1); /*direct input signal access*/
	
	/* Input Port 96 */
    ssSetInputPortMatrixDimensions(S,  96, 14, 1);
    ssSetInputPortDataType(S, 96, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 96, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 96, 1); /*direct input signal access*/
	
	/* Input Port 97 */
    ssSetInputPortMatrixDimensions(S,  97, 14, 1);
    ssSetInputPortDataType(S, 97, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 97, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 97, 1); /*direct input signal access*/
	
	/* Input Port 98 */
    ssSetInputPortMatrixDimensions(S,  98, 14, 1);
    ssSetInputPortDataType(S, 98, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 98, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 98, 1); /*direct input signal access*/
	
	/* Input Port 99 */
    ssSetInputPortMatrixDimensions(S,  99, 14, 1);
    ssSetInputPortDataType(S, 99, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 99, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 99, 1); /*direct input signal access*/
	
	/* Input Port 100 */
    ssSetInputPortMatrixDimensions(S,  100, 14, 1);
    ssSetInputPortDataType(S, 100, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 100, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 100, 1); /*direct input signal access*/
	
	/* Input Port 101 */
    ssSetInputPortMatrixDimensions(S,  101, 14, 1);
    ssSetInputPortDataType(S, 101, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 101, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 101, 1); /*direct input signal access*/
	
	/* Input Port 102 */
    ssSetInputPortMatrixDimensions(S,  102, 14, 1);
    ssSetInputPortDataType(S, 102, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 102, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 102, 1); /*direct input signal access*/
	
	/* Input Port 103 */
    ssSetInputPortMatrixDimensions(S,  103, 14, 1);
    ssSetInputPortDataType(S, 103, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 103, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 103, 1); /*direct input signal access*/
	
	/* Input Port 104 */
    ssSetInputPortMatrixDimensions(S,  104, 14, 1);
    ssSetInputPortDataType(S, 104, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 104, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 104, 1); /*direct input signal access*/
	
	/* Input Port 105 */
    ssSetInputPortMatrixDimensions(S,  105, 14, 1);
    ssSetInputPortDataType(S, 105, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 105, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 105, 1); /*direct input signal access*/
	
	/* Input Port 106 */
    ssSetInputPortMatrixDimensions(S,  106, 14, 1);
    ssSetInputPortDataType(S, 106, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 106, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 106, 1); /*direct input signal access*/
	
	/* Input Port 107 */
    ssSetInputPortMatrixDimensions(S,  107, 14, 1);
    ssSetInputPortDataType(S, 107, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 107, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 107, 1); /*direct input signal access*/
	
	/* Input Port 108 */
    ssSetInputPortMatrixDimensions(S,  108, 14, 1);
    ssSetInputPortDataType(S, 108, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 108, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 108, 1); /*direct input signal access*/
	
	/* Input Port 109 */
    ssSetInputPortMatrixDimensions(S,  109, 14, 1);
    ssSetInputPortDataType(S, 109, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 109, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 109, 1); /*direct input signal access*/
	
	/* Input Port 110 */
    ssSetInputPortMatrixDimensions(S,  110, 14, 1);
    ssSetInputPortDataType(S, 110, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 110, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 110, 1); /*direct input signal access*/
	
	/* Input Port 111 */
    ssSetInputPortMatrixDimensions(S,  111, 14, 1);
    ssSetInputPortDataType(S, 111, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 111, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 111, 1); /*direct input signal access*/
	
	/* Input Port 112 */
    ssSetInputPortMatrixDimensions(S,  112, 14, 1);
    ssSetInputPortDataType(S, 112, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 112, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 112, 1); /*direct input signal access*/
	
	/* Input Port 113 */
    ssSetInputPortMatrixDimensions(S,  113, 14, 1);
    ssSetInputPortDataType(S, 113, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 113, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 113, 1); /*direct input signal access*/
	
	/* Input Port 114 */
    ssSetInputPortMatrixDimensions(S,  114, 14, 1);
    ssSetInputPortDataType(S, 114, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 114, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 114, 1); /*direct input signal access*/
	
	/* Input Port 115 */
    ssSetInputPortMatrixDimensions(S,  115, 14, 1);
    ssSetInputPortDataType(S, 115, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 115, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 115, 1); /*direct input signal access*/
	
	/* Input Port 116 */
    ssSetInputPortMatrixDimensions(S,  116, 14, 1);
    ssSetInputPortDataType(S, 116, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 116, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 116, 1); /*direct input signal access*/
	
	/* Input Port 117 */
    ssSetInputPortMatrixDimensions(S,  117, 14, 1);
    ssSetInputPortDataType(S, 117, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 117, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 117, 1); /*direct input signal access*/
	
	/* Input Port 118 */
    ssSetInputPortMatrixDimensions(S,  118, 14, 1);
    ssSetInputPortDataType(S, 118, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 118, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 118, 1); /*direct input signal access*/
	
	/* Input Port 119 */
    ssSetInputPortMatrixDimensions(S,  119, 14, 1);
    ssSetInputPortDataType(S, 119, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 119, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 119, 1); /*direct input signal access*/
	
	/* Input Port 120 */
    ssSetInputPortMatrixDimensions(S,  120, 14, 1);
    ssSetInputPortDataType(S, 120, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 120, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 120, 1); /*direct input signal access*/
	
	/* Input Port 121 */
    ssSetInputPortMatrixDimensions(S,  121, 14, 1);
    ssSetInputPortDataType(S, 121, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 121, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 121, 1); /*direct input signal access*/
	
	/* Input Port 122 */
    ssSetInputPortMatrixDimensions(S,  122, 14, 1);
    ssSetInputPortDataType(S, 122, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 122, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 122, 1); /*direct input signal access*/
	
	/* Input Port 123 */
    ssSetInputPortMatrixDimensions(S,  123, 10, 14);
    ssSetInputPortDataType(S, 123, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 123, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 123, 1); /*direct input signal access*/
	
	/* Input Port 124 */
    ssSetInputPortMatrixDimensions(S,  124, 10, 14);
    ssSetInputPortDataType(S, 124, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 124, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 124, 1); /*direct input signal access*/
	
	/* Input Port 125 */
    ssSetInputPortMatrixDimensions(S,  125, 10, 14);
    ssSetInputPortDataType(S, 125, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 125, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 125, 1); /*direct input signal access*/
	
	/* Input Port 126 */
    ssSetInputPortMatrixDimensions(S,  126, 10, 14);
    ssSetInputPortDataType(S, 126, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 126, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 126, 1); /*direct input signal access*/
	
	/* Input Port 127 */
    ssSetInputPortMatrixDimensions(S,  127, 10, 14);
    ssSetInputPortDataType(S, 127, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 127, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 127, 1); /*direct input signal access*/
	
	/* Input Port 128 */
    ssSetInputPortMatrixDimensions(S,  128, 10, 14);
    ssSetInputPortDataType(S, 128, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 128, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 128, 1); /*direct input signal access*/
	
	/* Input Port 129 */
    ssSetInputPortMatrixDimensions(S,  129, 10, 14);
    ssSetInputPortDataType(S, 129, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 129, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 129, 1); /*direct input signal access*/
	
	/* Input Port 130 */
    ssSetInputPortMatrixDimensions(S,  130, 10, 14);
    ssSetInputPortDataType(S, 130, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 130, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 130, 1); /*direct input signal access*/
	
	/* Input Port 131 */
    ssSetInputPortMatrixDimensions(S,  131, 10, 14);
    ssSetInputPortDataType(S, 131, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 131, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 131, 1); /*direct input signal access*/
	
	/* Input Port 132 */
    ssSetInputPortMatrixDimensions(S,  132, 10, 14);
    ssSetInputPortDataType(S, 132, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 132, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 132, 1); /*direct input signal access*/
	
	/* Input Port 133 */
    ssSetInputPortMatrixDimensions(S,  133, 10, 14);
    ssSetInputPortDataType(S, 133, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 133, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 133, 1); /*direct input signal access*/
	
	/* Input Port 134 */
    ssSetInputPortMatrixDimensions(S,  134, 10, 14);
    ssSetInputPortDataType(S, 134, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 134, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 134, 1); /*direct input signal access*/
	
	/* Input Port 135 */
    ssSetInputPortMatrixDimensions(S,  135, 10, 14);
    ssSetInputPortDataType(S, 135, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 135, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 135, 1); /*direct input signal access*/
	
	/* Input Port 136 */
    ssSetInputPortMatrixDimensions(S,  136, 10, 14);
    ssSetInputPortDataType(S, 136, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 136, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 136, 1); /*direct input signal access*/
	
	/* Input Port 137 */
    ssSetInputPortMatrixDimensions(S,  137, 10, 14);
    ssSetInputPortDataType(S, 137, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 137, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 137, 1); /*direct input signal access*/
	
	/* Input Port 138 */
    ssSetInputPortMatrixDimensions(S,  138, 10, 14);
    ssSetInputPortDataType(S, 138, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 138, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 138, 1); /*direct input signal access*/
	
	/* Input Port 139 */
    ssSetInputPortMatrixDimensions(S,  139, 10, 14);
    ssSetInputPortDataType(S, 139, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 139, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 139, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 139, 1); /*direct input signal access*/
	
	/* Input Port 140 */
    ssSetInputPortMatrixDimensions(S,  140, 10, 14);
    ssSetInputPortDataType(S, 140, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 140, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 140, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 140, 1); /*direct input signal access*/
	
	/* Input Port 141 */
    ssSetInputPortMatrixDimensions(S,  141, 10, 14);
    ssSetInputPortDataType(S, 141, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 141, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 141, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 141, 1); /*direct input signal access*/
	
	/* Input Port 142 */
    ssSetInputPortMatrixDimensions(S,  142, 10, 14);
    ssSetInputPortDataType(S, 142, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 142, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 142, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 142, 1); /*direct input signal access*/
	
	/* Input Port 143 */
    ssSetInputPortMatrixDimensions(S,  143, 10, 14);
    ssSetInputPortDataType(S, 143, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 143, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 143, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 143, 1); /*direct input signal access*/
	
	/* Input Port 144 */
    ssSetInputPortMatrixDimensions(S,  144, 10, 14);
    ssSetInputPortDataType(S, 144, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 144, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 144, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 144, 1); /*direct input signal access*/
	
	/* Input Port 145 */
    ssSetInputPortMatrixDimensions(S,  145, 10, 14);
    ssSetInputPortDataType(S, 145, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 145, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 145, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 145, 1); /*direct input signal access*/
	
	/* Input Port 146 */
    ssSetInputPortMatrixDimensions(S,  146, 10, 14);
    ssSetInputPortDataType(S, 146, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 146, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 146, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 146, 1); /*direct input signal access*/
	
	/* Input Port 147 */
    ssSetInputPortMatrixDimensions(S,  147, 10, 14);
    ssSetInputPortDataType(S, 147, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 147, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 147, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 147, 1); /*direct input signal access*/
	
	/* Input Port 148 */
    ssSetInputPortMatrixDimensions(S,  148, 10, 14);
    ssSetInputPortDataType(S, 148, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 148, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 148, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 148, 1); /*direct input signal access*/
	
	/* Input Port 149 */
    ssSetInputPortMatrixDimensions(S,  149, 10, 14);
    ssSetInputPortDataType(S, 149, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 149, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 149, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 149, 1); /*direct input signal access*/
	
	/* Input Port 150 */
    ssSetInputPortMatrixDimensions(S,  150, 10, 14);
    ssSetInputPortDataType(S, 150, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 150, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 150, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 150, 1); /*direct input signal access*/
	
	/* Input Port 151 */
    ssSetInputPortMatrixDimensions(S,  151, 10, 14);
    ssSetInputPortDataType(S, 151, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 151, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 151, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 151, 1); /*direct input signal access*/
	
	/* Input Port 152 */
    ssSetInputPortMatrixDimensions(S,  152, 10, 14);
    ssSetInputPortDataType(S, 152, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 152, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 152, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 152, 1); /*direct input signal access*/
	
	/* Input Port 153 */
    ssSetInputPortMatrixDimensions(S,  153, 10, 14);
    ssSetInputPortDataType(S, 153, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 153, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 153, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 153, 1); /*direct input signal access*/
	
	/* Input Port 154 */
    ssSetInputPortMatrixDimensions(S,  154, 10, 14);
    ssSetInputPortDataType(S, 154, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 154, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 154, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 154, 1); /*direct input signal access*/
	
	/* Input Port 155 */
    ssSetInputPortMatrixDimensions(S,  155, 10, 14);
    ssSetInputPortDataType(S, 155, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 155, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 155, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 155, 1); /*direct input signal access*/
	
	/* Input Port 156 */
    ssSetInputPortMatrixDimensions(S,  156, 10, 14);
    ssSetInputPortDataType(S, 156, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 156, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 156, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 156, 1); /*direct input signal access*/
	
	/* Input Port 157 */
    ssSetInputPortMatrixDimensions(S,  157, 10, 14);
    ssSetInputPortDataType(S, 157, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 157, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 157, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 157, 1); /*direct input signal access*/
	
	/* Input Port 158 */
    ssSetInputPortMatrixDimensions(S,  158, 10, 14);
    ssSetInputPortDataType(S, 158, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 158, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 158, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 158, 1); /*direct input signal access*/
	
	/* Input Port 159 */
    ssSetInputPortMatrixDimensions(S,  159, 10, 14);
    ssSetInputPortDataType(S, 159, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 159, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 159, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 159, 1); /*direct input signal access*/
	
	/* Input Port 160 */
    ssSetInputPortMatrixDimensions(S,  160, 10, 14);
    ssSetInputPortDataType(S, 160, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 160, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 160, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 160, 1); /*direct input signal access*/
	
	/* Input Port 161 */
    ssSetInputPortMatrixDimensions(S,  161, 10, 14);
    ssSetInputPortDataType(S, 161, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 161, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 161, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 161, 1); /*direct input signal access*/
	
	/* Input Port 162 */
    ssSetInputPortMatrixDimensions(S,  162, 10, 14);
    ssSetInputPortDataType(S, 162, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 162, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 162, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 162, 1); /*direct input signal access*/
	
	/* Input Port 163 */
    ssSetInputPortMatrixDimensions(S,  163, 2, 14);
    ssSetInputPortDataType(S, 163, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 163, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 163, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 163, 1); /*direct input signal access*/
	
	/* Input Port 164 */
    ssSetInputPortMatrixDimensions(S,  164, 2, 14);
    ssSetInputPortDataType(S, 164, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 164, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 164, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 164, 1); /*direct input signal access*/
	
	/* Input Port 165 */
    ssSetInputPortMatrixDimensions(S,  165, 2, 14);
    ssSetInputPortDataType(S, 165, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 165, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 165, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 165, 1); /*direct input signal access*/
	
	/* Input Port 166 */
    ssSetInputPortMatrixDimensions(S,  166, 2, 14);
    ssSetInputPortDataType(S, 166, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 166, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 166, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 166, 1); /*direct input signal access*/
	
	/* Input Port 167 */
    ssSetInputPortMatrixDimensions(S,  167, 2, 14);
    ssSetInputPortDataType(S, 167, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 167, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 167, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 167, 1); /*direct input signal access*/
	
	/* Input Port 168 */
    ssSetInputPortMatrixDimensions(S,  168, 2, 14);
    ssSetInputPortDataType(S, 168, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 168, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 168, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 168, 1); /*direct input signal access*/
	
	/* Input Port 169 */
    ssSetInputPortMatrixDimensions(S,  169, 2, 14);
    ssSetInputPortDataType(S, 169, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 169, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 169, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 169, 1); /*direct input signal access*/
	
	/* Input Port 170 */
    ssSetInputPortMatrixDimensions(S,  170, 2, 14);
    ssSetInputPortDataType(S, 170, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 170, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 170, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 170, 1); /*direct input signal access*/
	
	/* Input Port 171 */
    ssSetInputPortMatrixDimensions(S,  171, 2, 14);
    ssSetInputPortDataType(S, 171, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 171, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 171, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 171, 1); /*direct input signal access*/
	
	/* Input Port 172 */
    ssSetInputPortMatrixDimensions(S,  172, 2, 14);
    ssSetInputPortDataType(S, 172, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 172, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 172, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 172, 1); /*direct input signal access*/
	
	/* Input Port 173 */
    ssSetInputPortMatrixDimensions(S,  173, 2, 14);
    ssSetInputPortDataType(S, 173, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 173, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 173, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 173, 1); /*direct input signal access*/
	
	/* Input Port 174 */
    ssSetInputPortMatrixDimensions(S,  174, 2, 14);
    ssSetInputPortDataType(S, 174, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 174, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 174, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 174, 1); /*direct input signal access*/
	
	/* Input Port 175 */
    ssSetInputPortMatrixDimensions(S,  175, 2, 14);
    ssSetInputPortDataType(S, 175, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 175, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 175, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 175, 1); /*direct input signal access*/
	
	/* Input Port 176 */
    ssSetInputPortMatrixDimensions(S,  176, 2, 14);
    ssSetInputPortDataType(S, 176, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 176, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 176, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 176, 1); /*direct input signal access*/
	
	/* Input Port 177 */
    ssSetInputPortMatrixDimensions(S,  177, 2, 14);
    ssSetInputPortDataType(S, 177, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 177, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 177, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 177, 1); /*direct input signal access*/
	
	/* Input Port 178 */
    ssSetInputPortMatrixDimensions(S,  178, 2, 14);
    ssSetInputPortDataType(S, 178, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 178, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 178, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 178, 1); /*direct input signal access*/
	
	/* Input Port 179 */
    ssSetInputPortMatrixDimensions(S,  179, 2, 14);
    ssSetInputPortDataType(S, 179, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 179, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 179, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 179, 1); /*direct input signal access*/
	
	/* Input Port 180 */
    ssSetInputPortMatrixDimensions(S,  180, 2, 14);
    ssSetInputPortDataType(S, 180, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 180, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 180, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 180, 1); /*direct input signal access*/
	
	/* Input Port 181 */
    ssSetInputPortMatrixDimensions(S,  181, 2, 14);
    ssSetInputPortDataType(S, 181, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 181, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 181, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 181, 1); /*direct input signal access*/
	
	/* Input Port 182 */
    ssSetInputPortMatrixDimensions(S,  182, 2, 14);
    ssSetInputPortDataType(S, 182, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 182, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 182, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 182, 1); /*direct input signal access*/
	
	/* Input Port 183 */
    ssSetInputPortMatrixDimensions(S,  183, 2, 14);
    ssSetInputPortDataType(S, 183, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 183, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 183, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 183, 1); /*direct input signal access*/
	
	/* Input Port 184 */
    ssSetInputPortMatrixDimensions(S,  184, 2, 14);
    ssSetInputPortDataType(S, 184, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 184, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 184, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 184, 1); /*direct input signal access*/
	
	/* Input Port 185 */
    ssSetInputPortMatrixDimensions(S,  185, 2, 14);
    ssSetInputPortDataType(S, 185, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 185, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 185, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 185, 1); /*direct input signal access*/
	
	/* Input Port 186 */
    ssSetInputPortMatrixDimensions(S,  186, 2, 14);
    ssSetInputPortDataType(S, 186, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 186, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 186, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 186, 1); /*direct input signal access*/
	
	/* Input Port 187 */
    ssSetInputPortMatrixDimensions(S,  187, 2, 14);
    ssSetInputPortDataType(S, 187, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 187, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 187, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 187, 1); /*direct input signal access*/
	
	/* Input Port 188 */
    ssSetInputPortMatrixDimensions(S,  188, 2, 14);
    ssSetInputPortDataType(S, 188, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 188, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 188, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 188, 1); /*direct input signal access*/
	
	/* Input Port 189 */
    ssSetInputPortMatrixDimensions(S,  189, 2, 14);
    ssSetInputPortDataType(S, 189, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 189, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 189, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 189, 1); /*direct input signal access*/
	
	/* Input Port 190 */
    ssSetInputPortMatrixDimensions(S,  190, 2, 14);
    ssSetInputPortDataType(S, 190, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 190, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 190, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 190, 1); /*direct input signal access*/
	
	/* Input Port 191 */
    ssSetInputPortMatrixDimensions(S,  191, 2, 14);
    ssSetInputPortDataType(S, 191, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 191, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 191, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 191, 1); /*direct input signal access*/
	
	/* Input Port 192 */
    ssSetInputPortMatrixDimensions(S,  192, 2, 14);
    ssSetInputPortDataType(S, 192, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 192, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 192, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 192, 1); /*direct input signal access*/
	
	/* Input Port 193 */
    ssSetInputPortMatrixDimensions(S,  193, 2, 14);
    ssSetInputPortDataType(S, 193, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 193, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 193, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 193, 1); /*direct input signal access*/
	
	/* Input Port 194 */
    ssSetInputPortMatrixDimensions(S,  194, 2, 14);
    ssSetInputPortDataType(S, 194, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 194, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 194, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 194, 1); /*direct input signal access*/
	
	/* Input Port 195 */
    ssSetInputPortMatrixDimensions(S,  195, 2, 14);
    ssSetInputPortDataType(S, 195, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 195, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 195, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 195, 1); /*direct input signal access*/
	
	/* Input Port 196 */
    ssSetInputPortMatrixDimensions(S,  196, 2, 14);
    ssSetInputPortDataType(S, 196, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 196, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 196, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 196, 1); /*direct input signal access*/
	
	/* Input Port 197 */
    ssSetInputPortMatrixDimensions(S,  197, 2, 14);
    ssSetInputPortDataType(S, 197, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 197, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 197, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 197, 1); /*direct input signal access*/
	
	/* Input Port 198 */
    ssSetInputPortMatrixDimensions(S,  198, 2, 14);
    ssSetInputPortDataType(S, 198, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 198, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 198, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 198, 1); /*direct input signal access*/
	
	/* Input Port 199 */
    ssSetInputPortMatrixDimensions(S,  199, 2, 14);
    ssSetInputPortDataType(S, 199, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 199, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 199, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 199, 1); /*direct input signal access*/
	
	/* Input Port 200 */
    ssSetInputPortMatrixDimensions(S,  200, 2, 14);
    ssSetInputPortDataType(S, 200, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 200, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 200, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 200, 1); /*direct input signal access*/
	
	/* Input Port 201 */
    ssSetInputPortMatrixDimensions(S,  201, 2, 14);
    ssSetInputPortDataType(S, 201, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 201, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 201, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 201, 1); /*direct input signal access*/
	
	/* Input Port 202 */
    ssSetInputPortMatrixDimensions(S,  202, 2, 14);
    ssSetInputPortDataType(S, 202, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 202, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 202, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 202, 1); /*direct input signal access*/
	
	/* Input Port 203 */
    ssSetInputPortMatrixDimensions(S,  203, 2, 1);
    ssSetInputPortDataType(S, 203, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 203, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 203, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 203, 1); /*direct input signal access*/
	
	/* Input Port 204 */
    ssSetInputPortMatrixDimensions(S,  204, 2, 1);
    ssSetInputPortDataType(S, 204, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 204, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 204, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 204, 1); /*direct input signal access*/
	
	/* Input Port 205 */
    ssSetInputPortMatrixDimensions(S,  205, 2, 1);
    ssSetInputPortDataType(S, 205, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 205, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 205, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 205, 1); /*direct input signal access*/
	
	/* Input Port 206 */
    ssSetInputPortMatrixDimensions(S,  206, 2, 1);
    ssSetInputPortDataType(S, 206, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 206, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 206, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 206, 1); /*direct input signal access*/
	
	/* Input Port 207 */
    ssSetInputPortMatrixDimensions(S,  207, 2, 1);
    ssSetInputPortDataType(S, 207, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 207, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 207, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 207, 1); /*direct input signal access*/
	
	/* Input Port 208 */
    ssSetInputPortMatrixDimensions(S,  208, 2, 1);
    ssSetInputPortDataType(S, 208, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 208, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 208, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 208, 1); /*direct input signal access*/
	
	/* Input Port 209 */
    ssSetInputPortMatrixDimensions(S,  209, 2, 1);
    ssSetInputPortDataType(S, 209, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 209, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 209, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 209, 1); /*direct input signal access*/
	
	/* Input Port 210 */
    ssSetInputPortMatrixDimensions(S,  210, 2, 1);
    ssSetInputPortDataType(S, 210, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 210, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 210, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 210, 1); /*direct input signal access*/
	
	/* Input Port 211 */
    ssSetInputPortMatrixDimensions(S,  211, 2, 1);
    ssSetInputPortDataType(S, 211, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 211, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 211, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 211, 1); /*direct input signal access*/
	
	/* Input Port 212 */
    ssSetInputPortMatrixDimensions(S,  212, 2, 1);
    ssSetInputPortDataType(S, 212, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 212, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 212, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 212, 1); /*direct input signal access*/
	
	/* Input Port 213 */
    ssSetInputPortMatrixDimensions(S,  213, 2, 1);
    ssSetInputPortDataType(S, 213, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 213, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 213, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 213, 1); /*direct input signal access*/
	
	/* Input Port 214 */
    ssSetInputPortMatrixDimensions(S,  214, 2, 1);
    ssSetInputPortDataType(S, 214, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 214, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 214, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 214, 1); /*direct input signal access*/
	
	/* Input Port 215 */
    ssSetInputPortMatrixDimensions(S,  215, 2, 1);
    ssSetInputPortDataType(S, 215, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 215, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 215, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 215, 1); /*direct input signal access*/
	
	/* Input Port 216 */
    ssSetInputPortMatrixDimensions(S,  216, 2, 1);
    ssSetInputPortDataType(S, 216, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 216, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 216, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 216, 1); /*direct input signal access*/
	
	/* Input Port 217 */
    ssSetInputPortMatrixDimensions(S,  217, 2, 1);
    ssSetInputPortDataType(S, 217, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 217, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 217, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 217, 1); /*direct input signal access*/
	
	/* Input Port 218 */
    ssSetInputPortMatrixDimensions(S,  218, 2, 1);
    ssSetInputPortDataType(S, 218, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 218, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 218, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 218, 1); /*direct input signal access*/
	
	/* Input Port 219 */
    ssSetInputPortMatrixDimensions(S,  219, 2, 1);
    ssSetInputPortDataType(S, 219, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 219, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 219, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 219, 1); /*direct input signal access*/
	
	/* Input Port 220 */
    ssSetInputPortMatrixDimensions(S,  220, 2, 1);
    ssSetInputPortDataType(S, 220, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 220, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 220, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 220, 1); /*direct input signal access*/
	
	/* Input Port 221 */
    ssSetInputPortMatrixDimensions(S,  221, 2, 1);
    ssSetInputPortDataType(S, 221, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 221, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 221, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 221, 1); /*direct input signal access*/
	
	/* Input Port 222 */
    ssSetInputPortMatrixDimensions(S,  222, 2, 1);
    ssSetInputPortDataType(S, 222, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 222, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 222, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 222, 1); /*direct input signal access*/
	
	/* Input Port 223 */
    ssSetInputPortMatrixDimensions(S,  223, 2, 1);
    ssSetInputPortDataType(S, 223, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 223, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 223, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 223, 1); /*direct input signal access*/
	
	/* Input Port 224 */
    ssSetInputPortMatrixDimensions(S,  224, 2, 1);
    ssSetInputPortDataType(S, 224, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 224, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 224, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 224, 1); /*direct input signal access*/
	
	/* Input Port 225 */
    ssSetInputPortMatrixDimensions(S,  225, 2, 1);
    ssSetInputPortDataType(S, 225, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 225, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 225, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 225, 1); /*direct input signal access*/
	
	/* Input Port 226 */
    ssSetInputPortMatrixDimensions(S,  226, 2, 1);
    ssSetInputPortDataType(S, 226, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 226, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 226, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 226, 1); /*direct input signal access*/
	
	/* Input Port 227 */
    ssSetInputPortMatrixDimensions(S,  227, 2, 1);
    ssSetInputPortDataType(S, 227, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 227, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 227, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 227, 1); /*direct input signal access*/
	
	/* Input Port 228 */
    ssSetInputPortMatrixDimensions(S,  228, 2, 1);
    ssSetInputPortDataType(S, 228, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 228, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 228, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 228, 1); /*direct input signal access*/
	
	/* Input Port 229 */
    ssSetInputPortMatrixDimensions(S,  229, 2, 1);
    ssSetInputPortDataType(S, 229, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 229, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 229, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 229, 1); /*direct input signal access*/
	
	/* Input Port 230 */
    ssSetInputPortMatrixDimensions(S,  230, 2, 1);
    ssSetInputPortDataType(S, 230, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 230, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 230, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 230, 1); /*direct input signal access*/
	
	/* Input Port 231 */
    ssSetInputPortMatrixDimensions(S,  231, 2, 1);
    ssSetInputPortDataType(S, 231, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 231, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 231, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 231, 1); /*direct input signal access*/
	
	/* Input Port 232 */
    ssSetInputPortMatrixDimensions(S,  232, 2, 1);
    ssSetInputPortDataType(S, 232, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 232, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 232, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 232, 1); /*direct input signal access*/
	
	/* Input Port 233 */
    ssSetInputPortMatrixDimensions(S,  233, 2, 1);
    ssSetInputPortDataType(S, 233, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 233, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 233, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 233, 1); /*direct input signal access*/
	
	/* Input Port 234 */
    ssSetInputPortMatrixDimensions(S,  234, 2, 1);
    ssSetInputPortDataType(S, 234, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 234, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 234, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 234, 1); /*direct input signal access*/
	
	/* Input Port 235 */
    ssSetInputPortMatrixDimensions(S,  235, 2, 1);
    ssSetInputPortDataType(S, 235, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 235, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 235, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 235, 1); /*direct input signal access*/
	
	/* Input Port 236 */
    ssSetInputPortMatrixDimensions(S,  236, 2, 1);
    ssSetInputPortDataType(S, 236, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 236, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 236, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 236, 1); /*direct input signal access*/
	
	/* Input Port 237 */
    ssSetInputPortMatrixDimensions(S,  237, 2, 1);
    ssSetInputPortDataType(S, 237, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 237, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 237, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 237, 1); /*direct input signal access*/
	
	/* Input Port 238 */
    ssSetInputPortMatrixDimensions(S,  238, 2, 1);
    ssSetInputPortDataType(S, 238, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 238, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 238, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 238, 1); /*direct input signal access*/
	
	/* Input Port 239 */
    ssSetInputPortMatrixDimensions(S,  239, 2, 1);
    ssSetInputPortDataType(S, 239, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 239, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 239, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 239, 1); /*direct input signal access*/
	
	/* Input Port 240 */
    ssSetInputPortMatrixDimensions(S,  240, 2, 1);
    ssSetInputPortDataType(S, 240, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 240, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 240, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 240, 1); /*direct input signal access*/
	
	/* Input Port 241 */
    ssSetInputPortMatrixDimensions(S,  241, 2, 1);
    ssSetInputPortDataType(S, 241, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 241, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 241, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 241, 1); /*direct input signal access*/
	
	/* Input Port 242 */
    ssSetInputPortMatrixDimensions(S,  242, 2, 1);
    ssSetInputPortDataType(S, 242, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 242, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 242, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 242, 1); /*direct input signal access*/
	
	/* Input Port 243 */
    ssSetInputPortMatrixDimensions(S,  243, 1, 1);
    ssSetInputPortDataType(S, 243, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 243, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 243, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 243, 1); /*direct input signal access*/
 


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
	const real_T *c_12 = (const real_T*) ssGetInputPortSignal(S,11);
	const real_T *c_13 = (const real_T*) ssGetInputPortSignal(S,12);
	const real_T *c_14 = (const real_T*) ssGetInputPortSignal(S,13);
	const real_T *c_15 = (const real_T*) ssGetInputPortSignal(S,14);
	const real_T *c_16 = (const real_T*) ssGetInputPortSignal(S,15);
	const real_T *c_17 = (const real_T*) ssGetInputPortSignal(S,16);
	const real_T *c_18 = (const real_T*) ssGetInputPortSignal(S,17);
	const real_T *c_19 = (const real_T*) ssGetInputPortSignal(S,18);
	const real_T *c_20 = (const real_T*) ssGetInputPortSignal(S,19);
	const real_T *c_21 = (const real_T*) ssGetInputPortSignal(S,20);
	const real_T *c_22 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *c_23 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *c_24 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *c_25 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *c_26 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *c_27 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *c_28 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *c_29 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *c_30 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *c_31 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *c_32 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *c_33 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *c_34 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *c_35 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *c_36 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *c_37 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *c_38 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *c_39 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *c_40 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *c_41 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *H_1 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *H_2 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *H_3 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *H_4 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *H_5 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *H_6 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *H_7 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *H_8 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *H_9 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *H_10 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *H_11 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *H_12 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *H_13 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *H_14 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *H_15 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *H_16 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *H_17 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *H_18 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *H_19 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *H_20 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *H_21 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *H_22 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *H_23 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *H_24 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *H_25 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *H_26 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *H_27 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *H_28 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *H_29 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *H_30 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *H_31 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *H_32 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *H_33 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *H_34 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *H_35 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *H_36 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *H_37 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *H_38 = (const real_T*) ssGetInputPortSignal(S,78);
	const real_T *H_39 = (const real_T*) ssGetInputPortSignal(S,79);
	const real_T *H_40 = (const real_T*) ssGetInputPortSignal(S,80);
	const real_T *H_41 = (const real_T*) ssGetInputPortSignal(S,81);
	const real_T *f_1 = (const real_T*) ssGetInputPortSignal(S,82);
	const real_T *f_2 = (const real_T*) ssGetInputPortSignal(S,83);
	const real_T *f_3 = (const real_T*) ssGetInputPortSignal(S,84);
	const real_T *f_4 = (const real_T*) ssGetInputPortSignal(S,85);
	const real_T *f_5 = (const real_T*) ssGetInputPortSignal(S,86);
	const real_T *f_6 = (const real_T*) ssGetInputPortSignal(S,87);
	const real_T *f_7 = (const real_T*) ssGetInputPortSignal(S,88);
	const real_T *f_8 = (const real_T*) ssGetInputPortSignal(S,89);
	const real_T *f_9 = (const real_T*) ssGetInputPortSignal(S,90);
	const real_T *f_10 = (const real_T*) ssGetInputPortSignal(S,91);
	const real_T *f_11 = (const real_T*) ssGetInputPortSignal(S,92);
	const real_T *f_12 = (const real_T*) ssGetInputPortSignal(S,93);
	const real_T *f_13 = (const real_T*) ssGetInputPortSignal(S,94);
	const real_T *f_14 = (const real_T*) ssGetInputPortSignal(S,95);
	const real_T *f_15 = (const real_T*) ssGetInputPortSignal(S,96);
	const real_T *f_16 = (const real_T*) ssGetInputPortSignal(S,97);
	const real_T *f_17 = (const real_T*) ssGetInputPortSignal(S,98);
	const real_T *f_18 = (const real_T*) ssGetInputPortSignal(S,99);
	const real_T *f_19 = (const real_T*) ssGetInputPortSignal(S,100);
	const real_T *f_20 = (const real_T*) ssGetInputPortSignal(S,101);
	const real_T *f_21 = (const real_T*) ssGetInputPortSignal(S,102);
	const real_T *f_22 = (const real_T*) ssGetInputPortSignal(S,103);
	const real_T *f_23 = (const real_T*) ssGetInputPortSignal(S,104);
	const real_T *f_24 = (const real_T*) ssGetInputPortSignal(S,105);
	const real_T *f_25 = (const real_T*) ssGetInputPortSignal(S,106);
	const real_T *f_26 = (const real_T*) ssGetInputPortSignal(S,107);
	const real_T *f_27 = (const real_T*) ssGetInputPortSignal(S,108);
	const real_T *f_28 = (const real_T*) ssGetInputPortSignal(S,109);
	const real_T *f_29 = (const real_T*) ssGetInputPortSignal(S,110);
	const real_T *f_30 = (const real_T*) ssGetInputPortSignal(S,111);
	const real_T *f_31 = (const real_T*) ssGetInputPortSignal(S,112);
	const real_T *f_32 = (const real_T*) ssGetInputPortSignal(S,113);
	const real_T *f_33 = (const real_T*) ssGetInputPortSignal(S,114);
	const real_T *f_34 = (const real_T*) ssGetInputPortSignal(S,115);
	const real_T *f_35 = (const real_T*) ssGetInputPortSignal(S,116);
	const real_T *f_36 = (const real_T*) ssGetInputPortSignal(S,117);
	const real_T *f_37 = (const real_T*) ssGetInputPortSignal(S,118);
	const real_T *f_38 = (const real_T*) ssGetInputPortSignal(S,119);
	const real_T *f_39 = (const real_T*) ssGetInputPortSignal(S,120);
	const real_T *f_40 = (const real_T*) ssGetInputPortSignal(S,121);
	const real_T *f_41 = (const real_T*) ssGetInputPortSignal(S,122);
	const real_T *C_1 = (const real_T*) ssGetInputPortSignal(S,123);
	const real_T *C_2 = (const real_T*) ssGetInputPortSignal(S,124);
	const real_T *C_3 = (const real_T*) ssGetInputPortSignal(S,125);
	const real_T *C_4 = (const real_T*) ssGetInputPortSignal(S,126);
	const real_T *C_5 = (const real_T*) ssGetInputPortSignal(S,127);
	const real_T *C_6 = (const real_T*) ssGetInputPortSignal(S,128);
	const real_T *C_7 = (const real_T*) ssGetInputPortSignal(S,129);
	const real_T *C_8 = (const real_T*) ssGetInputPortSignal(S,130);
	const real_T *C_9 = (const real_T*) ssGetInputPortSignal(S,131);
	const real_T *C_10 = (const real_T*) ssGetInputPortSignal(S,132);
	const real_T *C_11 = (const real_T*) ssGetInputPortSignal(S,133);
	const real_T *C_12 = (const real_T*) ssGetInputPortSignal(S,134);
	const real_T *C_13 = (const real_T*) ssGetInputPortSignal(S,135);
	const real_T *C_14 = (const real_T*) ssGetInputPortSignal(S,136);
	const real_T *C_15 = (const real_T*) ssGetInputPortSignal(S,137);
	const real_T *C_16 = (const real_T*) ssGetInputPortSignal(S,138);
	const real_T *C_17 = (const real_T*) ssGetInputPortSignal(S,139);
	const real_T *C_18 = (const real_T*) ssGetInputPortSignal(S,140);
	const real_T *C_19 = (const real_T*) ssGetInputPortSignal(S,141);
	const real_T *C_20 = (const real_T*) ssGetInputPortSignal(S,142);
	const real_T *C_21 = (const real_T*) ssGetInputPortSignal(S,143);
	const real_T *C_22 = (const real_T*) ssGetInputPortSignal(S,144);
	const real_T *C_23 = (const real_T*) ssGetInputPortSignal(S,145);
	const real_T *C_24 = (const real_T*) ssGetInputPortSignal(S,146);
	const real_T *C_25 = (const real_T*) ssGetInputPortSignal(S,147);
	const real_T *C_26 = (const real_T*) ssGetInputPortSignal(S,148);
	const real_T *C_27 = (const real_T*) ssGetInputPortSignal(S,149);
	const real_T *C_28 = (const real_T*) ssGetInputPortSignal(S,150);
	const real_T *C_29 = (const real_T*) ssGetInputPortSignal(S,151);
	const real_T *C_30 = (const real_T*) ssGetInputPortSignal(S,152);
	const real_T *C_31 = (const real_T*) ssGetInputPortSignal(S,153);
	const real_T *C_32 = (const real_T*) ssGetInputPortSignal(S,154);
	const real_T *C_33 = (const real_T*) ssGetInputPortSignal(S,155);
	const real_T *C_34 = (const real_T*) ssGetInputPortSignal(S,156);
	const real_T *C_35 = (const real_T*) ssGetInputPortSignal(S,157);
	const real_T *C_36 = (const real_T*) ssGetInputPortSignal(S,158);
	const real_T *C_37 = (const real_T*) ssGetInputPortSignal(S,159);
	const real_T *C_38 = (const real_T*) ssGetInputPortSignal(S,160);
	const real_T *C_39 = (const real_T*) ssGetInputPortSignal(S,161);
	const real_T *C_40 = (const real_T*) ssGetInputPortSignal(S,162);
	const real_T *A_2 = (const real_T*) ssGetInputPortSignal(S,163);
	const real_T *A_3 = (const real_T*) ssGetInputPortSignal(S,164);
	const real_T *A_4 = (const real_T*) ssGetInputPortSignal(S,165);
	const real_T *A_5 = (const real_T*) ssGetInputPortSignal(S,166);
	const real_T *A_6 = (const real_T*) ssGetInputPortSignal(S,167);
	const real_T *A_7 = (const real_T*) ssGetInputPortSignal(S,168);
	const real_T *A_8 = (const real_T*) ssGetInputPortSignal(S,169);
	const real_T *A_9 = (const real_T*) ssGetInputPortSignal(S,170);
	const real_T *A_10 = (const real_T*) ssGetInputPortSignal(S,171);
	const real_T *A_11 = (const real_T*) ssGetInputPortSignal(S,172);
	const real_T *A_12 = (const real_T*) ssGetInputPortSignal(S,173);
	const real_T *A_13 = (const real_T*) ssGetInputPortSignal(S,174);
	const real_T *A_14 = (const real_T*) ssGetInputPortSignal(S,175);
	const real_T *A_15 = (const real_T*) ssGetInputPortSignal(S,176);
	const real_T *A_16 = (const real_T*) ssGetInputPortSignal(S,177);
	const real_T *A_17 = (const real_T*) ssGetInputPortSignal(S,178);
	const real_T *A_18 = (const real_T*) ssGetInputPortSignal(S,179);
	const real_T *A_19 = (const real_T*) ssGetInputPortSignal(S,180);
	const real_T *A_20 = (const real_T*) ssGetInputPortSignal(S,181);
	const real_T *A_21 = (const real_T*) ssGetInputPortSignal(S,182);
	const real_T *A_22 = (const real_T*) ssGetInputPortSignal(S,183);
	const real_T *A_23 = (const real_T*) ssGetInputPortSignal(S,184);
	const real_T *A_24 = (const real_T*) ssGetInputPortSignal(S,185);
	const real_T *A_25 = (const real_T*) ssGetInputPortSignal(S,186);
	const real_T *A_26 = (const real_T*) ssGetInputPortSignal(S,187);
	const real_T *A_27 = (const real_T*) ssGetInputPortSignal(S,188);
	const real_T *A_28 = (const real_T*) ssGetInputPortSignal(S,189);
	const real_T *A_29 = (const real_T*) ssGetInputPortSignal(S,190);
	const real_T *A_30 = (const real_T*) ssGetInputPortSignal(S,191);
	const real_T *A_31 = (const real_T*) ssGetInputPortSignal(S,192);
	const real_T *A_32 = (const real_T*) ssGetInputPortSignal(S,193);
	const real_T *A_33 = (const real_T*) ssGetInputPortSignal(S,194);
	const real_T *A_34 = (const real_T*) ssGetInputPortSignal(S,195);
	const real_T *A_35 = (const real_T*) ssGetInputPortSignal(S,196);
	const real_T *A_36 = (const real_T*) ssGetInputPortSignal(S,197);
	const real_T *A_37 = (const real_T*) ssGetInputPortSignal(S,198);
	const real_T *A_38 = (const real_T*) ssGetInputPortSignal(S,199);
	const real_T *A_39 = (const real_T*) ssGetInputPortSignal(S,200);
	const real_T *A_40 = (const real_T*) ssGetInputPortSignal(S,201);
	const real_T *A_41 = (const real_T*) ssGetInputPortSignal(S,202);
	const real_T *b_2 = (const real_T*) ssGetInputPortSignal(S,203);
	const real_T *b_3 = (const real_T*) ssGetInputPortSignal(S,204);
	const real_T *b_4 = (const real_T*) ssGetInputPortSignal(S,205);
	const real_T *b_5 = (const real_T*) ssGetInputPortSignal(S,206);
	const real_T *b_6 = (const real_T*) ssGetInputPortSignal(S,207);
	const real_T *b_7 = (const real_T*) ssGetInputPortSignal(S,208);
	const real_T *b_8 = (const real_T*) ssGetInputPortSignal(S,209);
	const real_T *b_9 = (const real_T*) ssGetInputPortSignal(S,210);
	const real_T *b_10 = (const real_T*) ssGetInputPortSignal(S,211);
	const real_T *b_11 = (const real_T*) ssGetInputPortSignal(S,212);
	const real_T *b_12 = (const real_T*) ssGetInputPortSignal(S,213);
	const real_T *b_13 = (const real_T*) ssGetInputPortSignal(S,214);
	const real_T *b_14 = (const real_T*) ssGetInputPortSignal(S,215);
	const real_T *b_15 = (const real_T*) ssGetInputPortSignal(S,216);
	const real_T *b_16 = (const real_T*) ssGetInputPortSignal(S,217);
	const real_T *b_17 = (const real_T*) ssGetInputPortSignal(S,218);
	const real_T *b_18 = (const real_T*) ssGetInputPortSignal(S,219);
	const real_T *b_19 = (const real_T*) ssGetInputPortSignal(S,220);
	const real_T *b_20 = (const real_T*) ssGetInputPortSignal(S,221);
	const real_T *b_21 = (const real_T*) ssGetInputPortSignal(S,222);
	const real_T *b_22 = (const real_T*) ssGetInputPortSignal(S,223);
	const real_T *b_23 = (const real_T*) ssGetInputPortSignal(S,224);
	const real_T *b_24 = (const real_T*) ssGetInputPortSignal(S,225);
	const real_T *b_25 = (const real_T*) ssGetInputPortSignal(S,226);
	const real_T *b_26 = (const real_T*) ssGetInputPortSignal(S,227);
	const real_T *b_27 = (const real_T*) ssGetInputPortSignal(S,228);
	const real_T *b_28 = (const real_T*) ssGetInputPortSignal(S,229);
	const real_T *b_29 = (const real_T*) ssGetInputPortSignal(S,230);
	const real_T *b_30 = (const real_T*) ssGetInputPortSignal(S,231);
	const real_T *b_31 = (const real_T*) ssGetInputPortSignal(S,232);
	const real_T *b_32 = (const real_T*) ssGetInputPortSignal(S,233);
	const real_T *b_33 = (const real_T*) ssGetInputPortSignal(S,234);
	const real_T *b_34 = (const real_T*) ssGetInputPortSignal(S,235);
	const real_T *b_35 = (const real_T*) ssGetInputPortSignal(S,236);
	const real_T *b_36 = (const real_T*) ssGetInputPortSignal(S,237);
	const real_T *b_37 = (const real_T*) ssGetInputPortSignal(S,238);
	const real_T *b_38 = (const real_T*) ssGetInputPortSignal(S,239);
	const real_T *b_39 = (const real_T*) ssGetInputPortSignal(S,240);
	const real_T *b_40 = (const real_T*) ssGetInputPortSignal(S,241);
	const real_T *b_41 = (const real_T*) ssGetInputPortSignal(S,242);
	const solver_int32_unsigned *num_of_threads = (const solver_int32_unsigned*) ssGetInputPortSignal(S,243);
	
    real_T *X = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static MPCC_solv_40N_no_warm_no_hard_params params;
	static MPCC_solv_40N_no_warm_no_hard_output output;
	static MPCC_solv_40N_no_warm_no_hard_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<10; i++)
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

	for( i=0; i<10; i++)
	{ 
		params.c_12[i] = (double) c_12[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_13[i] = (double) c_13[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_14[i] = (double) c_14[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_15[i] = (double) c_15[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_16[i] = (double) c_16[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_17[i] = (double) c_17[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_18[i] = (double) c_18[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_19[i] = (double) c_19[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_20[i] = (double) c_20[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_21[i] = (double) c_21[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_22[i] = (double) c_22[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_23[i] = (double) c_23[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_24[i] = (double) c_24[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_25[i] = (double) c_25[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_26[i] = (double) c_26[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_27[i] = (double) c_27[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_28[i] = (double) c_28[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_29[i] = (double) c_29[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_30[i] = (double) c_30[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_31[i] = (double) c_31[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_32[i] = (double) c_32[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_33[i] = (double) c_33[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_34[i] = (double) c_34[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_35[i] = (double) c_35[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_36[i] = (double) c_36[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_37[i] = (double) c_37[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_38[i] = (double) c_38[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_39[i] = (double) c_39[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_40[i] = (double) c_40[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_41[i] = (double) c_41[i]; 
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

	for( i=0; i<196; i++)
	{ 
		params.H_12[i] = (double) H_12[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_13[i] = (double) H_13[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_14[i] = (double) H_14[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_15[i] = (double) H_15[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_16[i] = (double) H_16[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_17[i] = (double) H_17[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_18[i] = (double) H_18[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_19[i] = (double) H_19[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_20[i] = (double) H_20[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_21[i] = (double) H_21[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_22[i] = (double) H_22[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_23[i] = (double) H_23[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_24[i] = (double) H_24[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_25[i] = (double) H_25[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_26[i] = (double) H_26[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_27[i] = (double) H_27[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_28[i] = (double) H_28[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_29[i] = (double) H_29[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_30[i] = (double) H_30[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_31[i] = (double) H_31[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_32[i] = (double) H_32[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_33[i] = (double) H_33[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_34[i] = (double) H_34[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_35[i] = (double) H_35[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_36[i] = (double) H_36[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_37[i] = (double) H_37[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_38[i] = (double) H_38[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_39[i] = (double) H_39[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_40[i] = (double) H_40[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_41[i] = (double) H_41[i]; 
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

	for( i=0; i<14; i++)
	{ 
		params.f_12[i] = (double) f_12[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_13[i] = (double) f_13[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_14[i] = (double) f_14[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_15[i] = (double) f_15[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_16[i] = (double) f_16[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_17[i] = (double) f_17[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_18[i] = (double) f_18[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_19[i] = (double) f_19[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_20[i] = (double) f_20[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_21[i] = (double) f_21[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_22[i] = (double) f_22[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_23[i] = (double) f_23[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_24[i] = (double) f_24[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_25[i] = (double) f_25[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_26[i] = (double) f_26[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_27[i] = (double) f_27[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_28[i] = (double) f_28[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_29[i] = (double) f_29[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_30[i] = (double) f_30[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_31[i] = (double) f_31[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_32[i] = (double) f_32[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_33[i] = (double) f_33[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_34[i] = (double) f_34[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_35[i] = (double) f_35[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_36[i] = (double) f_36[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_37[i] = (double) f_37[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_38[i] = (double) f_38[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_39[i] = (double) f_39[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_40[i] = (double) f_40[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_41[i] = (double) f_41[i]; 
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

	for( i=0; i<140; i++)
	{ 
		params.C_11[i] = (double) C_11[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_12[i] = (double) C_12[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_13[i] = (double) C_13[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_14[i] = (double) C_14[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_15[i] = (double) C_15[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_16[i] = (double) C_16[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_17[i] = (double) C_17[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_18[i] = (double) C_18[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_19[i] = (double) C_19[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_20[i] = (double) C_20[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_21[i] = (double) C_21[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_22[i] = (double) C_22[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_23[i] = (double) C_23[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_24[i] = (double) C_24[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_25[i] = (double) C_25[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_26[i] = (double) C_26[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_27[i] = (double) C_27[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_28[i] = (double) C_28[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_29[i] = (double) C_29[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_30[i] = (double) C_30[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_31[i] = (double) C_31[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_32[i] = (double) C_32[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_33[i] = (double) C_33[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_34[i] = (double) C_34[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_35[i] = (double) C_35[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_36[i] = (double) C_36[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_37[i] = (double) C_37[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_38[i] = (double) C_38[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_39[i] = (double) C_39[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_40[i] = (double) C_40[i]; 
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

	for( i=0; i<28; i++)
	{ 
		params.A_12[i] = (double) A_12[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_13[i] = (double) A_13[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_14[i] = (double) A_14[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_15[i] = (double) A_15[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_16[i] = (double) A_16[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_17[i] = (double) A_17[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_18[i] = (double) A_18[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_19[i] = (double) A_19[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_20[i] = (double) A_20[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_21[i] = (double) A_21[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_22[i] = (double) A_22[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_23[i] = (double) A_23[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_24[i] = (double) A_24[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_25[i] = (double) A_25[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_26[i] = (double) A_26[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_27[i] = (double) A_27[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_28[i] = (double) A_28[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_29[i] = (double) A_29[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_30[i] = (double) A_30[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_31[i] = (double) A_31[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_32[i] = (double) A_32[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_33[i] = (double) A_33[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_34[i] = (double) A_34[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_35[i] = (double) A_35[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_36[i] = (double) A_36[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_37[i] = (double) A_37[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_38[i] = (double) A_38[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_39[i] = (double) A_39[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_40[i] = (double) A_40[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_41[i] = (double) A_41[i]; 
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

	for( i=0; i<2; i++)
	{ 
		params.b_12[i] = (double) b_12[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_13[i] = (double) b_13[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_14[i] = (double) b_14[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_15[i] = (double) b_15[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_16[i] = (double) b_16[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_17[i] = (double) b_17[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_18[i] = (double) b_18[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_19[i] = (double) b_19[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_20[i] = (double) b_20[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_21[i] = (double) b_21[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_22[i] = (double) b_22[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_23[i] = (double) b_23[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_24[i] = (double) b_24[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_25[i] = (double) b_25[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_26[i] = (double) b_26[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_27[i] = (double) b_27[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_28[i] = (double) b_28[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_29[i] = (double) b_29[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_30[i] = (double) b_30[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_31[i] = (double) b_31[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_32[i] = (double) b_32[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_33[i] = (double) b_33[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_34[i] = (double) b_34[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_35[i] = (double) b_35[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_36[i] = (double) b_36[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_37[i] = (double) b_37[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_38[i] = (double) b_38[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_39[i] = (double) b_39[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_40[i] = (double) b_40[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_41[i] = (double) b_41[i]; 
	}

	params.num_of_threads = *num_of_threads;

	

	

    #if SET_PRINTLEVEL_MPCC_solv_40N_no_warm_no_hard > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = MPCC_solv_40N_no_warm_no_hard_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPCC_solv_40N_no_warm_no_hard > 0
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



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
#define S_FUNCTION_NAME MPCC_solv_80N_no_warm_no_hard_invitedguest_simulinkBlock

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

	/* initialize input ports - there are 483 in total */
    if (!ssSetNumInputPorts(S, 483)) return;
    	
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
    ssSetInputPortMatrixDimensions(S,  41, 10, 1);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/
	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 10, 1);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/
	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 10, 1);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/
	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 10, 1);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/
	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 10, 1);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/
	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 10, 1);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/
	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 10, 1);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/
	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 10, 1);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/
	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 10, 1);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/
	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 10, 1);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/
	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 10, 1);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/
	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 10, 1);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/
	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 10, 1);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/
	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 10, 1);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/
	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 10, 1);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/
	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 10, 1);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/
	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 10, 1);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/
	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 10, 1);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/
	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 10, 1);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/
	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 10, 1);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/
	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 10, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/
	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 10, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/
	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 10, 1);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/
	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 10, 1);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/
	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 10, 1);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/
	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 10, 1);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/
	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 10, 1);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/
	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 10, 1);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/
	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 10, 1);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/
	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 10, 1);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/
	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 10, 1);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/
	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 10, 1);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/
	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 10, 1);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/
	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 10, 1);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/
	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 10, 1);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/
	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 10, 1);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/
	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 10, 1);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/
	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 10, 1);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/
	
	/* Input Port 79 */
    ssSetInputPortMatrixDimensions(S,  79, 10, 1);
    ssSetInputPortDataType(S, 79, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 79, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 79, 1); /*direct input signal access*/
	
	/* Input Port 80 */
    ssSetInputPortMatrixDimensions(S,  80, 10, 1);
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
    ssSetInputPortMatrixDimensions(S,  82, 14, 14);
    ssSetInputPortDataType(S, 82, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 82, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 82, 1); /*direct input signal access*/
	
	/* Input Port 83 */
    ssSetInputPortMatrixDimensions(S,  83, 14, 14);
    ssSetInputPortDataType(S, 83, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 83, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 83, 1); /*direct input signal access*/
	
	/* Input Port 84 */
    ssSetInputPortMatrixDimensions(S,  84, 14, 14);
    ssSetInputPortDataType(S, 84, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 84, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 84, 1); /*direct input signal access*/
	
	/* Input Port 85 */
    ssSetInputPortMatrixDimensions(S,  85, 14, 14);
    ssSetInputPortDataType(S, 85, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 85, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 85, 1); /*direct input signal access*/
	
	/* Input Port 86 */
    ssSetInputPortMatrixDimensions(S,  86, 14, 14);
    ssSetInputPortDataType(S, 86, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 86, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 86, 1); /*direct input signal access*/
	
	/* Input Port 87 */
    ssSetInputPortMatrixDimensions(S,  87, 14, 14);
    ssSetInputPortDataType(S, 87, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 87, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 87, 1); /*direct input signal access*/
	
	/* Input Port 88 */
    ssSetInputPortMatrixDimensions(S,  88, 14, 14);
    ssSetInputPortDataType(S, 88, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 88, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 88, 1); /*direct input signal access*/
	
	/* Input Port 89 */
    ssSetInputPortMatrixDimensions(S,  89, 14, 14);
    ssSetInputPortDataType(S, 89, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 89, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 89, 1); /*direct input signal access*/
	
	/* Input Port 90 */
    ssSetInputPortMatrixDimensions(S,  90, 14, 14);
    ssSetInputPortDataType(S, 90, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 90, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 90, 1); /*direct input signal access*/
	
	/* Input Port 91 */
    ssSetInputPortMatrixDimensions(S,  91, 14, 14);
    ssSetInputPortDataType(S, 91, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 91, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 91, 1); /*direct input signal access*/
	
	/* Input Port 92 */
    ssSetInputPortMatrixDimensions(S,  92, 14, 14);
    ssSetInputPortDataType(S, 92, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 92, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 92, 1); /*direct input signal access*/
	
	/* Input Port 93 */
    ssSetInputPortMatrixDimensions(S,  93, 14, 14);
    ssSetInputPortDataType(S, 93, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 93, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 93, 1); /*direct input signal access*/
	
	/* Input Port 94 */
    ssSetInputPortMatrixDimensions(S,  94, 14, 14);
    ssSetInputPortDataType(S, 94, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 94, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 94, 1); /*direct input signal access*/
	
	/* Input Port 95 */
    ssSetInputPortMatrixDimensions(S,  95, 14, 14);
    ssSetInputPortDataType(S, 95, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 95, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 95, 1); /*direct input signal access*/
	
	/* Input Port 96 */
    ssSetInputPortMatrixDimensions(S,  96, 14, 14);
    ssSetInputPortDataType(S, 96, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 96, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 96, 1); /*direct input signal access*/
	
	/* Input Port 97 */
    ssSetInputPortMatrixDimensions(S,  97, 14, 14);
    ssSetInputPortDataType(S, 97, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 97, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 97, 1); /*direct input signal access*/
	
	/* Input Port 98 */
    ssSetInputPortMatrixDimensions(S,  98, 14, 14);
    ssSetInputPortDataType(S, 98, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 98, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 98, 1); /*direct input signal access*/
	
	/* Input Port 99 */
    ssSetInputPortMatrixDimensions(S,  99, 14, 14);
    ssSetInputPortDataType(S, 99, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 99, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 99, 1); /*direct input signal access*/
	
	/* Input Port 100 */
    ssSetInputPortMatrixDimensions(S,  100, 14, 14);
    ssSetInputPortDataType(S, 100, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 100, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 100, 1); /*direct input signal access*/
	
	/* Input Port 101 */
    ssSetInputPortMatrixDimensions(S,  101, 14, 14);
    ssSetInputPortDataType(S, 101, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 101, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 101, 1); /*direct input signal access*/
	
	/* Input Port 102 */
    ssSetInputPortMatrixDimensions(S,  102, 14, 14);
    ssSetInputPortDataType(S, 102, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 102, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 102, 1); /*direct input signal access*/
	
	/* Input Port 103 */
    ssSetInputPortMatrixDimensions(S,  103, 14, 14);
    ssSetInputPortDataType(S, 103, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 103, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 103, 1); /*direct input signal access*/
	
	/* Input Port 104 */
    ssSetInputPortMatrixDimensions(S,  104, 14, 14);
    ssSetInputPortDataType(S, 104, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 104, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 104, 1); /*direct input signal access*/
	
	/* Input Port 105 */
    ssSetInputPortMatrixDimensions(S,  105, 14, 14);
    ssSetInputPortDataType(S, 105, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 105, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 105, 1); /*direct input signal access*/
	
	/* Input Port 106 */
    ssSetInputPortMatrixDimensions(S,  106, 14, 14);
    ssSetInputPortDataType(S, 106, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 106, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 106, 1); /*direct input signal access*/
	
	/* Input Port 107 */
    ssSetInputPortMatrixDimensions(S,  107, 14, 14);
    ssSetInputPortDataType(S, 107, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 107, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 107, 1); /*direct input signal access*/
	
	/* Input Port 108 */
    ssSetInputPortMatrixDimensions(S,  108, 14, 14);
    ssSetInputPortDataType(S, 108, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 108, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 108, 1); /*direct input signal access*/
	
	/* Input Port 109 */
    ssSetInputPortMatrixDimensions(S,  109, 14, 14);
    ssSetInputPortDataType(S, 109, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 109, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 109, 1); /*direct input signal access*/
	
	/* Input Port 110 */
    ssSetInputPortMatrixDimensions(S,  110, 14, 14);
    ssSetInputPortDataType(S, 110, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 110, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 110, 1); /*direct input signal access*/
	
	/* Input Port 111 */
    ssSetInputPortMatrixDimensions(S,  111, 14, 14);
    ssSetInputPortDataType(S, 111, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 111, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 111, 1); /*direct input signal access*/
	
	/* Input Port 112 */
    ssSetInputPortMatrixDimensions(S,  112, 14, 14);
    ssSetInputPortDataType(S, 112, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 112, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 112, 1); /*direct input signal access*/
	
	/* Input Port 113 */
    ssSetInputPortMatrixDimensions(S,  113, 14, 14);
    ssSetInputPortDataType(S, 113, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 113, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 113, 1); /*direct input signal access*/
	
	/* Input Port 114 */
    ssSetInputPortMatrixDimensions(S,  114, 14, 14);
    ssSetInputPortDataType(S, 114, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 114, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 114, 1); /*direct input signal access*/
	
	/* Input Port 115 */
    ssSetInputPortMatrixDimensions(S,  115, 14, 14);
    ssSetInputPortDataType(S, 115, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 115, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 115, 1); /*direct input signal access*/
	
	/* Input Port 116 */
    ssSetInputPortMatrixDimensions(S,  116, 14, 14);
    ssSetInputPortDataType(S, 116, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 116, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 116, 1); /*direct input signal access*/
	
	/* Input Port 117 */
    ssSetInputPortMatrixDimensions(S,  117, 14, 14);
    ssSetInputPortDataType(S, 117, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 117, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 117, 1); /*direct input signal access*/
	
	/* Input Port 118 */
    ssSetInputPortMatrixDimensions(S,  118, 14, 14);
    ssSetInputPortDataType(S, 118, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 118, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 118, 1); /*direct input signal access*/
	
	/* Input Port 119 */
    ssSetInputPortMatrixDimensions(S,  119, 14, 14);
    ssSetInputPortDataType(S, 119, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 119, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 119, 1); /*direct input signal access*/
	
	/* Input Port 120 */
    ssSetInputPortMatrixDimensions(S,  120, 14, 14);
    ssSetInputPortDataType(S, 120, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 120, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 120, 1); /*direct input signal access*/
	
	/* Input Port 121 */
    ssSetInputPortMatrixDimensions(S,  121, 14, 14);
    ssSetInputPortDataType(S, 121, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 121, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 121, 1); /*direct input signal access*/
	
	/* Input Port 122 */
    ssSetInputPortMatrixDimensions(S,  122, 14, 14);
    ssSetInputPortDataType(S, 122, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 122, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 122, 1); /*direct input signal access*/
	
	/* Input Port 123 */
    ssSetInputPortMatrixDimensions(S,  123, 14, 14);
    ssSetInputPortDataType(S, 123, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 123, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 123, 1); /*direct input signal access*/
	
	/* Input Port 124 */
    ssSetInputPortMatrixDimensions(S,  124, 14, 14);
    ssSetInputPortDataType(S, 124, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 124, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 124, 1); /*direct input signal access*/
	
	/* Input Port 125 */
    ssSetInputPortMatrixDimensions(S,  125, 14, 14);
    ssSetInputPortDataType(S, 125, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 125, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 125, 1); /*direct input signal access*/
	
	/* Input Port 126 */
    ssSetInputPortMatrixDimensions(S,  126, 14, 14);
    ssSetInputPortDataType(S, 126, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 126, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 126, 1); /*direct input signal access*/
	
	/* Input Port 127 */
    ssSetInputPortMatrixDimensions(S,  127, 14, 14);
    ssSetInputPortDataType(S, 127, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 127, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 127, 1); /*direct input signal access*/
	
	/* Input Port 128 */
    ssSetInputPortMatrixDimensions(S,  128, 14, 14);
    ssSetInputPortDataType(S, 128, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 128, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 128, 1); /*direct input signal access*/
	
	/* Input Port 129 */
    ssSetInputPortMatrixDimensions(S,  129, 14, 14);
    ssSetInputPortDataType(S, 129, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 129, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 129, 1); /*direct input signal access*/
	
	/* Input Port 130 */
    ssSetInputPortMatrixDimensions(S,  130, 14, 14);
    ssSetInputPortDataType(S, 130, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 130, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 130, 1); /*direct input signal access*/
	
	/* Input Port 131 */
    ssSetInputPortMatrixDimensions(S,  131, 14, 14);
    ssSetInputPortDataType(S, 131, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 131, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 131, 1); /*direct input signal access*/
	
	/* Input Port 132 */
    ssSetInputPortMatrixDimensions(S,  132, 14, 14);
    ssSetInputPortDataType(S, 132, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 132, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 132, 1); /*direct input signal access*/
	
	/* Input Port 133 */
    ssSetInputPortMatrixDimensions(S,  133, 14, 14);
    ssSetInputPortDataType(S, 133, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 133, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 133, 1); /*direct input signal access*/
	
	/* Input Port 134 */
    ssSetInputPortMatrixDimensions(S,  134, 14, 14);
    ssSetInputPortDataType(S, 134, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 134, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 134, 1); /*direct input signal access*/
	
	/* Input Port 135 */
    ssSetInputPortMatrixDimensions(S,  135, 14, 14);
    ssSetInputPortDataType(S, 135, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 135, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 135, 1); /*direct input signal access*/
	
	/* Input Port 136 */
    ssSetInputPortMatrixDimensions(S,  136, 14, 14);
    ssSetInputPortDataType(S, 136, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 136, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 136, 1); /*direct input signal access*/
	
	/* Input Port 137 */
    ssSetInputPortMatrixDimensions(S,  137, 14, 14);
    ssSetInputPortDataType(S, 137, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 137, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 137, 1); /*direct input signal access*/
	
	/* Input Port 138 */
    ssSetInputPortMatrixDimensions(S,  138, 14, 14);
    ssSetInputPortDataType(S, 138, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 138, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 138, 1); /*direct input signal access*/
	
	/* Input Port 139 */
    ssSetInputPortMatrixDimensions(S,  139, 14, 14);
    ssSetInputPortDataType(S, 139, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 139, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 139, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 139, 1); /*direct input signal access*/
	
	/* Input Port 140 */
    ssSetInputPortMatrixDimensions(S,  140, 14, 14);
    ssSetInputPortDataType(S, 140, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 140, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 140, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 140, 1); /*direct input signal access*/
	
	/* Input Port 141 */
    ssSetInputPortMatrixDimensions(S,  141, 14, 14);
    ssSetInputPortDataType(S, 141, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 141, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 141, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 141, 1); /*direct input signal access*/
	
	/* Input Port 142 */
    ssSetInputPortMatrixDimensions(S,  142, 14, 14);
    ssSetInputPortDataType(S, 142, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 142, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 142, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 142, 1); /*direct input signal access*/
	
	/* Input Port 143 */
    ssSetInputPortMatrixDimensions(S,  143, 14, 14);
    ssSetInputPortDataType(S, 143, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 143, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 143, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 143, 1); /*direct input signal access*/
	
	/* Input Port 144 */
    ssSetInputPortMatrixDimensions(S,  144, 14, 14);
    ssSetInputPortDataType(S, 144, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 144, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 144, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 144, 1); /*direct input signal access*/
	
	/* Input Port 145 */
    ssSetInputPortMatrixDimensions(S,  145, 14, 14);
    ssSetInputPortDataType(S, 145, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 145, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 145, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 145, 1); /*direct input signal access*/
	
	/* Input Port 146 */
    ssSetInputPortMatrixDimensions(S,  146, 14, 14);
    ssSetInputPortDataType(S, 146, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 146, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 146, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 146, 1); /*direct input signal access*/
	
	/* Input Port 147 */
    ssSetInputPortMatrixDimensions(S,  147, 14, 14);
    ssSetInputPortDataType(S, 147, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 147, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 147, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 147, 1); /*direct input signal access*/
	
	/* Input Port 148 */
    ssSetInputPortMatrixDimensions(S,  148, 14, 14);
    ssSetInputPortDataType(S, 148, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 148, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 148, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 148, 1); /*direct input signal access*/
	
	/* Input Port 149 */
    ssSetInputPortMatrixDimensions(S,  149, 14, 14);
    ssSetInputPortDataType(S, 149, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 149, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 149, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 149, 1); /*direct input signal access*/
	
	/* Input Port 150 */
    ssSetInputPortMatrixDimensions(S,  150, 14, 14);
    ssSetInputPortDataType(S, 150, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 150, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 150, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 150, 1); /*direct input signal access*/
	
	/* Input Port 151 */
    ssSetInputPortMatrixDimensions(S,  151, 14, 14);
    ssSetInputPortDataType(S, 151, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 151, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 151, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 151, 1); /*direct input signal access*/
	
	/* Input Port 152 */
    ssSetInputPortMatrixDimensions(S,  152, 14, 14);
    ssSetInputPortDataType(S, 152, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 152, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 152, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 152, 1); /*direct input signal access*/
	
	/* Input Port 153 */
    ssSetInputPortMatrixDimensions(S,  153, 14, 14);
    ssSetInputPortDataType(S, 153, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 153, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 153, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 153, 1); /*direct input signal access*/
	
	/* Input Port 154 */
    ssSetInputPortMatrixDimensions(S,  154, 14, 14);
    ssSetInputPortDataType(S, 154, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 154, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 154, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 154, 1); /*direct input signal access*/
	
	/* Input Port 155 */
    ssSetInputPortMatrixDimensions(S,  155, 14, 14);
    ssSetInputPortDataType(S, 155, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 155, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 155, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 155, 1); /*direct input signal access*/
	
	/* Input Port 156 */
    ssSetInputPortMatrixDimensions(S,  156, 14, 14);
    ssSetInputPortDataType(S, 156, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 156, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 156, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 156, 1); /*direct input signal access*/
	
	/* Input Port 157 */
    ssSetInputPortMatrixDimensions(S,  157, 14, 14);
    ssSetInputPortDataType(S, 157, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 157, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 157, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 157, 1); /*direct input signal access*/
	
	/* Input Port 158 */
    ssSetInputPortMatrixDimensions(S,  158, 14, 14);
    ssSetInputPortDataType(S, 158, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 158, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 158, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 158, 1); /*direct input signal access*/
	
	/* Input Port 159 */
    ssSetInputPortMatrixDimensions(S,  159, 14, 14);
    ssSetInputPortDataType(S, 159, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 159, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 159, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 159, 1); /*direct input signal access*/
	
	/* Input Port 160 */
    ssSetInputPortMatrixDimensions(S,  160, 14, 14);
    ssSetInputPortDataType(S, 160, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 160, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 160, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 160, 1); /*direct input signal access*/
	
	/* Input Port 161 */
    ssSetInputPortMatrixDimensions(S,  161, 14, 14);
    ssSetInputPortDataType(S, 161, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 161, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 161, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 161, 1); /*direct input signal access*/
	
	/* Input Port 162 */
    ssSetInputPortMatrixDimensions(S,  162, 14, 1);
    ssSetInputPortDataType(S, 162, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 162, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 162, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 162, 1); /*direct input signal access*/
	
	/* Input Port 163 */
    ssSetInputPortMatrixDimensions(S,  163, 14, 1);
    ssSetInputPortDataType(S, 163, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 163, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 163, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 163, 1); /*direct input signal access*/
	
	/* Input Port 164 */
    ssSetInputPortMatrixDimensions(S,  164, 14, 1);
    ssSetInputPortDataType(S, 164, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 164, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 164, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 164, 1); /*direct input signal access*/
	
	/* Input Port 165 */
    ssSetInputPortMatrixDimensions(S,  165, 14, 1);
    ssSetInputPortDataType(S, 165, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 165, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 165, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 165, 1); /*direct input signal access*/
	
	/* Input Port 166 */
    ssSetInputPortMatrixDimensions(S,  166, 14, 1);
    ssSetInputPortDataType(S, 166, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 166, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 166, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 166, 1); /*direct input signal access*/
	
	/* Input Port 167 */
    ssSetInputPortMatrixDimensions(S,  167, 14, 1);
    ssSetInputPortDataType(S, 167, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 167, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 167, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 167, 1); /*direct input signal access*/
	
	/* Input Port 168 */
    ssSetInputPortMatrixDimensions(S,  168, 14, 1);
    ssSetInputPortDataType(S, 168, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 168, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 168, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 168, 1); /*direct input signal access*/
	
	/* Input Port 169 */
    ssSetInputPortMatrixDimensions(S,  169, 14, 1);
    ssSetInputPortDataType(S, 169, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 169, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 169, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 169, 1); /*direct input signal access*/
	
	/* Input Port 170 */
    ssSetInputPortMatrixDimensions(S,  170, 14, 1);
    ssSetInputPortDataType(S, 170, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 170, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 170, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 170, 1); /*direct input signal access*/
	
	/* Input Port 171 */
    ssSetInputPortMatrixDimensions(S,  171, 14, 1);
    ssSetInputPortDataType(S, 171, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 171, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 171, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 171, 1); /*direct input signal access*/
	
	/* Input Port 172 */
    ssSetInputPortMatrixDimensions(S,  172, 14, 1);
    ssSetInputPortDataType(S, 172, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 172, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 172, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 172, 1); /*direct input signal access*/
	
	/* Input Port 173 */
    ssSetInputPortMatrixDimensions(S,  173, 14, 1);
    ssSetInputPortDataType(S, 173, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 173, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 173, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 173, 1); /*direct input signal access*/
	
	/* Input Port 174 */
    ssSetInputPortMatrixDimensions(S,  174, 14, 1);
    ssSetInputPortDataType(S, 174, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 174, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 174, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 174, 1); /*direct input signal access*/
	
	/* Input Port 175 */
    ssSetInputPortMatrixDimensions(S,  175, 14, 1);
    ssSetInputPortDataType(S, 175, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 175, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 175, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 175, 1); /*direct input signal access*/
	
	/* Input Port 176 */
    ssSetInputPortMatrixDimensions(S,  176, 14, 1);
    ssSetInputPortDataType(S, 176, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 176, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 176, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 176, 1); /*direct input signal access*/
	
	/* Input Port 177 */
    ssSetInputPortMatrixDimensions(S,  177, 14, 1);
    ssSetInputPortDataType(S, 177, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 177, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 177, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 177, 1); /*direct input signal access*/
	
	/* Input Port 178 */
    ssSetInputPortMatrixDimensions(S,  178, 14, 1);
    ssSetInputPortDataType(S, 178, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 178, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 178, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 178, 1); /*direct input signal access*/
	
	/* Input Port 179 */
    ssSetInputPortMatrixDimensions(S,  179, 14, 1);
    ssSetInputPortDataType(S, 179, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 179, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 179, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 179, 1); /*direct input signal access*/
	
	/* Input Port 180 */
    ssSetInputPortMatrixDimensions(S,  180, 14, 1);
    ssSetInputPortDataType(S, 180, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 180, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 180, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 180, 1); /*direct input signal access*/
	
	/* Input Port 181 */
    ssSetInputPortMatrixDimensions(S,  181, 14, 1);
    ssSetInputPortDataType(S, 181, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 181, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 181, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 181, 1); /*direct input signal access*/
	
	/* Input Port 182 */
    ssSetInputPortMatrixDimensions(S,  182, 14, 1);
    ssSetInputPortDataType(S, 182, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 182, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 182, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 182, 1); /*direct input signal access*/
	
	/* Input Port 183 */
    ssSetInputPortMatrixDimensions(S,  183, 14, 1);
    ssSetInputPortDataType(S, 183, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 183, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 183, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 183, 1); /*direct input signal access*/
	
	/* Input Port 184 */
    ssSetInputPortMatrixDimensions(S,  184, 14, 1);
    ssSetInputPortDataType(S, 184, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 184, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 184, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 184, 1); /*direct input signal access*/
	
	/* Input Port 185 */
    ssSetInputPortMatrixDimensions(S,  185, 14, 1);
    ssSetInputPortDataType(S, 185, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 185, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 185, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 185, 1); /*direct input signal access*/
	
	/* Input Port 186 */
    ssSetInputPortMatrixDimensions(S,  186, 14, 1);
    ssSetInputPortDataType(S, 186, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 186, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 186, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 186, 1); /*direct input signal access*/
	
	/* Input Port 187 */
    ssSetInputPortMatrixDimensions(S,  187, 14, 1);
    ssSetInputPortDataType(S, 187, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 187, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 187, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 187, 1); /*direct input signal access*/
	
	/* Input Port 188 */
    ssSetInputPortMatrixDimensions(S,  188, 14, 1);
    ssSetInputPortDataType(S, 188, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 188, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 188, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 188, 1); /*direct input signal access*/
	
	/* Input Port 189 */
    ssSetInputPortMatrixDimensions(S,  189, 14, 1);
    ssSetInputPortDataType(S, 189, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 189, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 189, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 189, 1); /*direct input signal access*/
	
	/* Input Port 190 */
    ssSetInputPortMatrixDimensions(S,  190, 14, 1);
    ssSetInputPortDataType(S, 190, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 190, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 190, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 190, 1); /*direct input signal access*/
	
	/* Input Port 191 */
    ssSetInputPortMatrixDimensions(S,  191, 14, 1);
    ssSetInputPortDataType(S, 191, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 191, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 191, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 191, 1); /*direct input signal access*/
	
	/* Input Port 192 */
    ssSetInputPortMatrixDimensions(S,  192, 14, 1);
    ssSetInputPortDataType(S, 192, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 192, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 192, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 192, 1); /*direct input signal access*/
	
	/* Input Port 193 */
    ssSetInputPortMatrixDimensions(S,  193, 14, 1);
    ssSetInputPortDataType(S, 193, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 193, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 193, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 193, 1); /*direct input signal access*/
	
	/* Input Port 194 */
    ssSetInputPortMatrixDimensions(S,  194, 14, 1);
    ssSetInputPortDataType(S, 194, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 194, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 194, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 194, 1); /*direct input signal access*/
	
	/* Input Port 195 */
    ssSetInputPortMatrixDimensions(S,  195, 14, 1);
    ssSetInputPortDataType(S, 195, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 195, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 195, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 195, 1); /*direct input signal access*/
	
	/* Input Port 196 */
    ssSetInputPortMatrixDimensions(S,  196, 14, 1);
    ssSetInputPortDataType(S, 196, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 196, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 196, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 196, 1); /*direct input signal access*/
	
	/* Input Port 197 */
    ssSetInputPortMatrixDimensions(S,  197, 14, 1);
    ssSetInputPortDataType(S, 197, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 197, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 197, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 197, 1); /*direct input signal access*/
	
	/* Input Port 198 */
    ssSetInputPortMatrixDimensions(S,  198, 14, 1);
    ssSetInputPortDataType(S, 198, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 198, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 198, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 198, 1); /*direct input signal access*/
	
	/* Input Port 199 */
    ssSetInputPortMatrixDimensions(S,  199, 14, 1);
    ssSetInputPortDataType(S, 199, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 199, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 199, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 199, 1); /*direct input signal access*/
	
	/* Input Port 200 */
    ssSetInputPortMatrixDimensions(S,  200, 14, 1);
    ssSetInputPortDataType(S, 200, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 200, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 200, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 200, 1); /*direct input signal access*/
	
	/* Input Port 201 */
    ssSetInputPortMatrixDimensions(S,  201, 14, 1);
    ssSetInputPortDataType(S, 201, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 201, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 201, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 201, 1); /*direct input signal access*/
	
	/* Input Port 202 */
    ssSetInputPortMatrixDimensions(S,  202, 14, 1);
    ssSetInputPortDataType(S, 202, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 202, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 202, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 202, 1); /*direct input signal access*/
	
	/* Input Port 203 */
    ssSetInputPortMatrixDimensions(S,  203, 14, 1);
    ssSetInputPortDataType(S, 203, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 203, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 203, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 203, 1); /*direct input signal access*/
	
	/* Input Port 204 */
    ssSetInputPortMatrixDimensions(S,  204, 14, 1);
    ssSetInputPortDataType(S, 204, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 204, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 204, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 204, 1); /*direct input signal access*/
	
	/* Input Port 205 */
    ssSetInputPortMatrixDimensions(S,  205, 14, 1);
    ssSetInputPortDataType(S, 205, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 205, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 205, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 205, 1); /*direct input signal access*/
	
	/* Input Port 206 */
    ssSetInputPortMatrixDimensions(S,  206, 14, 1);
    ssSetInputPortDataType(S, 206, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 206, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 206, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 206, 1); /*direct input signal access*/
	
	/* Input Port 207 */
    ssSetInputPortMatrixDimensions(S,  207, 14, 1);
    ssSetInputPortDataType(S, 207, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 207, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 207, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 207, 1); /*direct input signal access*/
	
	/* Input Port 208 */
    ssSetInputPortMatrixDimensions(S,  208, 14, 1);
    ssSetInputPortDataType(S, 208, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 208, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 208, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 208, 1); /*direct input signal access*/
	
	/* Input Port 209 */
    ssSetInputPortMatrixDimensions(S,  209, 14, 1);
    ssSetInputPortDataType(S, 209, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 209, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 209, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 209, 1); /*direct input signal access*/
	
	/* Input Port 210 */
    ssSetInputPortMatrixDimensions(S,  210, 14, 1);
    ssSetInputPortDataType(S, 210, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 210, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 210, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 210, 1); /*direct input signal access*/
	
	/* Input Port 211 */
    ssSetInputPortMatrixDimensions(S,  211, 14, 1);
    ssSetInputPortDataType(S, 211, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 211, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 211, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 211, 1); /*direct input signal access*/
	
	/* Input Port 212 */
    ssSetInputPortMatrixDimensions(S,  212, 14, 1);
    ssSetInputPortDataType(S, 212, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 212, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 212, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 212, 1); /*direct input signal access*/
	
	/* Input Port 213 */
    ssSetInputPortMatrixDimensions(S,  213, 14, 1);
    ssSetInputPortDataType(S, 213, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 213, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 213, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 213, 1); /*direct input signal access*/
	
	/* Input Port 214 */
    ssSetInputPortMatrixDimensions(S,  214, 14, 1);
    ssSetInputPortDataType(S, 214, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 214, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 214, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 214, 1); /*direct input signal access*/
	
	/* Input Port 215 */
    ssSetInputPortMatrixDimensions(S,  215, 14, 1);
    ssSetInputPortDataType(S, 215, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 215, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 215, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 215, 1); /*direct input signal access*/
	
	/* Input Port 216 */
    ssSetInputPortMatrixDimensions(S,  216, 14, 1);
    ssSetInputPortDataType(S, 216, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 216, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 216, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 216, 1); /*direct input signal access*/
	
	/* Input Port 217 */
    ssSetInputPortMatrixDimensions(S,  217, 14, 1);
    ssSetInputPortDataType(S, 217, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 217, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 217, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 217, 1); /*direct input signal access*/
	
	/* Input Port 218 */
    ssSetInputPortMatrixDimensions(S,  218, 14, 1);
    ssSetInputPortDataType(S, 218, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 218, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 218, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 218, 1); /*direct input signal access*/
	
	/* Input Port 219 */
    ssSetInputPortMatrixDimensions(S,  219, 14, 1);
    ssSetInputPortDataType(S, 219, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 219, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 219, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 219, 1); /*direct input signal access*/
	
	/* Input Port 220 */
    ssSetInputPortMatrixDimensions(S,  220, 14, 1);
    ssSetInputPortDataType(S, 220, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 220, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 220, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 220, 1); /*direct input signal access*/
	
	/* Input Port 221 */
    ssSetInputPortMatrixDimensions(S,  221, 14, 1);
    ssSetInputPortDataType(S, 221, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 221, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 221, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 221, 1); /*direct input signal access*/
	
	/* Input Port 222 */
    ssSetInputPortMatrixDimensions(S,  222, 14, 1);
    ssSetInputPortDataType(S, 222, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 222, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 222, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 222, 1); /*direct input signal access*/
	
	/* Input Port 223 */
    ssSetInputPortMatrixDimensions(S,  223, 14, 1);
    ssSetInputPortDataType(S, 223, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 223, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 223, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 223, 1); /*direct input signal access*/
	
	/* Input Port 224 */
    ssSetInputPortMatrixDimensions(S,  224, 14, 1);
    ssSetInputPortDataType(S, 224, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 224, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 224, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 224, 1); /*direct input signal access*/
	
	/* Input Port 225 */
    ssSetInputPortMatrixDimensions(S,  225, 14, 1);
    ssSetInputPortDataType(S, 225, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 225, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 225, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 225, 1); /*direct input signal access*/
	
	/* Input Port 226 */
    ssSetInputPortMatrixDimensions(S,  226, 14, 1);
    ssSetInputPortDataType(S, 226, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 226, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 226, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 226, 1); /*direct input signal access*/
	
	/* Input Port 227 */
    ssSetInputPortMatrixDimensions(S,  227, 14, 1);
    ssSetInputPortDataType(S, 227, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 227, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 227, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 227, 1); /*direct input signal access*/
	
	/* Input Port 228 */
    ssSetInputPortMatrixDimensions(S,  228, 14, 1);
    ssSetInputPortDataType(S, 228, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 228, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 228, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 228, 1); /*direct input signal access*/
	
	/* Input Port 229 */
    ssSetInputPortMatrixDimensions(S,  229, 14, 1);
    ssSetInputPortDataType(S, 229, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 229, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 229, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 229, 1); /*direct input signal access*/
	
	/* Input Port 230 */
    ssSetInputPortMatrixDimensions(S,  230, 14, 1);
    ssSetInputPortDataType(S, 230, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 230, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 230, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 230, 1); /*direct input signal access*/
	
	/* Input Port 231 */
    ssSetInputPortMatrixDimensions(S,  231, 14, 1);
    ssSetInputPortDataType(S, 231, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 231, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 231, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 231, 1); /*direct input signal access*/
	
	/* Input Port 232 */
    ssSetInputPortMatrixDimensions(S,  232, 14, 1);
    ssSetInputPortDataType(S, 232, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 232, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 232, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 232, 1); /*direct input signal access*/
	
	/* Input Port 233 */
    ssSetInputPortMatrixDimensions(S,  233, 14, 1);
    ssSetInputPortDataType(S, 233, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 233, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 233, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 233, 1); /*direct input signal access*/
	
	/* Input Port 234 */
    ssSetInputPortMatrixDimensions(S,  234, 14, 1);
    ssSetInputPortDataType(S, 234, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 234, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 234, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 234, 1); /*direct input signal access*/
	
	/* Input Port 235 */
    ssSetInputPortMatrixDimensions(S,  235, 14, 1);
    ssSetInputPortDataType(S, 235, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 235, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 235, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 235, 1); /*direct input signal access*/
	
	/* Input Port 236 */
    ssSetInputPortMatrixDimensions(S,  236, 14, 1);
    ssSetInputPortDataType(S, 236, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 236, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 236, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 236, 1); /*direct input signal access*/
	
	/* Input Port 237 */
    ssSetInputPortMatrixDimensions(S,  237, 14, 1);
    ssSetInputPortDataType(S, 237, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 237, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 237, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 237, 1); /*direct input signal access*/
	
	/* Input Port 238 */
    ssSetInputPortMatrixDimensions(S,  238, 14, 1);
    ssSetInputPortDataType(S, 238, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 238, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 238, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 238, 1); /*direct input signal access*/
	
	/* Input Port 239 */
    ssSetInputPortMatrixDimensions(S,  239, 14, 1);
    ssSetInputPortDataType(S, 239, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 239, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 239, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 239, 1); /*direct input signal access*/
	
	/* Input Port 240 */
    ssSetInputPortMatrixDimensions(S,  240, 14, 1);
    ssSetInputPortDataType(S, 240, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 240, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 240, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 240, 1); /*direct input signal access*/
	
	/* Input Port 241 */
    ssSetInputPortMatrixDimensions(S,  241, 14, 1);
    ssSetInputPortDataType(S, 241, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 241, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 241, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 241, 1); /*direct input signal access*/
	
	/* Input Port 242 */
    ssSetInputPortMatrixDimensions(S,  242, 14, 1);
    ssSetInputPortDataType(S, 242, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 242, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 242, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 242, 1); /*direct input signal access*/
	
	/* Input Port 243 */
    ssSetInputPortMatrixDimensions(S,  243, 10, 14);
    ssSetInputPortDataType(S, 243, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 243, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 243, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 243, 1); /*direct input signal access*/
	
	/* Input Port 244 */
    ssSetInputPortMatrixDimensions(S,  244, 10, 14);
    ssSetInputPortDataType(S, 244, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 244, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 244, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 244, 1); /*direct input signal access*/
	
	/* Input Port 245 */
    ssSetInputPortMatrixDimensions(S,  245, 10, 14);
    ssSetInputPortDataType(S, 245, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 245, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 245, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 245, 1); /*direct input signal access*/
	
	/* Input Port 246 */
    ssSetInputPortMatrixDimensions(S,  246, 10, 14);
    ssSetInputPortDataType(S, 246, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 246, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 246, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 246, 1); /*direct input signal access*/
	
	/* Input Port 247 */
    ssSetInputPortMatrixDimensions(S,  247, 10, 14);
    ssSetInputPortDataType(S, 247, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 247, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 247, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 247, 1); /*direct input signal access*/
	
	/* Input Port 248 */
    ssSetInputPortMatrixDimensions(S,  248, 10, 14);
    ssSetInputPortDataType(S, 248, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 248, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 248, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 248, 1); /*direct input signal access*/
	
	/* Input Port 249 */
    ssSetInputPortMatrixDimensions(S,  249, 10, 14);
    ssSetInputPortDataType(S, 249, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 249, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 249, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 249, 1); /*direct input signal access*/
	
	/* Input Port 250 */
    ssSetInputPortMatrixDimensions(S,  250, 10, 14);
    ssSetInputPortDataType(S, 250, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 250, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 250, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 250, 1); /*direct input signal access*/
	
	/* Input Port 251 */
    ssSetInputPortMatrixDimensions(S,  251, 10, 14);
    ssSetInputPortDataType(S, 251, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 251, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 251, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 251, 1); /*direct input signal access*/
	
	/* Input Port 252 */
    ssSetInputPortMatrixDimensions(S,  252, 10, 14);
    ssSetInputPortDataType(S, 252, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 252, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 252, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 252, 1); /*direct input signal access*/
	
	/* Input Port 253 */
    ssSetInputPortMatrixDimensions(S,  253, 10, 14);
    ssSetInputPortDataType(S, 253, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 253, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 253, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 253, 1); /*direct input signal access*/
	
	/* Input Port 254 */
    ssSetInputPortMatrixDimensions(S,  254, 10, 14);
    ssSetInputPortDataType(S, 254, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 254, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 254, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 254, 1); /*direct input signal access*/
	
	/* Input Port 255 */
    ssSetInputPortMatrixDimensions(S,  255, 10, 14);
    ssSetInputPortDataType(S, 255, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 255, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 255, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 255, 1); /*direct input signal access*/
	
	/* Input Port 256 */
    ssSetInputPortMatrixDimensions(S,  256, 10, 14);
    ssSetInputPortDataType(S, 256, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 256, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 256, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 256, 1); /*direct input signal access*/
	
	/* Input Port 257 */
    ssSetInputPortMatrixDimensions(S,  257, 10, 14);
    ssSetInputPortDataType(S, 257, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 257, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 257, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 257, 1); /*direct input signal access*/
	
	/* Input Port 258 */
    ssSetInputPortMatrixDimensions(S,  258, 10, 14);
    ssSetInputPortDataType(S, 258, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 258, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 258, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 258, 1); /*direct input signal access*/
	
	/* Input Port 259 */
    ssSetInputPortMatrixDimensions(S,  259, 10, 14);
    ssSetInputPortDataType(S, 259, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 259, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 259, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 259, 1); /*direct input signal access*/
	
	/* Input Port 260 */
    ssSetInputPortMatrixDimensions(S,  260, 10, 14);
    ssSetInputPortDataType(S, 260, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 260, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 260, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 260, 1); /*direct input signal access*/
	
	/* Input Port 261 */
    ssSetInputPortMatrixDimensions(S,  261, 10, 14);
    ssSetInputPortDataType(S, 261, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 261, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 261, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 261, 1); /*direct input signal access*/
	
	/* Input Port 262 */
    ssSetInputPortMatrixDimensions(S,  262, 10, 14);
    ssSetInputPortDataType(S, 262, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 262, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 262, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 262, 1); /*direct input signal access*/
	
	/* Input Port 263 */
    ssSetInputPortMatrixDimensions(S,  263, 10, 14);
    ssSetInputPortDataType(S, 263, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 263, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 263, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 263, 1); /*direct input signal access*/
	
	/* Input Port 264 */
    ssSetInputPortMatrixDimensions(S,  264, 10, 14);
    ssSetInputPortDataType(S, 264, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 264, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 264, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 264, 1); /*direct input signal access*/
	
	/* Input Port 265 */
    ssSetInputPortMatrixDimensions(S,  265, 10, 14);
    ssSetInputPortDataType(S, 265, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 265, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 265, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 265, 1); /*direct input signal access*/
	
	/* Input Port 266 */
    ssSetInputPortMatrixDimensions(S,  266, 10, 14);
    ssSetInputPortDataType(S, 266, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 266, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 266, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 266, 1); /*direct input signal access*/
	
	/* Input Port 267 */
    ssSetInputPortMatrixDimensions(S,  267, 10, 14);
    ssSetInputPortDataType(S, 267, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 267, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 267, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 267, 1); /*direct input signal access*/
	
	/* Input Port 268 */
    ssSetInputPortMatrixDimensions(S,  268, 10, 14);
    ssSetInputPortDataType(S, 268, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 268, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 268, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 268, 1); /*direct input signal access*/
	
	/* Input Port 269 */
    ssSetInputPortMatrixDimensions(S,  269, 10, 14);
    ssSetInputPortDataType(S, 269, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 269, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 269, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 269, 1); /*direct input signal access*/
	
	/* Input Port 270 */
    ssSetInputPortMatrixDimensions(S,  270, 10, 14);
    ssSetInputPortDataType(S, 270, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 270, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 270, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 270, 1); /*direct input signal access*/
	
	/* Input Port 271 */
    ssSetInputPortMatrixDimensions(S,  271, 10, 14);
    ssSetInputPortDataType(S, 271, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 271, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 271, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 271, 1); /*direct input signal access*/
	
	/* Input Port 272 */
    ssSetInputPortMatrixDimensions(S,  272, 10, 14);
    ssSetInputPortDataType(S, 272, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 272, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 272, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 272, 1); /*direct input signal access*/
	
	/* Input Port 273 */
    ssSetInputPortMatrixDimensions(S,  273, 10, 14);
    ssSetInputPortDataType(S, 273, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 273, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 273, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 273, 1); /*direct input signal access*/
	
	/* Input Port 274 */
    ssSetInputPortMatrixDimensions(S,  274, 10, 14);
    ssSetInputPortDataType(S, 274, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 274, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 274, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 274, 1); /*direct input signal access*/
	
	/* Input Port 275 */
    ssSetInputPortMatrixDimensions(S,  275, 10, 14);
    ssSetInputPortDataType(S, 275, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 275, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 275, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 275, 1); /*direct input signal access*/
	
	/* Input Port 276 */
    ssSetInputPortMatrixDimensions(S,  276, 10, 14);
    ssSetInputPortDataType(S, 276, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 276, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 276, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 276, 1); /*direct input signal access*/
	
	/* Input Port 277 */
    ssSetInputPortMatrixDimensions(S,  277, 10, 14);
    ssSetInputPortDataType(S, 277, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 277, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 277, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 277, 1); /*direct input signal access*/
	
	/* Input Port 278 */
    ssSetInputPortMatrixDimensions(S,  278, 10, 14);
    ssSetInputPortDataType(S, 278, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 278, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 278, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 278, 1); /*direct input signal access*/
	
	/* Input Port 279 */
    ssSetInputPortMatrixDimensions(S,  279, 10, 14);
    ssSetInputPortDataType(S, 279, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 279, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 279, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 279, 1); /*direct input signal access*/
	
	/* Input Port 280 */
    ssSetInputPortMatrixDimensions(S,  280, 10, 14);
    ssSetInputPortDataType(S, 280, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 280, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 280, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 280, 1); /*direct input signal access*/
	
	/* Input Port 281 */
    ssSetInputPortMatrixDimensions(S,  281, 10, 14);
    ssSetInputPortDataType(S, 281, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 281, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 281, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 281, 1); /*direct input signal access*/
	
	/* Input Port 282 */
    ssSetInputPortMatrixDimensions(S,  282, 10, 14);
    ssSetInputPortDataType(S, 282, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 282, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 282, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 282, 1); /*direct input signal access*/
	
	/* Input Port 283 */
    ssSetInputPortMatrixDimensions(S,  283, 10, 14);
    ssSetInputPortDataType(S, 283, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 283, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 283, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 283, 1); /*direct input signal access*/
	
	/* Input Port 284 */
    ssSetInputPortMatrixDimensions(S,  284, 10, 14);
    ssSetInputPortDataType(S, 284, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 284, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 284, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 284, 1); /*direct input signal access*/
	
	/* Input Port 285 */
    ssSetInputPortMatrixDimensions(S,  285, 10, 14);
    ssSetInputPortDataType(S, 285, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 285, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 285, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 285, 1); /*direct input signal access*/
	
	/* Input Port 286 */
    ssSetInputPortMatrixDimensions(S,  286, 10, 14);
    ssSetInputPortDataType(S, 286, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 286, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 286, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 286, 1); /*direct input signal access*/
	
	/* Input Port 287 */
    ssSetInputPortMatrixDimensions(S,  287, 10, 14);
    ssSetInputPortDataType(S, 287, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 287, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 287, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 287, 1); /*direct input signal access*/
	
	/* Input Port 288 */
    ssSetInputPortMatrixDimensions(S,  288, 10, 14);
    ssSetInputPortDataType(S, 288, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 288, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 288, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 288, 1); /*direct input signal access*/
	
	/* Input Port 289 */
    ssSetInputPortMatrixDimensions(S,  289, 10, 14);
    ssSetInputPortDataType(S, 289, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 289, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 289, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 289, 1); /*direct input signal access*/
	
	/* Input Port 290 */
    ssSetInputPortMatrixDimensions(S,  290, 10, 14);
    ssSetInputPortDataType(S, 290, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 290, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 290, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 290, 1); /*direct input signal access*/
	
	/* Input Port 291 */
    ssSetInputPortMatrixDimensions(S,  291, 10, 14);
    ssSetInputPortDataType(S, 291, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 291, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 291, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 291, 1); /*direct input signal access*/
	
	/* Input Port 292 */
    ssSetInputPortMatrixDimensions(S,  292, 10, 14);
    ssSetInputPortDataType(S, 292, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 292, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 292, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 292, 1); /*direct input signal access*/
	
	/* Input Port 293 */
    ssSetInputPortMatrixDimensions(S,  293, 10, 14);
    ssSetInputPortDataType(S, 293, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 293, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 293, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 293, 1); /*direct input signal access*/
	
	/* Input Port 294 */
    ssSetInputPortMatrixDimensions(S,  294, 10, 14);
    ssSetInputPortDataType(S, 294, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 294, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 294, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 294, 1); /*direct input signal access*/
	
	/* Input Port 295 */
    ssSetInputPortMatrixDimensions(S,  295, 10, 14);
    ssSetInputPortDataType(S, 295, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 295, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 295, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 295, 1); /*direct input signal access*/
	
	/* Input Port 296 */
    ssSetInputPortMatrixDimensions(S,  296, 10, 14);
    ssSetInputPortDataType(S, 296, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 296, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 296, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 296, 1); /*direct input signal access*/
	
	/* Input Port 297 */
    ssSetInputPortMatrixDimensions(S,  297, 10, 14);
    ssSetInputPortDataType(S, 297, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 297, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 297, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 297, 1); /*direct input signal access*/
	
	/* Input Port 298 */
    ssSetInputPortMatrixDimensions(S,  298, 10, 14);
    ssSetInputPortDataType(S, 298, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 298, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 298, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 298, 1); /*direct input signal access*/
	
	/* Input Port 299 */
    ssSetInputPortMatrixDimensions(S,  299, 10, 14);
    ssSetInputPortDataType(S, 299, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 299, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 299, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 299, 1); /*direct input signal access*/
	
	/* Input Port 300 */
    ssSetInputPortMatrixDimensions(S,  300, 10, 14);
    ssSetInputPortDataType(S, 300, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 300, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 300, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 300, 1); /*direct input signal access*/
	
	/* Input Port 301 */
    ssSetInputPortMatrixDimensions(S,  301, 10, 14);
    ssSetInputPortDataType(S, 301, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 301, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 301, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 301, 1); /*direct input signal access*/
	
	/* Input Port 302 */
    ssSetInputPortMatrixDimensions(S,  302, 10, 14);
    ssSetInputPortDataType(S, 302, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 302, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 302, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 302, 1); /*direct input signal access*/
	
	/* Input Port 303 */
    ssSetInputPortMatrixDimensions(S,  303, 10, 14);
    ssSetInputPortDataType(S, 303, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 303, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 303, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 303, 1); /*direct input signal access*/
	
	/* Input Port 304 */
    ssSetInputPortMatrixDimensions(S,  304, 10, 14);
    ssSetInputPortDataType(S, 304, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 304, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 304, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 304, 1); /*direct input signal access*/
	
	/* Input Port 305 */
    ssSetInputPortMatrixDimensions(S,  305, 10, 14);
    ssSetInputPortDataType(S, 305, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 305, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 305, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 305, 1); /*direct input signal access*/
	
	/* Input Port 306 */
    ssSetInputPortMatrixDimensions(S,  306, 10, 14);
    ssSetInputPortDataType(S, 306, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 306, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 306, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 306, 1); /*direct input signal access*/
	
	/* Input Port 307 */
    ssSetInputPortMatrixDimensions(S,  307, 10, 14);
    ssSetInputPortDataType(S, 307, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 307, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 307, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 307, 1); /*direct input signal access*/
	
	/* Input Port 308 */
    ssSetInputPortMatrixDimensions(S,  308, 10, 14);
    ssSetInputPortDataType(S, 308, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 308, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 308, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 308, 1); /*direct input signal access*/
	
	/* Input Port 309 */
    ssSetInputPortMatrixDimensions(S,  309, 10, 14);
    ssSetInputPortDataType(S, 309, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 309, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 309, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 309, 1); /*direct input signal access*/
	
	/* Input Port 310 */
    ssSetInputPortMatrixDimensions(S,  310, 10, 14);
    ssSetInputPortDataType(S, 310, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 310, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 310, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 310, 1); /*direct input signal access*/
	
	/* Input Port 311 */
    ssSetInputPortMatrixDimensions(S,  311, 10, 14);
    ssSetInputPortDataType(S, 311, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 311, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 311, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 311, 1); /*direct input signal access*/
	
	/* Input Port 312 */
    ssSetInputPortMatrixDimensions(S,  312, 10, 14);
    ssSetInputPortDataType(S, 312, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 312, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 312, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 312, 1); /*direct input signal access*/
	
	/* Input Port 313 */
    ssSetInputPortMatrixDimensions(S,  313, 10, 14);
    ssSetInputPortDataType(S, 313, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 313, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 313, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 313, 1); /*direct input signal access*/
	
	/* Input Port 314 */
    ssSetInputPortMatrixDimensions(S,  314, 10, 14);
    ssSetInputPortDataType(S, 314, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 314, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 314, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 314, 1); /*direct input signal access*/
	
	/* Input Port 315 */
    ssSetInputPortMatrixDimensions(S,  315, 10, 14);
    ssSetInputPortDataType(S, 315, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 315, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 315, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 315, 1); /*direct input signal access*/
	
	/* Input Port 316 */
    ssSetInputPortMatrixDimensions(S,  316, 10, 14);
    ssSetInputPortDataType(S, 316, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 316, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 316, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 316, 1); /*direct input signal access*/
	
	/* Input Port 317 */
    ssSetInputPortMatrixDimensions(S,  317, 10, 14);
    ssSetInputPortDataType(S, 317, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 317, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 317, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 317, 1); /*direct input signal access*/
	
	/* Input Port 318 */
    ssSetInputPortMatrixDimensions(S,  318, 10, 14);
    ssSetInputPortDataType(S, 318, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 318, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 318, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 318, 1); /*direct input signal access*/
	
	/* Input Port 319 */
    ssSetInputPortMatrixDimensions(S,  319, 10, 14);
    ssSetInputPortDataType(S, 319, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 319, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 319, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 319, 1); /*direct input signal access*/
	
	/* Input Port 320 */
    ssSetInputPortMatrixDimensions(S,  320, 10, 14);
    ssSetInputPortDataType(S, 320, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 320, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 320, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 320, 1); /*direct input signal access*/
	
	/* Input Port 321 */
    ssSetInputPortMatrixDimensions(S,  321, 10, 14);
    ssSetInputPortDataType(S, 321, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 321, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 321, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 321, 1); /*direct input signal access*/
	
	/* Input Port 322 */
    ssSetInputPortMatrixDimensions(S,  322, 10, 14);
    ssSetInputPortDataType(S, 322, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 322, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 322, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 322, 1); /*direct input signal access*/
	
	/* Input Port 323 */
    ssSetInputPortMatrixDimensions(S,  323, 2, 14);
    ssSetInputPortDataType(S, 323, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 323, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 323, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 323, 1); /*direct input signal access*/
	
	/* Input Port 324 */
    ssSetInputPortMatrixDimensions(S,  324, 2, 14);
    ssSetInputPortDataType(S, 324, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 324, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 324, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 324, 1); /*direct input signal access*/
	
	/* Input Port 325 */
    ssSetInputPortMatrixDimensions(S,  325, 2, 14);
    ssSetInputPortDataType(S, 325, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 325, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 325, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 325, 1); /*direct input signal access*/
	
	/* Input Port 326 */
    ssSetInputPortMatrixDimensions(S,  326, 2, 14);
    ssSetInputPortDataType(S, 326, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 326, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 326, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 326, 1); /*direct input signal access*/
	
	/* Input Port 327 */
    ssSetInputPortMatrixDimensions(S,  327, 2, 14);
    ssSetInputPortDataType(S, 327, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 327, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 327, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 327, 1); /*direct input signal access*/
	
	/* Input Port 328 */
    ssSetInputPortMatrixDimensions(S,  328, 2, 14);
    ssSetInputPortDataType(S, 328, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 328, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 328, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 328, 1); /*direct input signal access*/
	
	/* Input Port 329 */
    ssSetInputPortMatrixDimensions(S,  329, 2, 14);
    ssSetInputPortDataType(S, 329, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 329, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 329, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 329, 1); /*direct input signal access*/
	
	/* Input Port 330 */
    ssSetInputPortMatrixDimensions(S,  330, 2, 14);
    ssSetInputPortDataType(S, 330, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 330, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 330, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 330, 1); /*direct input signal access*/
	
	/* Input Port 331 */
    ssSetInputPortMatrixDimensions(S,  331, 2, 14);
    ssSetInputPortDataType(S, 331, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 331, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 331, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 331, 1); /*direct input signal access*/
	
	/* Input Port 332 */
    ssSetInputPortMatrixDimensions(S,  332, 2, 14);
    ssSetInputPortDataType(S, 332, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 332, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 332, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 332, 1); /*direct input signal access*/
	
	/* Input Port 333 */
    ssSetInputPortMatrixDimensions(S,  333, 2, 14);
    ssSetInputPortDataType(S, 333, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 333, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 333, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 333, 1); /*direct input signal access*/
	
	/* Input Port 334 */
    ssSetInputPortMatrixDimensions(S,  334, 2, 14);
    ssSetInputPortDataType(S, 334, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 334, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 334, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 334, 1); /*direct input signal access*/
	
	/* Input Port 335 */
    ssSetInputPortMatrixDimensions(S,  335, 2, 14);
    ssSetInputPortDataType(S, 335, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 335, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 335, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 335, 1); /*direct input signal access*/
	
	/* Input Port 336 */
    ssSetInputPortMatrixDimensions(S,  336, 2, 14);
    ssSetInputPortDataType(S, 336, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 336, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 336, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 336, 1); /*direct input signal access*/
	
	/* Input Port 337 */
    ssSetInputPortMatrixDimensions(S,  337, 2, 14);
    ssSetInputPortDataType(S, 337, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 337, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 337, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 337, 1); /*direct input signal access*/
	
	/* Input Port 338 */
    ssSetInputPortMatrixDimensions(S,  338, 2, 14);
    ssSetInputPortDataType(S, 338, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 338, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 338, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 338, 1); /*direct input signal access*/
	
	/* Input Port 339 */
    ssSetInputPortMatrixDimensions(S,  339, 2, 14);
    ssSetInputPortDataType(S, 339, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 339, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 339, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 339, 1); /*direct input signal access*/
	
	/* Input Port 340 */
    ssSetInputPortMatrixDimensions(S,  340, 2, 14);
    ssSetInputPortDataType(S, 340, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 340, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 340, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 340, 1); /*direct input signal access*/
	
	/* Input Port 341 */
    ssSetInputPortMatrixDimensions(S,  341, 2, 14);
    ssSetInputPortDataType(S, 341, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 341, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 341, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 341, 1); /*direct input signal access*/
	
	/* Input Port 342 */
    ssSetInputPortMatrixDimensions(S,  342, 2, 14);
    ssSetInputPortDataType(S, 342, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 342, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 342, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 342, 1); /*direct input signal access*/
	
	/* Input Port 343 */
    ssSetInputPortMatrixDimensions(S,  343, 2, 14);
    ssSetInputPortDataType(S, 343, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 343, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 343, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 343, 1); /*direct input signal access*/
	
	/* Input Port 344 */
    ssSetInputPortMatrixDimensions(S,  344, 2, 14);
    ssSetInputPortDataType(S, 344, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 344, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 344, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 344, 1); /*direct input signal access*/
	
	/* Input Port 345 */
    ssSetInputPortMatrixDimensions(S,  345, 2, 14);
    ssSetInputPortDataType(S, 345, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 345, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 345, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 345, 1); /*direct input signal access*/
	
	/* Input Port 346 */
    ssSetInputPortMatrixDimensions(S,  346, 2, 14);
    ssSetInputPortDataType(S, 346, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 346, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 346, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 346, 1); /*direct input signal access*/
	
	/* Input Port 347 */
    ssSetInputPortMatrixDimensions(S,  347, 2, 14);
    ssSetInputPortDataType(S, 347, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 347, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 347, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 347, 1); /*direct input signal access*/
	
	/* Input Port 348 */
    ssSetInputPortMatrixDimensions(S,  348, 2, 14);
    ssSetInputPortDataType(S, 348, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 348, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 348, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 348, 1); /*direct input signal access*/
	
	/* Input Port 349 */
    ssSetInputPortMatrixDimensions(S,  349, 2, 14);
    ssSetInputPortDataType(S, 349, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 349, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 349, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 349, 1); /*direct input signal access*/
	
	/* Input Port 350 */
    ssSetInputPortMatrixDimensions(S,  350, 2, 14);
    ssSetInputPortDataType(S, 350, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 350, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 350, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 350, 1); /*direct input signal access*/
	
	/* Input Port 351 */
    ssSetInputPortMatrixDimensions(S,  351, 2, 14);
    ssSetInputPortDataType(S, 351, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 351, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 351, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 351, 1); /*direct input signal access*/
	
	/* Input Port 352 */
    ssSetInputPortMatrixDimensions(S,  352, 2, 14);
    ssSetInputPortDataType(S, 352, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 352, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 352, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 352, 1); /*direct input signal access*/
	
	/* Input Port 353 */
    ssSetInputPortMatrixDimensions(S,  353, 2, 14);
    ssSetInputPortDataType(S, 353, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 353, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 353, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 353, 1); /*direct input signal access*/
	
	/* Input Port 354 */
    ssSetInputPortMatrixDimensions(S,  354, 2, 14);
    ssSetInputPortDataType(S, 354, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 354, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 354, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 354, 1); /*direct input signal access*/
	
	/* Input Port 355 */
    ssSetInputPortMatrixDimensions(S,  355, 2, 14);
    ssSetInputPortDataType(S, 355, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 355, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 355, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 355, 1); /*direct input signal access*/
	
	/* Input Port 356 */
    ssSetInputPortMatrixDimensions(S,  356, 2, 14);
    ssSetInputPortDataType(S, 356, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 356, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 356, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 356, 1); /*direct input signal access*/
	
	/* Input Port 357 */
    ssSetInputPortMatrixDimensions(S,  357, 2, 14);
    ssSetInputPortDataType(S, 357, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 357, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 357, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 357, 1); /*direct input signal access*/
	
	/* Input Port 358 */
    ssSetInputPortMatrixDimensions(S,  358, 2, 14);
    ssSetInputPortDataType(S, 358, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 358, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 358, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 358, 1); /*direct input signal access*/
	
	/* Input Port 359 */
    ssSetInputPortMatrixDimensions(S,  359, 2, 14);
    ssSetInputPortDataType(S, 359, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 359, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 359, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 359, 1); /*direct input signal access*/
	
	/* Input Port 360 */
    ssSetInputPortMatrixDimensions(S,  360, 2, 14);
    ssSetInputPortDataType(S, 360, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 360, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 360, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 360, 1); /*direct input signal access*/
	
	/* Input Port 361 */
    ssSetInputPortMatrixDimensions(S,  361, 2, 14);
    ssSetInputPortDataType(S, 361, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 361, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 361, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 361, 1); /*direct input signal access*/
	
	/* Input Port 362 */
    ssSetInputPortMatrixDimensions(S,  362, 2, 14);
    ssSetInputPortDataType(S, 362, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 362, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 362, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 362, 1); /*direct input signal access*/
	
	/* Input Port 363 */
    ssSetInputPortMatrixDimensions(S,  363, 2, 14);
    ssSetInputPortDataType(S, 363, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 363, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 363, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 363, 1); /*direct input signal access*/
	
	/* Input Port 364 */
    ssSetInputPortMatrixDimensions(S,  364, 2, 14);
    ssSetInputPortDataType(S, 364, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 364, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 364, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 364, 1); /*direct input signal access*/
	
	/* Input Port 365 */
    ssSetInputPortMatrixDimensions(S,  365, 2, 14);
    ssSetInputPortDataType(S, 365, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 365, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 365, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 365, 1); /*direct input signal access*/
	
	/* Input Port 366 */
    ssSetInputPortMatrixDimensions(S,  366, 2, 14);
    ssSetInputPortDataType(S, 366, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 366, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 366, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 366, 1); /*direct input signal access*/
	
	/* Input Port 367 */
    ssSetInputPortMatrixDimensions(S,  367, 2, 14);
    ssSetInputPortDataType(S, 367, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 367, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 367, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 367, 1); /*direct input signal access*/
	
	/* Input Port 368 */
    ssSetInputPortMatrixDimensions(S,  368, 2, 14);
    ssSetInputPortDataType(S, 368, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 368, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 368, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 368, 1); /*direct input signal access*/
	
	/* Input Port 369 */
    ssSetInputPortMatrixDimensions(S,  369, 2, 14);
    ssSetInputPortDataType(S, 369, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 369, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 369, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 369, 1); /*direct input signal access*/
	
	/* Input Port 370 */
    ssSetInputPortMatrixDimensions(S,  370, 2, 14);
    ssSetInputPortDataType(S, 370, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 370, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 370, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 370, 1); /*direct input signal access*/
	
	/* Input Port 371 */
    ssSetInputPortMatrixDimensions(S,  371, 2, 14);
    ssSetInputPortDataType(S, 371, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 371, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 371, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 371, 1); /*direct input signal access*/
	
	/* Input Port 372 */
    ssSetInputPortMatrixDimensions(S,  372, 2, 14);
    ssSetInputPortDataType(S, 372, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 372, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 372, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 372, 1); /*direct input signal access*/
	
	/* Input Port 373 */
    ssSetInputPortMatrixDimensions(S,  373, 2, 14);
    ssSetInputPortDataType(S, 373, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 373, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 373, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 373, 1); /*direct input signal access*/
	
	/* Input Port 374 */
    ssSetInputPortMatrixDimensions(S,  374, 2, 14);
    ssSetInputPortDataType(S, 374, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 374, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 374, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 374, 1); /*direct input signal access*/
	
	/* Input Port 375 */
    ssSetInputPortMatrixDimensions(S,  375, 2, 14);
    ssSetInputPortDataType(S, 375, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 375, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 375, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 375, 1); /*direct input signal access*/
	
	/* Input Port 376 */
    ssSetInputPortMatrixDimensions(S,  376, 2, 14);
    ssSetInputPortDataType(S, 376, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 376, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 376, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 376, 1); /*direct input signal access*/
	
	/* Input Port 377 */
    ssSetInputPortMatrixDimensions(S,  377, 2, 14);
    ssSetInputPortDataType(S, 377, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 377, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 377, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 377, 1); /*direct input signal access*/
	
	/* Input Port 378 */
    ssSetInputPortMatrixDimensions(S,  378, 2, 14);
    ssSetInputPortDataType(S, 378, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 378, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 378, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 378, 1); /*direct input signal access*/
	
	/* Input Port 379 */
    ssSetInputPortMatrixDimensions(S,  379, 2, 14);
    ssSetInputPortDataType(S, 379, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 379, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 379, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 379, 1); /*direct input signal access*/
	
	/* Input Port 380 */
    ssSetInputPortMatrixDimensions(S,  380, 2, 14);
    ssSetInputPortDataType(S, 380, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 380, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 380, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 380, 1); /*direct input signal access*/
	
	/* Input Port 381 */
    ssSetInputPortMatrixDimensions(S,  381, 2, 14);
    ssSetInputPortDataType(S, 381, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 381, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 381, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 381, 1); /*direct input signal access*/
	
	/* Input Port 382 */
    ssSetInputPortMatrixDimensions(S,  382, 2, 14);
    ssSetInputPortDataType(S, 382, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 382, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 382, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 382, 1); /*direct input signal access*/
	
	/* Input Port 383 */
    ssSetInputPortMatrixDimensions(S,  383, 2, 14);
    ssSetInputPortDataType(S, 383, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 383, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 383, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 383, 1); /*direct input signal access*/
	
	/* Input Port 384 */
    ssSetInputPortMatrixDimensions(S,  384, 2, 14);
    ssSetInputPortDataType(S, 384, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 384, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 384, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 384, 1); /*direct input signal access*/
	
	/* Input Port 385 */
    ssSetInputPortMatrixDimensions(S,  385, 2, 14);
    ssSetInputPortDataType(S, 385, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 385, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 385, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 385, 1); /*direct input signal access*/
	
	/* Input Port 386 */
    ssSetInputPortMatrixDimensions(S,  386, 2, 14);
    ssSetInputPortDataType(S, 386, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 386, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 386, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 386, 1); /*direct input signal access*/
	
	/* Input Port 387 */
    ssSetInputPortMatrixDimensions(S,  387, 2, 14);
    ssSetInputPortDataType(S, 387, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 387, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 387, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 387, 1); /*direct input signal access*/
	
	/* Input Port 388 */
    ssSetInputPortMatrixDimensions(S,  388, 2, 14);
    ssSetInputPortDataType(S, 388, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 388, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 388, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 388, 1); /*direct input signal access*/
	
	/* Input Port 389 */
    ssSetInputPortMatrixDimensions(S,  389, 2, 14);
    ssSetInputPortDataType(S, 389, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 389, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 389, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 389, 1); /*direct input signal access*/
	
	/* Input Port 390 */
    ssSetInputPortMatrixDimensions(S,  390, 2, 14);
    ssSetInputPortDataType(S, 390, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 390, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 390, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 390, 1); /*direct input signal access*/
	
	/* Input Port 391 */
    ssSetInputPortMatrixDimensions(S,  391, 2, 14);
    ssSetInputPortDataType(S, 391, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 391, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 391, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 391, 1); /*direct input signal access*/
	
	/* Input Port 392 */
    ssSetInputPortMatrixDimensions(S,  392, 2, 14);
    ssSetInputPortDataType(S, 392, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 392, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 392, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 392, 1); /*direct input signal access*/
	
	/* Input Port 393 */
    ssSetInputPortMatrixDimensions(S,  393, 2, 14);
    ssSetInputPortDataType(S, 393, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 393, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 393, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 393, 1); /*direct input signal access*/
	
	/* Input Port 394 */
    ssSetInputPortMatrixDimensions(S,  394, 2, 14);
    ssSetInputPortDataType(S, 394, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 394, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 394, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 394, 1); /*direct input signal access*/
	
	/* Input Port 395 */
    ssSetInputPortMatrixDimensions(S,  395, 2, 14);
    ssSetInputPortDataType(S, 395, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 395, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 395, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 395, 1); /*direct input signal access*/
	
	/* Input Port 396 */
    ssSetInputPortMatrixDimensions(S,  396, 2, 14);
    ssSetInputPortDataType(S, 396, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 396, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 396, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 396, 1); /*direct input signal access*/
	
	/* Input Port 397 */
    ssSetInputPortMatrixDimensions(S,  397, 2, 14);
    ssSetInputPortDataType(S, 397, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 397, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 397, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 397, 1); /*direct input signal access*/
	
	/* Input Port 398 */
    ssSetInputPortMatrixDimensions(S,  398, 2, 14);
    ssSetInputPortDataType(S, 398, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 398, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 398, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 398, 1); /*direct input signal access*/
	
	/* Input Port 399 */
    ssSetInputPortMatrixDimensions(S,  399, 2, 14);
    ssSetInputPortDataType(S, 399, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 399, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 399, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 399, 1); /*direct input signal access*/
	
	/* Input Port 400 */
    ssSetInputPortMatrixDimensions(S,  400, 2, 14);
    ssSetInputPortDataType(S, 400, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 400, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 400, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 400, 1); /*direct input signal access*/
	
	/* Input Port 401 */
    ssSetInputPortMatrixDimensions(S,  401, 2, 14);
    ssSetInputPortDataType(S, 401, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 401, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 401, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 401, 1); /*direct input signal access*/
	
	/* Input Port 402 */
    ssSetInputPortMatrixDimensions(S,  402, 2, 14);
    ssSetInputPortDataType(S, 402, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 402, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 402, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 402, 1); /*direct input signal access*/
	
	/* Input Port 403 */
    ssSetInputPortMatrixDimensions(S,  403, 2, 1);
    ssSetInputPortDataType(S, 403, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 403, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 403, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 403, 1); /*direct input signal access*/
	
	/* Input Port 404 */
    ssSetInputPortMatrixDimensions(S,  404, 2, 1);
    ssSetInputPortDataType(S, 404, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 404, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 404, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 404, 1); /*direct input signal access*/
	
	/* Input Port 405 */
    ssSetInputPortMatrixDimensions(S,  405, 2, 1);
    ssSetInputPortDataType(S, 405, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 405, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 405, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 405, 1); /*direct input signal access*/
	
	/* Input Port 406 */
    ssSetInputPortMatrixDimensions(S,  406, 2, 1);
    ssSetInputPortDataType(S, 406, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 406, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 406, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 406, 1); /*direct input signal access*/
	
	/* Input Port 407 */
    ssSetInputPortMatrixDimensions(S,  407, 2, 1);
    ssSetInputPortDataType(S, 407, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 407, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 407, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 407, 1); /*direct input signal access*/
	
	/* Input Port 408 */
    ssSetInputPortMatrixDimensions(S,  408, 2, 1);
    ssSetInputPortDataType(S, 408, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 408, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 408, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 408, 1); /*direct input signal access*/
	
	/* Input Port 409 */
    ssSetInputPortMatrixDimensions(S,  409, 2, 1);
    ssSetInputPortDataType(S, 409, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 409, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 409, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 409, 1); /*direct input signal access*/
	
	/* Input Port 410 */
    ssSetInputPortMatrixDimensions(S,  410, 2, 1);
    ssSetInputPortDataType(S, 410, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 410, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 410, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 410, 1); /*direct input signal access*/
	
	/* Input Port 411 */
    ssSetInputPortMatrixDimensions(S,  411, 2, 1);
    ssSetInputPortDataType(S, 411, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 411, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 411, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 411, 1); /*direct input signal access*/
	
	/* Input Port 412 */
    ssSetInputPortMatrixDimensions(S,  412, 2, 1);
    ssSetInputPortDataType(S, 412, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 412, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 412, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 412, 1); /*direct input signal access*/
	
	/* Input Port 413 */
    ssSetInputPortMatrixDimensions(S,  413, 2, 1);
    ssSetInputPortDataType(S, 413, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 413, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 413, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 413, 1); /*direct input signal access*/
	
	/* Input Port 414 */
    ssSetInputPortMatrixDimensions(S,  414, 2, 1);
    ssSetInputPortDataType(S, 414, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 414, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 414, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 414, 1); /*direct input signal access*/
	
	/* Input Port 415 */
    ssSetInputPortMatrixDimensions(S,  415, 2, 1);
    ssSetInputPortDataType(S, 415, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 415, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 415, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 415, 1); /*direct input signal access*/
	
	/* Input Port 416 */
    ssSetInputPortMatrixDimensions(S,  416, 2, 1);
    ssSetInputPortDataType(S, 416, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 416, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 416, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 416, 1); /*direct input signal access*/
	
	/* Input Port 417 */
    ssSetInputPortMatrixDimensions(S,  417, 2, 1);
    ssSetInputPortDataType(S, 417, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 417, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 417, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 417, 1); /*direct input signal access*/
	
	/* Input Port 418 */
    ssSetInputPortMatrixDimensions(S,  418, 2, 1);
    ssSetInputPortDataType(S, 418, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 418, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 418, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 418, 1); /*direct input signal access*/
	
	/* Input Port 419 */
    ssSetInputPortMatrixDimensions(S,  419, 2, 1);
    ssSetInputPortDataType(S, 419, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 419, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 419, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 419, 1); /*direct input signal access*/
	
	/* Input Port 420 */
    ssSetInputPortMatrixDimensions(S,  420, 2, 1);
    ssSetInputPortDataType(S, 420, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 420, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 420, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 420, 1); /*direct input signal access*/
	
	/* Input Port 421 */
    ssSetInputPortMatrixDimensions(S,  421, 2, 1);
    ssSetInputPortDataType(S, 421, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 421, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 421, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 421, 1); /*direct input signal access*/
	
	/* Input Port 422 */
    ssSetInputPortMatrixDimensions(S,  422, 2, 1);
    ssSetInputPortDataType(S, 422, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 422, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 422, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 422, 1); /*direct input signal access*/
	
	/* Input Port 423 */
    ssSetInputPortMatrixDimensions(S,  423, 2, 1);
    ssSetInputPortDataType(S, 423, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 423, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 423, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 423, 1); /*direct input signal access*/
	
	/* Input Port 424 */
    ssSetInputPortMatrixDimensions(S,  424, 2, 1);
    ssSetInputPortDataType(S, 424, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 424, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 424, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 424, 1); /*direct input signal access*/
	
	/* Input Port 425 */
    ssSetInputPortMatrixDimensions(S,  425, 2, 1);
    ssSetInputPortDataType(S, 425, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 425, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 425, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 425, 1); /*direct input signal access*/
	
	/* Input Port 426 */
    ssSetInputPortMatrixDimensions(S,  426, 2, 1);
    ssSetInputPortDataType(S, 426, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 426, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 426, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 426, 1); /*direct input signal access*/
	
	/* Input Port 427 */
    ssSetInputPortMatrixDimensions(S,  427, 2, 1);
    ssSetInputPortDataType(S, 427, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 427, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 427, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 427, 1); /*direct input signal access*/
	
	/* Input Port 428 */
    ssSetInputPortMatrixDimensions(S,  428, 2, 1);
    ssSetInputPortDataType(S, 428, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 428, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 428, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 428, 1); /*direct input signal access*/
	
	/* Input Port 429 */
    ssSetInputPortMatrixDimensions(S,  429, 2, 1);
    ssSetInputPortDataType(S, 429, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 429, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 429, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 429, 1); /*direct input signal access*/
	
	/* Input Port 430 */
    ssSetInputPortMatrixDimensions(S,  430, 2, 1);
    ssSetInputPortDataType(S, 430, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 430, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 430, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 430, 1); /*direct input signal access*/
	
	/* Input Port 431 */
    ssSetInputPortMatrixDimensions(S,  431, 2, 1);
    ssSetInputPortDataType(S, 431, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 431, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 431, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 431, 1); /*direct input signal access*/
	
	/* Input Port 432 */
    ssSetInputPortMatrixDimensions(S,  432, 2, 1);
    ssSetInputPortDataType(S, 432, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 432, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 432, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 432, 1); /*direct input signal access*/
	
	/* Input Port 433 */
    ssSetInputPortMatrixDimensions(S,  433, 2, 1);
    ssSetInputPortDataType(S, 433, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 433, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 433, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 433, 1); /*direct input signal access*/
	
	/* Input Port 434 */
    ssSetInputPortMatrixDimensions(S,  434, 2, 1);
    ssSetInputPortDataType(S, 434, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 434, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 434, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 434, 1); /*direct input signal access*/
	
	/* Input Port 435 */
    ssSetInputPortMatrixDimensions(S,  435, 2, 1);
    ssSetInputPortDataType(S, 435, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 435, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 435, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 435, 1); /*direct input signal access*/
	
	/* Input Port 436 */
    ssSetInputPortMatrixDimensions(S,  436, 2, 1);
    ssSetInputPortDataType(S, 436, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 436, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 436, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 436, 1); /*direct input signal access*/
	
	/* Input Port 437 */
    ssSetInputPortMatrixDimensions(S,  437, 2, 1);
    ssSetInputPortDataType(S, 437, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 437, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 437, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 437, 1); /*direct input signal access*/
	
	/* Input Port 438 */
    ssSetInputPortMatrixDimensions(S,  438, 2, 1);
    ssSetInputPortDataType(S, 438, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 438, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 438, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 438, 1); /*direct input signal access*/
	
	/* Input Port 439 */
    ssSetInputPortMatrixDimensions(S,  439, 2, 1);
    ssSetInputPortDataType(S, 439, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 439, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 439, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 439, 1); /*direct input signal access*/
	
	/* Input Port 440 */
    ssSetInputPortMatrixDimensions(S,  440, 2, 1);
    ssSetInputPortDataType(S, 440, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 440, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 440, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 440, 1); /*direct input signal access*/
	
	/* Input Port 441 */
    ssSetInputPortMatrixDimensions(S,  441, 2, 1);
    ssSetInputPortDataType(S, 441, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 441, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 441, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 441, 1); /*direct input signal access*/
	
	/* Input Port 442 */
    ssSetInputPortMatrixDimensions(S,  442, 2, 1);
    ssSetInputPortDataType(S, 442, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 442, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 442, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 442, 1); /*direct input signal access*/
	
	/* Input Port 443 */
    ssSetInputPortMatrixDimensions(S,  443, 2, 1);
    ssSetInputPortDataType(S, 443, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 443, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 443, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 443, 1); /*direct input signal access*/
	
	/* Input Port 444 */
    ssSetInputPortMatrixDimensions(S,  444, 2, 1);
    ssSetInputPortDataType(S, 444, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 444, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 444, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 444, 1); /*direct input signal access*/
	
	/* Input Port 445 */
    ssSetInputPortMatrixDimensions(S,  445, 2, 1);
    ssSetInputPortDataType(S, 445, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 445, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 445, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 445, 1); /*direct input signal access*/
	
	/* Input Port 446 */
    ssSetInputPortMatrixDimensions(S,  446, 2, 1);
    ssSetInputPortDataType(S, 446, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 446, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 446, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 446, 1); /*direct input signal access*/
	
	/* Input Port 447 */
    ssSetInputPortMatrixDimensions(S,  447, 2, 1);
    ssSetInputPortDataType(S, 447, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 447, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 447, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 447, 1); /*direct input signal access*/
	
	/* Input Port 448 */
    ssSetInputPortMatrixDimensions(S,  448, 2, 1);
    ssSetInputPortDataType(S, 448, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 448, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 448, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 448, 1); /*direct input signal access*/
	
	/* Input Port 449 */
    ssSetInputPortMatrixDimensions(S,  449, 2, 1);
    ssSetInputPortDataType(S, 449, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 449, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 449, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 449, 1); /*direct input signal access*/
	
	/* Input Port 450 */
    ssSetInputPortMatrixDimensions(S,  450, 2, 1);
    ssSetInputPortDataType(S, 450, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 450, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 450, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 450, 1); /*direct input signal access*/
	
	/* Input Port 451 */
    ssSetInputPortMatrixDimensions(S,  451, 2, 1);
    ssSetInputPortDataType(S, 451, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 451, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 451, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 451, 1); /*direct input signal access*/
	
	/* Input Port 452 */
    ssSetInputPortMatrixDimensions(S,  452, 2, 1);
    ssSetInputPortDataType(S, 452, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 452, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 452, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 452, 1); /*direct input signal access*/
	
	/* Input Port 453 */
    ssSetInputPortMatrixDimensions(S,  453, 2, 1);
    ssSetInputPortDataType(S, 453, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 453, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 453, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 453, 1); /*direct input signal access*/
	
	/* Input Port 454 */
    ssSetInputPortMatrixDimensions(S,  454, 2, 1);
    ssSetInputPortDataType(S, 454, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 454, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 454, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 454, 1); /*direct input signal access*/
	
	/* Input Port 455 */
    ssSetInputPortMatrixDimensions(S,  455, 2, 1);
    ssSetInputPortDataType(S, 455, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 455, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 455, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 455, 1); /*direct input signal access*/
	
	/* Input Port 456 */
    ssSetInputPortMatrixDimensions(S,  456, 2, 1);
    ssSetInputPortDataType(S, 456, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 456, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 456, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 456, 1); /*direct input signal access*/
	
	/* Input Port 457 */
    ssSetInputPortMatrixDimensions(S,  457, 2, 1);
    ssSetInputPortDataType(S, 457, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 457, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 457, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 457, 1); /*direct input signal access*/
	
	/* Input Port 458 */
    ssSetInputPortMatrixDimensions(S,  458, 2, 1);
    ssSetInputPortDataType(S, 458, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 458, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 458, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 458, 1); /*direct input signal access*/
	
	/* Input Port 459 */
    ssSetInputPortMatrixDimensions(S,  459, 2, 1);
    ssSetInputPortDataType(S, 459, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 459, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 459, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 459, 1); /*direct input signal access*/
	
	/* Input Port 460 */
    ssSetInputPortMatrixDimensions(S,  460, 2, 1);
    ssSetInputPortDataType(S, 460, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 460, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 460, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 460, 1); /*direct input signal access*/
	
	/* Input Port 461 */
    ssSetInputPortMatrixDimensions(S,  461, 2, 1);
    ssSetInputPortDataType(S, 461, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 461, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 461, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 461, 1); /*direct input signal access*/
	
	/* Input Port 462 */
    ssSetInputPortMatrixDimensions(S,  462, 2, 1);
    ssSetInputPortDataType(S, 462, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 462, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 462, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 462, 1); /*direct input signal access*/
	
	/* Input Port 463 */
    ssSetInputPortMatrixDimensions(S,  463, 2, 1);
    ssSetInputPortDataType(S, 463, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 463, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 463, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 463, 1); /*direct input signal access*/
	
	/* Input Port 464 */
    ssSetInputPortMatrixDimensions(S,  464, 2, 1);
    ssSetInputPortDataType(S, 464, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 464, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 464, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 464, 1); /*direct input signal access*/
	
	/* Input Port 465 */
    ssSetInputPortMatrixDimensions(S,  465, 2, 1);
    ssSetInputPortDataType(S, 465, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 465, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 465, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 465, 1); /*direct input signal access*/
	
	/* Input Port 466 */
    ssSetInputPortMatrixDimensions(S,  466, 2, 1);
    ssSetInputPortDataType(S, 466, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 466, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 466, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 466, 1); /*direct input signal access*/
	
	/* Input Port 467 */
    ssSetInputPortMatrixDimensions(S,  467, 2, 1);
    ssSetInputPortDataType(S, 467, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 467, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 467, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 467, 1); /*direct input signal access*/
	
	/* Input Port 468 */
    ssSetInputPortMatrixDimensions(S,  468, 2, 1);
    ssSetInputPortDataType(S, 468, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 468, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 468, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 468, 1); /*direct input signal access*/
	
	/* Input Port 469 */
    ssSetInputPortMatrixDimensions(S,  469, 2, 1);
    ssSetInputPortDataType(S, 469, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 469, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 469, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 469, 1); /*direct input signal access*/
	
	/* Input Port 470 */
    ssSetInputPortMatrixDimensions(S,  470, 2, 1);
    ssSetInputPortDataType(S, 470, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 470, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 470, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 470, 1); /*direct input signal access*/
	
	/* Input Port 471 */
    ssSetInputPortMatrixDimensions(S,  471, 2, 1);
    ssSetInputPortDataType(S, 471, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 471, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 471, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 471, 1); /*direct input signal access*/
	
	/* Input Port 472 */
    ssSetInputPortMatrixDimensions(S,  472, 2, 1);
    ssSetInputPortDataType(S, 472, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 472, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 472, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 472, 1); /*direct input signal access*/
	
	/* Input Port 473 */
    ssSetInputPortMatrixDimensions(S,  473, 2, 1);
    ssSetInputPortDataType(S, 473, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 473, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 473, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 473, 1); /*direct input signal access*/
	
	/* Input Port 474 */
    ssSetInputPortMatrixDimensions(S,  474, 2, 1);
    ssSetInputPortDataType(S, 474, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 474, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 474, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 474, 1); /*direct input signal access*/
	
	/* Input Port 475 */
    ssSetInputPortMatrixDimensions(S,  475, 2, 1);
    ssSetInputPortDataType(S, 475, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 475, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 475, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 475, 1); /*direct input signal access*/
	
	/* Input Port 476 */
    ssSetInputPortMatrixDimensions(S,  476, 2, 1);
    ssSetInputPortDataType(S, 476, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 476, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 476, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 476, 1); /*direct input signal access*/
	
	/* Input Port 477 */
    ssSetInputPortMatrixDimensions(S,  477, 2, 1);
    ssSetInputPortDataType(S, 477, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 477, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 477, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 477, 1); /*direct input signal access*/
	
	/* Input Port 478 */
    ssSetInputPortMatrixDimensions(S,  478, 2, 1);
    ssSetInputPortDataType(S, 478, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 478, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 478, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 478, 1); /*direct input signal access*/
	
	/* Input Port 479 */
    ssSetInputPortMatrixDimensions(S,  479, 2, 1);
    ssSetInputPortDataType(S, 479, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 479, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 479, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 479, 1); /*direct input signal access*/
	
	/* Input Port 480 */
    ssSetInputPortMatrixDimensions(S,  480, 2, 1);
    ssSetInputPortDataType(S, 480, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 480, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 480, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 480, 1); /*direct input signal access*/
	
	/* Input Port 481 */
    ssSetInputPortMatrixDimensions(S,  481, 2, 1);
    ssSetInputPortDataType(S, 481, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 481, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 481, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 481, 1); /*direct input signal access*/
	
	/* Input Port 482 */
    ssSetInputPortMatrixDimensions(S,  482, 2, 1);
    ssSetInputPortDataType(S, 482, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 482, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 482, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 482, 1); /*direct input signal access*/
 


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
	const real_T *c_42 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *c_43 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *c_44 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *c_45 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *c_46 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *c_47 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *c_48 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *c_49 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *c_50 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *c_51 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *c_52 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *c_53 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *c_54 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *c_55 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *c_56 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *c_57 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *c_58 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *c_59 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *c_60 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *c_61 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *c_62 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *c_63 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *c_64 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *c_65 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *c_66 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *c_67 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *c_68 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *c_69 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *c_70 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *c_71 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *c_72 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *c_73 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *c_74 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *c_75 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *c_76 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *c_77 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *c_78 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *c_79 = (const real_T*) ssGetInputPortSignal(S,78);
	const real_T *c_80 = (const real_T*) ssGetInputPortSignal(S,79);
	const real_T *c_81 = (const real_T*) ssGetInputPortSignal(S,80);
	const real_T *H_1 = (const real_T*) ssGetInputPortSignal(S,81);
	const real_T *H_2 = (const real_T*) ssGetInputPortSignal(S,82);
	const real_T *H_3 = (const real_T*) ssGetInputPortSignal(S,83);
	const real_T *H_4 = (const real_T*) ssGetInputPortSignal(S,84);
	const real_T *H_5 = (const real_T*) ssGetInputPortSignal(S,85);
	const real_T *H_6 = (const real_T*) ssGetInputPortSignal(S,86);
	const real_T *H_7 = (const real_T*) ssGetInputPortSignal(S,87);
	const real_T *H_8 = (const real_T*) ssGetInputPortSignal(S,88);
	const real_T *H_9 = (const real_T*) ssGetInputPortSignal(S,89);
	const real_T *H_10 = (const real_T*) ssGetInputPortSignal(S,90);
	const real_T *H_11 = (const real_T*) ssGetInputPortSignal(S,91);
	const real_T *H_12 = (const real_T*) ssGetInputPortSignal(S,92);
	const real_T *H_13 = (const real_T*) ssGetInputPortSignal(S,93);
	const real_T *H_14 = (const real_T*) ssGetInputPortSignal(S,94);
	const real_T *H_15 = (const real_T*) ssGetInputPortSignal(S,95);
	const real_T *H_16 = (const real_T*) ssGetInputPortSignal(S,96);
	const real_T *H_17 = (const real_T*) ssGetInputPortSignal(S,97);
	const real_T *H_18 = (const real_T*) ssGetInputPortSignal(S,98);
	const real_T *H_19 = (const real_T*) ssGetInputPortSignal(S,99);
	const real_T *H_20 = (const real_T*) ssGetInputPortSignal(S,100);
	const real_T *H_21 = (const real_T*) ssGetInputPortSignal(S,101);
	const real_T *H_22 = (const real_T*) ssGetInputPortSignal(S,102);
	const real_T *H_23 = (const real_T*) ssGetInputPortSignal(S,103);
	const real_T *H_24 = (const real_T*) ssGetInputPortSignal(S,104);
	const real_T *H_25 = (const real_T*) ssGetInputPortSignal(S,105);
	const real_T *H_26 = (const real_T*) ssGetInputPortSignal(S,106);
	const real_T *H_27 = (const real_T*) ssGetInputPortSignal(S,107);
	const real_T *H_28 = (const real_T*) ssGetInputPortSignal(S,108);
	const real_T *H_29 = (const real_T*) ssGetInputPortSignal(S,109);
	const real_T *H_30 = (const real_T*) ssGetInputPortSignal(S,110);
	const real_T *H_31 = (const real_T*) ssGetInputPortSignal(S,111);
	const real_T *H_32 = (const real_T*) ssGetInputPortSignal(S,112);
	const real_T *H_33 = (const real_T*) ssGetInputPortSignal(S,113);
	const real_T *H_34 = (const real_T*) ssGetInputPortSignal(S,114);
	const real_T *H_35 = (const real_T*) ssGetInputPortSignal(S,115);
	const real_T *H_36 = (const real_T*) ssGetInputPortSignal(S,116);
	const real_T *H_37 = (const real_T*) ssGetInputPortSignal(S,117);
	const real_T *H_38 = (const real_T*) ssGetInputPortSignal(S,118);
	const real_T *H_39 = (const real_T*) ssGetInputPortSignal(S,119);
	const real_T *H_40 = (const real_T*) ssGetInputPortSignal(S,120);
	const real_T *H_41 = (const real_T*) ssGetInputPortSignal(S,121);
	const real_T *H_42 = (const real_T*) ssGetInputPortSignal(S,122);
	const real_T *H_43 = (const real_T*) ssGetInputPortSignal(S,123);
	const real_T *H_44 = (const real_T*) ssGetInputPortSignal(S,124);
	const real_T *H_45 = (const real_T*) ssGetInputPortSignal(S,125);
	const real_T *H_46 = (const real_T*) ssGetInputPortSignal(S,126);
	const real_T *H_47 = (const real_T*) ssGetInputPortSignal(S,127);
	const real_T *H_48 = (const real_T*) ssGetInputPortSignal(S,128);
	const real_T *H_49 = (const real_T*) ssGetInputPortSignal(S,129);
	const real_T *H_50 = (const real_T*) ssGetInputPortSignal(S,130);
	const real_T *H_51 = (const real_T*) ssGetInputPortSignal(S,131);
	const real_T *H_52 = (const real_T*) ssGetInputPortSignal(S,132);
	const real_T *H_53 = (const real_T*) ssGetInputPortSignal(S,133);
	const real_T *H_54 = (const real_T*) ssGetInputPortSignal(S,134);
	const real_T *H_55 = (const real_T*) ssGetInputPortSignal(S,135);
	const real_T *H_56 = (const real_T*) ssGetInputPortSignal(S,136);
	const real_T *H_57 = (const real_T*) ssGetInputPortSignal(S,137);
	const real_T *H_58 = (const real_T*) ssGetInputPortSignal(S,138);
	const real_T *H_59 = (const real_T*) ssGetInputPortSignal(S,139);
	const real_T *H_60 = (const real_T*) ssGetInputPortSignal(S,140);
	const real_T *H_61 = (const real_T*) ssGetInputPortSignal(S,141);
	const real_T *H_62 = (const real_T*) ssGetInputPortSignal(S,142);
	const real_T *H_63 = (const real_T*) ssGetInputPortSignal(S,143);
	const real_T *H_64 = (const real_T*) ssGetInputPortSignal(S,144);
	const real_T *H_65 = (const real_T*) ssGetInputPortSignal(S,145);
	const real_T *H_66 = (const real_T*) ssGetInputPortSignal(S,146);
	const real_T *H_67 = (const real_T*) ssGetInputPortSignal(S,147);
	const real_T *H_68 = (const real_T*) ssGetInputPortSignal(S,148);
	const real_T *H_69 = (const real_T*) ssGetInputPortSignal(S,149);
	const real_T *H_70 = (const real_T*) ssGetInputPortSignal(S,150);
	const real_T *H_71 = (const real_T*) ssGetInputPortSignal(S,151);
	const real_T *H_72 = (const real_T*) ssGetInputPortSignal(S,152);
	const real_T *H_73 = (const real_T*) ssGetInputPortSignal(S,153);
	const real_T *H_74 = (const real_T*) ssGetInputPortSignal(S,154);
	const real_T *H_75 = (const real_T*) ssGetInputPortSignal(S,155);
	const real_T *H_76 = (const real_T*) ssGetInputPortSignal(S,156);
	const real_T *H_77 = (const real_T*) ssGetInputPortSignal(S,157);
	const real_T *H_78 = (const real_T*) ssGetInputPortSignal(S,158);
	const real_T *H_79 = (const real_T*) ssGetInputPortSignal(S,159);
	const real_T *H_80 = (const real_T*) ssGetInputPortSignal(S,160);
	const real_T *H_81 = (const real_T*) ssGetInputPortSignal(S,161);
	const real_T *f_1 = (const real_T*) ssGetInputPortSignal(S,162);
	const real_T *f_2 = (const real_T*) ssGetInputPortSignal(S,163);
	const real_T *f_3 = (const real_T*) ssGetInputPortSignal(S,164);
	const real_T *f_4 = (const real_T*) ssGetInputPortSignal(S,165);
	const real_T *f_5 = (const real_T*) ssGetInputPortSignal(S,166);
	const real_T *f_6 = (const real_T*) ssGetInputPortSignal(S,167);
	const real_T *f_7 = (const real_T*) ssGetInputPortSignal(S,168);
	const real_T *f_8 = (const real_T*) ssGetInputPortSignal(S,169);
	const real_T *f_9 = (const real_T*) ssGetInputPortSignal(S,170);
	const real_T *f_10 = (const real_T*) ssGetInputPortSignal(S,171);
	const real_T *f_11 = (const real_T*) ssGetInputPortSignal(S,172);
	const real_T *f_12 = (const real_T*) ssGetInputPortSignal(S,173);
	const real_T *f_13 = (const real_T*) ssGetInputPortSignal(S,174);
	const real_T *f_14 = (const real_T*) ssGetInputPortSignal(S,175);
	const real_T *f_15 = (const real_T*) ssGetInputPortSignal(S,176);
	const real_T *f_16 = (const real_T*) ssGetInputPortSignal(S,177);
	const real_T *f_17 = (const real_T*) ssGetInputPortSignal(S,178);
	const real_T *f_18 = (const real_T*) ssGetInputPortSignal(S,179);
	const real_T *f_19 = (const real_T*) ssGetInputPortSignal(S,180);
	const real_T *f_20 = (const real_T*) ssGetInputPortSignal(S,181);
	const real_T *f_21 = (const real_T*) ssGetInputPortSignal(S,182);
	const real_T *f_22 = (const real_T*) ssGetInputPortSignal(S,183);
	const real_T *f_23 = (const real_T*) ssGetInputPortSignal(S,184);
	const real_T *f_24 = (const real_T*) ssGetInputPortSignal(S,185);
	const real_T *f_25 = (const real_T*) ssGetInputPortSignal(S,186);
	const real_T *f_26 = (const real_T*) ssGetInputPortSignal(S,187);
	const real_T *f_27 = (const real_T*) ssGetInputPortSignal(S,188);
	const real_T *f_28 = (const real_T*) ssGetInputPortSignal(S,189);
	const real_T *f_29 = (const real_T*) ssGetInputPortSignal(S,190);
	const real_T *f_30 = (const real_T*) ssGetInputPortSignal(S,191);
	const real_T *f_31 = (const real_T*) ssGetInputPortSignal(S,192);
	const real_T *f_32 = (const real_T*) ssGetInputPortSignal(S,193);
	const real_T *f_33 = (const real_T*) ssGetInputPortSignal(S,194);
	const real_T *f_34 = (const real_T*) ssGetInputPortSignal(S,195);
	const real_T *f_35 = (const real_T*) ssGetInputPortSignal(S,196);
	const real_T *f_36 = (const real_T*) ssGetInputPortSignal(S,197);
	const real_T *f_37 = (const real_T*) ssGetInputPortSignal(S,198);
	const real_T *f_38 = (const real_T*) ssGetInputPortSignal(S,199);
	const real_T *f_39 = (const real_T*) ssGetInputPortSignal(S,200);
	const real_T *f_40 = (const real_T*) ssGetInputPortSignal(S,201);
	const real_T *f_41 = (const real_T*) ssGetInputPortSignal(S,202);
	const real_T *f_42 = (const real_T*) ssGetInputPortSignal(S,203);
	const real_T *f_43 = (const real_T*) ssGetInputPortSignal(S,204);
	const real_T *f_44 = (const real_T*) ssGetInputPortSignal(S,205);
	const real_T *f_45 = (const real_T*) ssGetInputPortSignal(S,206);
	const real_T *f_46 = (const real_T*) ssGetInputPortSignal(S,207);
	const real_T *f_47 = (const real_T*) ssGetInputPortSignal(S,208);
	const real_T *f_48 = (const real_T*) ssGetInputPortSignal(S,209);
	const real_T *f_49 = (const real_T*) ssGetInputPortSignal(S,210);
	const real_T *f_50 = (const real_T*) ssGetInputPortSignal(S,211);
	const real_T *f_51 = (const real_T*) ssGetInputPortSignal(S,212);
	const real_T *f_52 = (const real_T*) ssGetInputPortSignal(S,213);
	const real_T *f_53 = (const real_T*) ssGetInputPortSignal(S,214);
	const real_T *f_54 = (const real_T*) ssGetInputPortSignal(S,215);
	const real_T *f_55 = (const real_T*) ssGetInputPortSignal(S,216);
	const real_T *f_56 = (const real_T*) ssGetInputPortSignal(S,217);
	const real_T *f_57 = (const real_T*) ssGetInputPortSignal(S,218);
	const real_T *f_58 = (const real_T*) ssGetInputPortSignal(S,219);
	const real_T *f_59 = (const real_T*) ssGetInputPortSignal(S,220);
	const real_T *f_60 = (const real_T*) ssGetInputPortSignal(S,221);
	const real_T *f_61 = (const real_T*) ssGetInputPortSignal(S,222);
	const real_T *f_62 = (const real_T*) ssGetInputPortSignal(S,223);
	const real_T *f_63 = (const real_T*) ssGetInputPortSignal(S,224);
	const real_T *f_64 = (const real_T*) ssGetInputPortSignal(S,225);
	const real_T *f_65 = (const real_T*) ssGetInputPortSignal(S,226);
	const real_T *f_66 = (const real_T*) ssGetInputPortSignal(S,227);
	const real_T *f_67 = (const real_T*) ssGetInputPortSignal(S,228);
	const real_T *f_68 = (const real_T*) ssGetInputPortSignal(S,229);
	const real_T *f_69 = (const real_T*) ssGetInputPortSignal(S,230);
	const real_T *f_70 = (const real_T*) ssGetInputPortSignal(S,231);
	const real_T *f_71 = (const real_T*) ssGetInputPortSignal(S,232);
	const real_T *f_72 = (const real_T*) ssGetInputPortSignal(S,233);
	const real_T *f_73 = (const real_T*) ssGetInputPortSignal(S,234);
	const real_T *f_74 = (const real_T*) ssGetInputPortSignal(S,235);
	const real_T *f_75 = (const real_T*) ssGetInputPortSignal(S,236);
	const real_T *f_76 = (const real_T*) ssGetInputPortSignal(S,237);
	const real_T *f_77 = (const real_T*) ssGetInputPortSignal(S,238);
	const real_T *f_78 = (const real_T*) ssGetInputPortSignal(S,239);
	const real_T *f_79 = (const real_T*) ssGetInputPortSignal(S,240);
	const real_T *f_80 = (const real_T*) ssGetInputPortSignal(S,241);
	const real_T *f_81 = (const real_T*) ssGetInputPortSignal(S,242);
	const real_T *C_1 = (const real_T*) ssGetInputPortSignal(S,243);
	const real_T *C_2 = (const real_T*) ssGetInputPortSignal(S,244);
	const real_T *C_3 = (const real_T*) ssGetInputPortSignal(S,245);
	const real_T *C_4 = (const real_T*) ssGetInputPortSignal(S,246);
	const real_T *C_5 = (const real_T*) ssGetInputPortSignal(S,247);
	const real_T *C_6 = (const real_T*) ssGetInputPortSignal(S,248);
	const real_T *C_7 = (const real_T*) ssGetInputPortSignal(S,249);
	const real_T *C_8 = (const real_T*) ssGetInputPortSignal(S,250);
	const real_T *C_9 = (const real_T*) ssGetInputPortSignal(S,251);
	const real_T *C_10 = (const real_T*) ssGetInputPortSignal(S,252);
	const real_T *C_11 = (const real_T*) ssGetInputPortSignal(S,253);
	const real_T *C_12 = (const real_T*) ssGetInputPortSignal(S,254);
	const real_T *C_13 = (const real_T*) ssGetInputPortSignal(S,255);
	const real_T *C_14 = (const real_T*) ssGetInputPortSignal(S,256);
	const real_T *C_15 = (const real_T*) ssGetInputPortSignal(S,257);
	const real_T *C_16 = (const real_T*) ssGetInputPortSignal(S,258);
	const real_T *C_17 = (const real_T*) ssGetInputPortSignal(S,259);
	const real_T *C_18 = (const real_T*) ssGetInputPortSignal(S,260);
	const real_T *C_19 = (const real_T*) ssGetInputPortSignal(S,261);
	const real_T *C_20 = (const real_T*) ssGetInputPortSignal(S,262);
	const real_T *C_21 = (const real_T*) ssGetInputPortSignal(S,263);
	const real_T *C_22 = (const real_T*) ssGetInputPortSignal(S,264);
	const real_T *C_23 = (const real_T*) ssGetInputPortSignal(S,265);
	const real_T *C_24 = (const real_T*) ssGetInputPortSignal(S,266);
	const real_T *C_25 = (const real_T*) ssGetInputPortSignal(S,267);
	const real_T *C_26 = (const real_T*) ssGetInputPortSignal(S,268);
	const real_T *C_27 = (const real_T*) ssGetInputPortSignal(S,269);
	const real_T *C_28 = (const real_T*) ssGetInputPortSignal(S,270);
	const real_T *C_29 = (const real_T*) ssGetInputPortSignal(S,271);
	const real_T *C_30 = (const real_T*) ssGetInputPortSignal(S,272);
	const real_T *C_31 = (const real_T*) ssGetInputPortSignal(S,273);
	const real_T *C_32 = (const real_T*) ssGetInputPortSignal(S,274);
	const real_T *C_33 = (const real_T*) ssGetInputPortSignal(S,275);
	const real_T *C_34 = (const real_T*) ssGetInputPortSignal(S,276);
	const real_T *C_35 = (const real_T*) ssGetInputPortSignal(S,277);
	const real_T *C_36 = (const real_T*) ssGetInputPortSignal(S,278);
	const real_T *C_37 = (const real_T*) ssGetInputPortSignal(S,279);
	const real_T *C_38 = (const real_T*) ssGetInputPortSignal(S,280);
	const real_T *C_39 = (const real_T*) ssGetInputPortSignal(S,281);
	const real_T *C_40 = (const real_T*) ssGetInputPortSignal(S,282);
	const real_T *C_41 = (const real_T*) ssGetInputPortSignal(S,283);
	const real_T *C_42 = (const real_T*) ssGetInputPortSignal(S,284);
	const real_T *C_43 = (const real_T*) ssGetInputPortSignal(S,285);
	const real_T *C_44 = (const real_T*) ssGetInputPortSignal(S,286);
	const real_T *C_45 = (const real_T*) ssGetInputPortSignal(S,287);
	const real_T *C_46 = (const real_T*) ssGetInputPortSignal(S,288);
	const real_T *C_47 = (const real_T*) ssGetInputPortSignal(S,289);
	const real_T *C_48 = (const real_T*) ssGetInputPortSignal(S,290);
	const real_T *C_49 = (const real_T*) ssGetInputPortSignal(S,291);
	const real_T *C_50 = (const real_T*) ssGetInputPortSignal(S,292);
	const real_T *C_51 = (const real_T*) ssGetInputPortSignal(S,293);
	const real_T *C_52 = (const real_T*) ssGetInputPortSignal(S,294);
	const real_T *C_53 = (const real_T*) ssGetInputPortSignal(S,295);
	const real_T *C_54 = (const real_T*) ssGetInputPortSignal(S,296);
	const real_T *C_55 = (const real_T*) ssGetInputPortSignal(S,297);
	const real_T *C_56 = (const real_T*) ssGetInputPortSignal(S,298);
	const real_T *C_57 = (const real_T*) ssGetInputPortSignal(S,299);
	const real_T *C_58 = (const real_T*) ssGetInputPortSignal(S,300);
	const real_T *C_59 = (const real_T*) ssGetInputPortSignal(S,301);
	const real_T *C_60 = (const real_T*) ssGetInputPortSignal(S,302);
	const real_T *C_61 = (const real_T*) ssGetInputPortSignal(S,303);
	const real_T *C_62 = (const real_T*) ssGetInputPortSignal(S,304);
	const real_T *C_63 = (const real_T*) ssGetInputPortSignal(S,305);
	const real_T *C_64 = (const real_T*) ssGetInputPortSignal(S,306);
	const real_T *C_65 = (const real_T*) ssGetInputPortSignal(S,307);
	const real_T *C_66 = (const real_T*) ssGetInputPortSignal(S,308);
	const real_T *C_67 = (const real_T*) ssGetInputPortSignal(S,309);
	const real_T *C_68 = (const real_T*) ssGetInputPortSignal(S,310);
	const real_T *C_69 = (const real_T*) ssGetInputPortSignal(S,311);
	const real_T *C_70 = (const real_T*) ssGetInputPortSignal(S,312);
	const real_T *C_71 = (const real_T*) ssGetInputPortSignal(S,313);
	const real_T *C_72 = (const real_T*) ssGetInputPortSignal(S,314);
	const real_T *C_73 = (const real_T*) ssGetInputPortSignal(S,315);
	const real_T *C_74 = (const real_T*) ssGetInputPortSignal(S,316);
	const real_T *C_75 = (const real_T*) ssGetInputPortSignal(S,317);
	const real_T *C_76 = (const real_T*) ssGetInputPortSignal(S,318);
	const real_T *C_77 = (const real_T*) ssGetInputPortSignal(S,319);
	const real_T *C_78 = (const real_T*) ssGetInputPortSignal(S,320);
	const real_T *C_79 = (const real_T*) ssGetInputPortSignal(S,321);
	const real_T *C_80 = (const real_T*) ssGetInputPortSignal(S,322);
	const real_T *A_2 = (const real_T*) ssGetInputPortSignal(S,323);
	const real_T *A_3 = (const real_T*) ssGetInputPortSignal(S,324);
	const real_T *A_4 = (const real_T*) ssGetInputPortSignal(S,325);
	const real_T *A_5 = (const real_T*) ssGetInputPortSignal(S,326);
	const real_T *A_6 = (const real_T*) ssGetInputPortSignal(S,327);
	const real_T *A_7 = (const real_T*) ssGetInputPortSignal(S,328);
	const real_T *A_8 = (const real_T*) ssGetInputPortSignal(S,329);
	const real_T *A_9 = (const real_T*) ssGetInputPortSignal(S,330);
	const real_T *A_10 = (const real_T*) ssGetInputPortSignal(S,331);
	const real_T *A_11 = (const real_T*) ssGetInputPortSignal(S,332);
	const real_T *A_12 = (const real_T*) ssGetInputPortSignal(S,333);
	const real_T *A_13 = (const real_T*) ssGetInputPortSignal(S,334);
	const real_T *A_14 = (const real_T*) ssGetInputPortSignal(S,335);
	const real_T *A_15 = (const real_T*) ssGetInputPortSignal(S,336);
	const real_T *A_16 = (const real_T*) ssGetInputPortSignal(S,337);
	const real_T *A_17 = (const real_T*) ssGetInputPortSignal(S,338);
	const real_T *A_18 = (const real_T*) ssGetInputPortSignal(S,339);
	const real_T *A_19 = (const real_T*) ssGetInputPortSignal(S,340);
	const real_T *A_20 = (const real_T*) ssGetInputPortSignal(S,341);
	const real_T *A_21 = (const real_T*) ssGetInputPortSignal(S,342);
	const real_T *A_22 = (const real_T*) ssGetInputPortSignal(S,343);
	const real_T *A_23 = (const real_T*) ssGetInputPortSignal(S,344);
	const real_T *A_24 = (const real_T*) ssGetInputPortSignal(S,345);
	const real_T *A_25 = (const real_T*) ssGetInputPortSignal(S,346);
	const real_T *A_26 = (const real_T*) ssGetInputPortSignal(S,347);
	const real_T *A_27 = (const real_T*) ssGetInputPortSignal(S,348);
	const real_T *A_28 = (const real_T*) ssGetInputPortSignal(S,349);
	const real_T *A_29 = (const real_T*) ssGetInputPortSignal(S,350);
	const real_T *A_30 = (const real_T*) ssGetInputPortSignal(S,351);
	const real_T *A_31 = (const real_T*) ssGetInputPortSignal(S,352);
	const real_T *A_32 = (const real_T*) ssGetInputPortSignal(S,353);
	const real_T *A_33 = (const real_T*) ssGetInputPortSignal(S,354);
	const real_T *A_34 = (const real_T*) ssGetInputPortSignal(S,355);
	const real_T *A_35 = (const real_T*) ssGetInputPortSignal(S,356);
	const real_T *A_36 = (const real_T*) ssGetInputPortSignal(S,357);
	const real_T *A_37 = (const real_T*) ssGetInputPortSignal(S,358);
	const real_T *A_38 = (const real_T*) ssGetInputPortSignal(S,359);
	const real_T *A_39 = (const real_T*) ssGetInputPortSignal(S,360);
	const real_T *A_40 = (const real_T*) ssGetInputPortSignal(S,361);
	const real_T *A_41 = (const real_T*) ssGetInputPortSignal(S,362);
	const real_T *A_42 = (const real_T*) ssGetInputPortSignal(S,363);
	const real_T *A_43 = (const real_T*) ssGetInputPortSignal(S,364);
	const real_T *A_44 = (const real_T*) ssGetInputPortSignal(S,365);
	const real_T *A_45 = (const real_T*) ssGetInputPortSignal(S,366);
	const real_T *A_46 = (const real_T*) ssGetInputPortSignal(S,367);
	const real_T *A_47 = (const real_T*) ssGetInputPortSignal(S,368);
	const real_T *A_48 = (const real_T*) ssGetInputPortSignal(S,369);
	const real_T *A_49 = (const real_T*) ssGetInputPortSignal(S,370);
	const real_T *A_50 = (const real_T*) ssGetInputPortSignal(S,371);
	const real_T *A_51 = (const real_T*) ssGetInputPortSignal(S,372);
	const real_T *A_52 = (const real_T*) ssGetInputPortSignal(S,373);
	const real_T *A_53 = (const real_T*) ssGetInputPortSignal(S,374);
	const real_T *A_54 = (const real_T*) ssGetInputPortSignal(S,375);
	const real_T *A_55 = (const real_T*) ssGetInputPortSignal(S,376);
	const real_T *A_56 = (const real_T*) ssGetInputPortSignal(S,377);
	const real_T *A_57 = (const real_T*) ssGetInputPortSignal(S,378);
	const real_T *A_58 = (const real_T*) ssGetInputPortSignal(S,379);
	const real_T *A_59 = (const real_T*) ssGetInputPortSignal(S,380);
	const real_T *A_60 = (const real_T*) ssGetInputPortSignal(S,381);
	const real_T *A_61 = (const real_T*) ssGetInputPortSignal(S,382);
	const real_T *A_62 = (const real_T*) ssGetInputPortSignal(S,383);
	const real_T *A_63 = (const real_T*) ssGetInputPortSignal(S,384);
	const real_T *A_64 = (const real_T*) ssGetInputPortSignal(S,385);
	const real_T *A_65 = (const real_T*) ssGetInputPortSignal(S,386);
	const real_T *A_66 = (const real_T*) ssGetInputPortSignal(S,387);
	const real_T *A_67 = (const real_T*) ssGetInputPortSignal(S,388);
	const real_T *A_68 = (const real_T*) ssGetInputPortSignal(S,389);
	const real_T *A_69 = (const real_T*) ssGetInputPortSignal(S,390);
	const real_T *A_70 = (const real_T*) ssGetInputPortSignal(S,391);
	const real_T *A_71 = (const real_T*) ssGetInputPortSignal(S,392);
	const real_T *A_72 = (const real_T*) ssGetInputPortSignal(S,393);
	const real_T *A_73 = (const real_T*) ssGetInputPortSignal(S,394);
	const real_T *A_74 = (const real_T*) ssGetInputPortSignal(S,395);
	const real_T *A_75 = (const real_T*) ssGetInputPortSignal(S,396);
	const real_T *A_76 = (const real_T*) ssGetInputPortSignal(S,397);
	const real_T *A_77 = (const real_T*) ssGetInputPortSignal(S,398);
	const real_T *A_78 = (const real_T*) ssGetInputPortSignal(S,399);
	const real_T *A_79 = (const real_T*) ssGetInputPortSignal(S,400);
	const real_T *A_80 = (const real_T*) ssGetInputPortSignal(S,401);
	const real_T *A_81 = (const real_T*) ssGetInputPortSignal(S,402);
	const real_T *b_2 = (const real_T*) ssGetInputPortSignal(S,403);
	const real_T *b_3 = (const real_T*) ssGetInputPortSignal(S,404);
	const real_T *b_4 = (const real_T*) ssGetInputPortSignal(S,405);
	const real_T *b_5 = (const real_T*) ssGetInputPortSignal(S,406);
	const real_T *b_6 = (const real_T*) ssGetInputPortSignal(S,407);
	const real_T *b_7 = (const real_T*) ssGetInputPortSignal(S,408);
	const real_T *b_8 = (const real_T*) ssGetInputPortSignal(S,409);
	const real_T *b_9 = (const real_T*) ssGetInputPortSignal(S,410);
	const real_T *b_10 = (const real_T*) ssGetInputPortSignal(S,411);
	const real_T *b_11 = (const real_T*) ssGetInputPortSignal(S,412);
	const real_T *b_12 = (const real_T*) ssGetInputPortSignal(S,413);
	const real_T *b_13 = (const real_T*) ssGetInputPortSignal(S,414);
	const real_T *b_14 = (const real_T*) ssGetInputPortSignal(S,415);
	const real_T *b_15 = (const real_T*) ssGetInputPortSignal(S,416);
	const real_T *b_16 = (const real_T*) ssGetInputPortSignal(S,417);
	const real_T *b_17 = (const real_T*) ssGetInputPortSignal(S,418);
	const real_T *b_18 = (const real_T*) ssGetInputPortSignal(S,419);
	const real_T *b_19 = (const real_T*) ssGetInputPortSignal(S,420);
	const real_T *b_20 = (const real_T*) ssGetInputPortSignal(S,421);
	const real_T *b_21 = (const real_T*) ssGetInputPortSignal(S,422);
	const real_T *b_22 = (const real_T*) ssGetInputPortSignal(S,423);
	const real_T *b_23 = (const real_T*) ssGetInputPortSignal(S,424);
	const real_T *b_24 = (const real_T*) ssGetInputPortSignal(S,425);
	const real_T *b_25 = (const real_T*) ssGetInputPortSignal(S,426);
	const real_T *b_26 = (const real_T*) ssGetInputPortSignal(S,427);
	const real_T *b_27 = (const real_T*) ssGetInputPortSignal(S,428);
	const real_T *b_28 = (const real_T*) ssGetInputPortSignal(S,429);
	const real_T *b_29 = (const real_T*) ssGetInputPortSignal(S,430);
	const real_T *b_30 = (const real_T*) ssGetInputPortSignal(S,431);
	const real_T *b_31 = (const real_T*) ssGetInputPortSignal(S,432);
	const real_T *b_32 = (const real_T*) ssGetInputPortSignal(S,433);
	const real_T *b_33 = (const real_T*) ssGetInputPortSignal(S,434);
	const real_T *b_34 = (const real_T*) ssGetInputPortSignal(S,435);
	const real_T *b_35 = (const real_T*) ssGetInputPortSignal(S,436);
	const real_T *b_36 = (const real_T*) ssGetInputPortSignal(S,437);
	const real_T *b_37 = (const real_T*) ssGetInputPortSignal(S,438);
	const real_T *b_38 = (const real_T*) ssGetInputPortSignal(S,439);
	const real_T *b_39 = (const real_T*) ssGetInputPortSignal(S,440);
	const real_T *b_40 = (const real_T*) ssGetInputPortSignal(S,441);
	const real_T *b_41 = (const real_T*) ssGetInputPortSignal(S,442);
	const real_T *b_42 = (const real_T*) ssGetInputPortSignal(S,443);
	const real_T *b_43 = (const real_T*) ssGetInputPortSignal(S,444);
	const real_T *b_44 = (const real_T*) ssGetInputPortSignal(S,445);
	const real_T *b_45 = (const real_T*) ssGetInputPortSignal(S,446);
	const real_T *b_46 = (const real_T*) ssGetInputPortSignal(S,447);
	const real_T *b_47 = (const real_T*) ssGetInputPortSignal(S,448);
	const real_T *b_48 = (const real_T*) ssGetInputPortSignal(S,449);
	const real_T *b_49 = (const real_T*) ssGetInputPortSignal(S,450);
	const real_T *b_50 = (const real_T*) ssGetInputPortSignal(S,451);
	const real_T *b_51 = (const real_T*) ssGetInputPortSignal(S,452);
	const real_T *b_52 = (const real_T*) ssGetInputPortSignal(S,453);
	const real_T *b_53 = (const real_T*) ssGetInputPortSignal(S,454);
	const real_T *b_54 = (const real_T*) ssGetInputPortSignal(S,455);
	const real_T *b_55 = (const real_T*) ssGetInputPortSignal(S,456);
	const real_T *b_56 = (const real_T*) ssGetInputPortSignal(S,457);
	const real_T *b_57 = (const real_T*) ssGetInputPortSignal(S,458);
	const real_T *b_58 = (const real_T*) ssGetInputPortSignal(S,459);
	const real_T *b_59 = (const real_T*) ssGetInputPortSignal(S,460);
	const real_T *b_60 = (const real_T*) ssGetInputPortSignal(S,461);
	const real_T *b_61 = (const real_T*) ssGetInputPortSignal(S,462);
	const real_T *b_62 = (const real_T*) ssGetInputPortSignal(S,463);
	const real_T *b_63 = (const real_T*) ssGetInputPortSignal(S,464);
	const real_T *b_64 = (const real_T*) ssGetInputPortSignal(S,465);
	const real_T *b_65 = (const real_T*) ssGetInputPortSignal(S,466);
	const real_T *b_66 = (const real_T*) ssGetInputPortSignal(S,467);
	const real_T *b_67 = (const real_T*) ssGetInputPortSignal(S,468);
	const real_T *b_68 = (const real_T*) ssGetInputPortSignal(S,469);
	const real_T *b_69 = (const real_T*) ssGetInputPortSignal(S,470);
	const real_T *b_70 = (const real_T*) ssGetInputPortSignal(S,471);
	const real_T *b_71 = (const real_T*) ssGetInputPortSignal(S,472);
	const real_T *b_72 = (const real_T*) ssGetInputPortSignal(S,473);
	const real_T *b_73 = (const real_T*) ssGetInputPortSignal(S,474);
	const real_T *b_74 = (const real_T*) ssGetInputPortSignal(S,475);
	const real_T *b_75 = (const real_T*) ssGetInputPortSignal(S,476);
	const real_T *b_76 = (const real_T*) ssGetInputPortSignal(S,477);
	const real_T *b_77 = (const real_T*) ssGetInputPortSignal(S,478);
	const real_T *b_78 = (const real_T*) ssGetInputPortSignal(S,479);
	const real_T *b_79 = (const real_T*) ssGetInputPortSignal(S,480);
	const real_T *b_80 = (const real_T*) ssGetInputPortSignal(S,481);
	const real_T *b_81 = (const real_T*) ssGetInputPortSignal(S,482);
	
    real_T *X = (real_T*) ssGetOutputPortSignal(S,0);
	
	

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

	for( i=0; i<10; i++)
	{ 
		params.c_42[i] = (double) c_42[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_43[i] = (double) c_43[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_44[i] = (double) c_44[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_45[i] = (double) c_45[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_46[i] = (double) c_46[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_47[i] = (double) c_47[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_48[i] = (double) c_48[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_49[i] = (double) c_49[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_50[i] = (double) c_50[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_51[i] = (double) c_51[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_52[i] = (double) c_52[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_53[i] = (double) c_53[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_54[i] = (double) c_54[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_55[i] = (double) c_55[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_56[i] = (double) c_56[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_57[i] = (double) c_57[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_58[i] = (double) c_58[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_59[i] = (double) c_59[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_60[i] = (double) c_60[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_61[i] = (double) c_61[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_62[i] = (double) c_62[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_63[i] = (double) c_63[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_64[i] = (double) c_64[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_65[i] = (double) c_65[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_66[i] = (double) c_66[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_67[i] = (double) c_67[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_68[i] = (double) c_68[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_69[i] = (double) c_69[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_70[i] = (double) c_70[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_71[i] = (double) c_71[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_72[i] = (double) c_72[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_73[i] = (double) c_73[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_74[i] = (double) c_74[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_75[i] = (double) c_75[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_76[i] = (double) c_76[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_77[i] = (double) c_77[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_78[i] = (double) c_78[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_79[i] = (double) c_79[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_80[i] = (double) c_80[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c_81[i] = (double) c_81[i]; 
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

	for( i=0; i<196; i++)
	{ 
		params.H_42[i] = (double) H_42[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_43[i] = (double) H_43[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_44[i] = (double) H_44[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_45[i] = (double) H_45[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_46[i] = (double) H_46[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_47[i] = (double) H_47[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_48[i] = (double) H_48[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_49[i] = (double) H_49[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_50[i] = (double) H_50[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_51[i] = (double) H_51[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_52[i] = (double) H_52[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_53[i] = (double) H_53[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_54[i] = (double) H_54[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_55[i] = (double) H_55[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_56[i] = (double) H_56[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_57[i] = (double) H_57[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_58[i] = (double) H_58[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_59[i] = (double) H_59[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_60[i] = (double) H_60[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_61[i] = (double) H_61[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_62[i] = (double) H_62[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_63[i] = (double) H_63[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_64[i] = (double) H_64[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_65[i] = (double) H_65[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_66[i] = (double) H_66[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_67[i] = (double) H_67[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_68[i] = (double) H_68[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_69[i] = (double) H_69[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_70[i] = (double) H_70[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_71[i] = (double) H_71[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_72[i] = (double) H_72[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_73[i] = (double) H_73[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_74[i] = (double) H_74[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_75[i] = (double) H_75[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_76[i] = (double) H_76[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_77[i] = (double) H_77[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_78[i] = (double) H_78[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_79[i] = (double) H_79[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_80[i] = (double) H_80[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H_81[i] = (double) H_81[i]; 
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

	for( i=0; i<14; i++)
	{ 
		params.f_42[i] = (double) f_42[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_43[i] = (double) f_43[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_44[i] = (double) f_44[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_45[i] = (double) f_45[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_46[i] = (double) f_46[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_47[i] = (double) f_47[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_48[i] = (double) f_48[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_49[i] = (double) f_49[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_50[i] = (double) f_50[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_51[i] = (double) f_51[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_52[i] = (double) f_52[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_53[i] = (double) f_53[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_54[i] = (double) f_54[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_55[i] = (double) f_55[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_56[i] = (double) f_56[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_57[i] = (double) f_57[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_58[i] = (double) f_58[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_59[i] = (double) f_59[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_60[i] = (double) f_60[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_61[i] = (double) f_61[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_62[i] = (double) f_62[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_63[i] = (double) f_63[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_64[i] = (double) f_64[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_65[i] = (double) f_65[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_66[i] = (double) f_66[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_67[i] = (double) f_67[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_68[i] = (double) f_68[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_69[i] = (double) f_69[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_70[i] = (double) f_70[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_71[i] = (double) f_71[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_72[i] = (double) f_72[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_73[i] = (double) f_73[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_74[i] = (double) f_74[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_75[i] = (double) f_75[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_76[i] = (double) f_76[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_77[i] = (double) f_77[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_78[i] = (double) f_78[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_79[i] = (double) f_79[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_80[i] = (double) f_80[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f_81[i] = (double) f_81[i]; 
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

	for( i=0; i<140; i++)
	{ 
		params.C_41[i] = (double) C_41[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_42[i] = (double) C_42[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_43[i] = (double) C_43[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_44[i] = (double) C_44[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_45[i] = (double) C_45[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_46[i] = (double) C_46[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_47[i] = (double) C_47[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_48[i] = (double) C_48[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_49[i] = (double) C_49[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_50[i] = (double) C_50[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_51[i] = (double) C_51[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_52[i] = (double) C_52[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_53[i] = (double) C_53[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_54[i] = (double) C_54[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_55[i] = (double) C_55[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_56[i] = (double) C_56[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_57[i] = (double) C_57[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_58[i] = (double) C_58[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_59[i] = (double) C_59[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_60[i] = (double) C_60[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_61[i] = (double) C_61[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_62[i] = (double) C_62[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_63[i] = (double) C_63[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_64[i] = (double) C_64[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_65[i] = (double) C_65[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_66[i] = (double) C_66[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_67[i] = (double) C_67[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_68[i] = (double) C_68[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_69[i] = (double) C_69[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_70[i] = (double) C_70[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_71[i] = (double) C_71[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_72[i] = (double) C_72[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_73[i] = (double) C_73[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_74[i] = (double) C_74[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_75[i] = (double) C_75[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_76[i] = (double) C_76[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_77[i] = (double) C_77[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_78[i] = (double) C_78[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_79[i] = (double) C_79[i]; 
	}

	for( i=0; i<140; i++)
	{ 
		params.C_80[i] = (double) C_80[i]; 
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

	for( i=0; i<28; i++)
	{ 
		params.A_42[i] = (double) A_42[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_43[i] = (double) A_43[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_44[i] = (double) A_44[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_45[i] = (double) A_45[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_46[i] = (double) A_46[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_47[i] = (double) A_47[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_48[i] = (double) A_48[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_49[i] = (double) A_49[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_50[i] = (double) A_50[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_51[i] = (double) A_51[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_52[i] = (double) A_52[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_53[i] = (double) A_53[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_54[i] = (double) A_54[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_55[i] = (double) A_55[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_56[i] = (double) A_56[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_57[i] = (double) A_57[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_58[i] = (double) A_58[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_59[i] = (double) A_59[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_60[i] = (double) A_60[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_61[i] = (double) A_61[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_62[i] = (double) A_62[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_63[i] = (double) A_63[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_64[i] = (double) A_64[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_65[i] = (double) A_65[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_66[i] = (double) A_66[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_67[i] = (double) A_67[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_68[i] = (double) A_68[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_69[i] = (double) A_69[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_70[i] = (double) A_70[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_71[i] = (double) A_71[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_72[i] = (double) A_72[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_73[i] = (double) A_73[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_74[i] = (double) A_74[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_75[i] = (double) A_75[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_76[i] = (double) A_76[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_77[i] = (double) A_77[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_78[i] = (double) A_78[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_79[i] = (double) A_79[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_80[i] = (double) A_80[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A_81[i] = (double) A_81[i]; 
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

	for( i=0; i<2; i++)
	{ 
		params.b_42[i] = (double) b_42[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_43[i] = (double) b_43[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_44[i] = (double) b_44[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_45[i] = (double) b_45[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_46[i] = (double) b_46[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_47[i] = (double) b_47[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_48[i] = (double) b_48[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_49[i] = (double) b_49[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_50[i] = (double) b_50[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_51[i] = (double) b_51[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_52[i] = (double) b_52[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_53[i] = (double) b_53[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_54[i] = (double) b_54[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_55[i] = (double) b_55[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_56[i] = (double) b_56[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_57[i] = (double) b_57[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_58[i] = (double) b_58[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_59[i] = (double) b_59[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_60[i] = (double) b_60[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_61[i] = (double) b_61[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_62[i] = (double) b_62[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_63[i] = (double) b_63[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_64[i] = (double) b_64[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_65[i] = (double) b_65[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_66[i] = (double) b_66[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_67[i] = (double) b_67[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_68[i] = (double) b_68[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_69[i] = (double) b_69[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_70[i] = (double) b_70[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_71[i] = (double) b_71[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_72[i] = (double) b_72[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_73[i] = (double) b_73[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_74[i] = (double) b_74[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_75[i] = (double) b_75[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_76[i] = (double) b_76[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_77[i] = (double) b_77[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_78[i] = (double) b_78[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_79[i] = (double) b_79[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_80[i] = (double) b_80[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b_81[i] = (double) b_81[i]; 
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



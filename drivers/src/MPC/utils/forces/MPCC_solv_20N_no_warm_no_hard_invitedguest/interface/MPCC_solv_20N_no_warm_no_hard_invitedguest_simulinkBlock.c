/*
MPCC_solv_20N_no_warm_no_hard_invitedguest : A fast customized optimization solver.

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
#define S_FUNCTION_NAME MPCC_solv_20N_no_warm_no_hard_invitedguest_simulinkBlock

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/MPCC_solv_20N_no_warm_no_hard_invitedguest.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef MPCC_solv_20N_no_warm_no_hard_invitedguestinterface_float MPCC_solv_20N_no_warm_no_hard_invitedguestnmpc_float;





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

	/* initialize input ports - there are 123 in total */
    if (!ssSetNumInputPorts(S, 123)) return;
    	
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
    ssSetInputPortMatrixDimensions(S,  21, 14, 14);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/
	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 14, 14);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/
	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 14, 14);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/
	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 14, 14);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/
	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 14, 14);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/
	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 14, 14);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/
	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 14, 14);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/
	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 14, 14);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/
	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 14, 14);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/
	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 14, 14);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/
	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 14, 14);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/
	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 14, 14);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/
	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 14, 14);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/
	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 14, 14);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/
	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 14, 14);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/
	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 14, 14);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/
	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 14, 14);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/
	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 14, 14);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/
	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 14, 14);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/
	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 14, 14);
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
    ssSetInputPortMatrixDimensions(S,  42, 14, 1);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/
	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 14, 1);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/
	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 14, 1);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/
	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 14, 1);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/
	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 14, 1);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/
	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 14, 1);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/
	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 14, 1);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/
	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 14, 1);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/
	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 14, 1);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/
	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 14, 1);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/
	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 14, 1);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/
	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 14, 1);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/
	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 14, 1);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/
	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 14, 1);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/
	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 14, 1);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/
	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 14, 1);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/
	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 14, 1);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/
	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 14, 1);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/
	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 14, 1);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/
	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 14, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/
	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 14, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/
	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 10, 14);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/
	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 10, 14);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/
	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 10, 14);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/
	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 10, 14);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/
	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 10, 14);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/
	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 10, 14);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/
	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 10, 14);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/
	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 10, 14);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/
	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 10, 14);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/
	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 10, 14);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/
	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 10, 14);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/
	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 10, 14);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/
	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 10, 14);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/
	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 10, 14);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/
	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 10, 14);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/
	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 10, 14);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/
	
	/* Input Port 79 */
    ssSetInputPortMatrixDimensions(S,  79, 10, 14);
    ssSetInputPortDataType(S, 79, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 79, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 79, 1); /*direct input signal access*/
	
	/* Input Port 80 */
    ssSetInputPortMatrixDimensions(S,  80, 10, 14);
    ssSetInputPortDataType(S, 80, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 80, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 80, 1); /*direct input signal access*/
	
	/* Input Port 81 */
    ssSetInputPortMatrixDimensions(S,  81, 10, 14);
    ssSetInputPortDataType(S, 81, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 81, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 81, 1); /*direct input signal access*/
	
	/* Input Port 82 */
    ssSetInputPortMatrixDimensions(S,  82, 10, 14);
    ssSetInputPortDataType(S, 82, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 82, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 82, 1); /*direct input signal access*/
	
	/* Input Port 83 */
    ssSetInputPortMatrixDimensions(S,  83, 2, 14);
    ssSetInputPortDataType(S, 83, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 83, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 83, 1); /*direct input signal access*/
	
	/* Input Port 84 */
    ssSetInputPortMatrixDimensions(S,  84, 2, 14);
    ssSetInputPortDataType(S, 84, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 84, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 84, 1); /*direct input signal access*/
	
	/* Input Port 85 */
    ssSetInputPortMatrixDimensions(S,  85, 2, 14);
    ssSetInputPortDataType(S, 85, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 85, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 85, 1); /*direct input signal access*/
	
	/* Input Port 86 */
    ssSetInputPortMatrixDimensions(S,  86, 2, 14);
    ssSetInputPortDataType(S, 86, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 86, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 86, 1); /*direct input signal access*/
	
	/* Input Port 87 */
    ssSetInputPortMatrixDimensions(S,  87, 2, 14);
    ssSetInputPortDataType(S, 87, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 87, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 87, 1); /*direct input signal access*/
	
	/* Input Port 88 */
    ssSetInputPortMatrixDimensions(S,  88, 2, 14);
    ssSetInputPortDataType(S, 88, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 88, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 88, 1); /*direct input signal access*/
	
	/* Input Port 89 */
    ssSetInputPortMatrixDimensions(S,  89, 2, 14);
    ssSetInputPortDataType(S, 89, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 89, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 89, 1); /*direct input signal access*/
	
	/* Input Port 90 */
    ssSetInputPortMatrixDimensions(S,  90, 2, 14);
    ssSetInputPortDataType(S, 90, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 90, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 90, 1); /*direct input signal access*/
	
	/* Input Port 91 */
    ssSetInputPortMatrixDimensions(S,  91, 2, 14);
    ssSetInputPortDataType(S, 91, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 91, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 91, 1); /*direct input signal access*/
	
	/* Input Port 92 */
    ssSetInputPortMatrixDimensions(S,  92, 2, 14);
    ssSetInputPortDataType(S, 92, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 92, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 92, 1); /*direct input signal access*/
	
	/* Input Port 93 */
    ssSetInputPortMatrixDimensions(S,  93, 2, 14);
    ssSetInputPortDataType(S, 93, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 93, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 93, 1); /*direct input signal access*/
	
	/* Input Port 94 */
    ssSetInputPortMatrixDimensions(S,  94, 2, 14);
    ssSetInputPortDataType(S, 94, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 94, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 94, 1); /*direct input signal access*/
	
	/* Input Port 95 */
    ssSetInputPortMatrixDimensions(S,  95, 2, 14);
    ssSetInputPortDataType(S, 95, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 95, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 95, 1); /*direct input signal access*/
	
	/* Input Port 96 */
    ssSetInputPortMatrixDimensions(S,  96, 2, 14);
    ssSetInputPortDataType(S, 96, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 96, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 96, 1); /*direct input signal access*/
	
	/* Input Port 97 */
    ssSetInputPortMatrixDimensions(S,  97, 2, 14);
    ssSetInputPortDataType(S, 97, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 97, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 97, 1); /*direct input signal access*/
	
	/* Input Port 98 */
    ssSetInputPortMatrixDimensions(S,  98, 2, 14);
    ssSetInputPortDataType(S, 98, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 98, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 98, 1); /*direct input signal access*/
	
	/* Input Port 99 */
    ssSetInputPortMatrixDimensions(S,  99, 2, 14);
    ssSetInputPortDataType(S, 99, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 99, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 99, 1); /*direct input signal access*/
	
	/* Input Port 100 */
    ssSetInputPortMatrixDimensions(S,  100, 2, 14);
    ssSetInputPortDataType(S, 100, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 100, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 100, 1); /*direct input signal access*/
	
	/* Input Port 101 */
    ssSetInputPortMatrixDimensions(S,  101, 2, 14);
    ssSetInputPortDataType(S, 101, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 101, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 101, 1); /*direct input signal access*/
	
	/* Input Port 102 */
    ssSetInputPortMatrixDimensions(S,  102, 2, 14);
    ssSetInputPortDataType(S, 102, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 102, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 102, 1); /*direct input signal access*/
	
	/* Input Port 103 */
    ssSetInputPortMatrixDimensions(S,  103, 2, 1);
    ssSetInputPortDataType(S, 103, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 103, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 103, 1); /*direct input signal access*/
	
	/* Input Port 104 */
    ssSetInputPortMatrixDimensions(S,  104, 2, 1);
    ssSetInputPortDataType(S, 104, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 104, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 104, 1); /*direct input signal access*/
	
	/* Input Port 105 */
    ssSetInputPortMatrixDimensions(S,  105, 2, 1);
    ssSetInputPortDataType(S, 105, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 105, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 105, 1); /*direct input signal access*/
	
	/* Input Port 106 */
    ssSetInputPortMatrixDimensions(S,  106, 2, 1);
    ssSetInputPortDataType(S, 106, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 106, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 106, 1); /*direct input signal access*/
	
	/* Input Port 107 */
    ssSetInputPortMatrixDimensions(S,  107, 2, 1);
    ssSetInputPortDataType(S, 107, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 107, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 107, 1); /*direct input signal access*/
	
	/* Input Port 108 */
    ssSetInputPortMatrixDimensions(S,  108, 2, 1);
    ssSetInputPortDataType(S, 108, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 108, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 108, 1); /*direct input signal access*/
	
	/* Input Port 109 */
    ssSetInputPortMatrixDimensions(S,  109, 2, 1);
    ssSetInputPortDataType(S, 109, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 109, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 109, 1); /*direct input signal access*/
	
	/* Input Port 110 */
    ssSetInputPortMatrixDimensions(S,  110, 2, 1);
    ssSetInputPortDataType(S, 110, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 110, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 110, 1); /*direct input signal access*/
	
	/* Input Port 111 */
    ssSetInputPortMatrixDimensions(S,  111, 2, 1);
    ssSetInputPortDataType(S, 111, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 111, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 111, 1); /*direct input signal access*/
	
	/* Input Port 112 */
    ssSetInputPortMatrixDimensions(S,  112, 2, 1);
    ssSetInputPortDataType(S, 112, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 112, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 112, 1); /*direct input signal access*/
	
	/* Input Port 113 */
    ssSetInputPortMatrixDimensions(S,  113, 2, 1);
    ssSetInputPortDataType(S, 113, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 113, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 113, 1); /*direct input signal access*/
	
	/* Input Port 114 */
    ssSetInputPortMatrixDimensions(S,  114, 2, 1);
    ssSetInputPortDataType(S, 114, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 114, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 114, 1); /*direct input signal access*/
	
	/* Input Port 115 */
    ssSetInputPortMatrixDimensions(S,  115, 2, 1);
    ssSetInputPortDataType(S, 115, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 115, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 115, 1); /*direct input signal access*/
	
	/* Input Port 116 */
    ssSetInputPortMatrixDimensions(S,  116, 2, 1);
    ssSetInputPortDataType(S, 116, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 116, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 116, 1); /*direct input signal access*/
	
	/* Input Port 117 */
    ssSetInputPortMatrixDimensions(S,  117, 2, 1);
    ssSetInputPortDataType(S, 117, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 117, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 117, 1); /*direct input signal access*/
	
	/* Input Port 118 */
    ssSetInputPortMatrixDimensions(S,  118, 2, 1);
    ssSetInputPortDataType(S, 118, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 118, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 118, 1); /*direct input signal access*/
	
	/* Input Port 119 */
    ssSetInputPortMatrixDimensions(S,  119, 2, 1);
    ssSetInputPortDataType(S, 119, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 119, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 119, 1); /*direct input signal access*/
	
	/* Input Port 120 */
    ssSetInputPortMatrixDimensions(S,  120, 2, 1);
    ssSetInputPortDataType(S, 120, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 120, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 120, 1); /*direct input signal access*/
	
	/* Input Port 121 */
    ssSetInputPortMatrixDimensions(S,  121, 2, 1);
    ssSetInputPortDataType(S, 121, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 121, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 121, 1); /*direct input signal access*/
	
	/* Input Port 122 */
    ssSetInputPortMatrixDimensions(S,  122, 2, 1);
    ssSetInputPortDataType(S, 122, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 122, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 122, 1); /*direct input signal access*/
 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 252, 1);
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
	const real_T *H_1 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *H_2 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *H_3 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *H_4 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *H_5 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *H_6 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *H_7 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *H_8 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *H_9 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *H_10 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *H_11 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *H_12 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *H_13 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *H_14 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *H_15 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *H_16 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *H_17 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *H_18 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *H_19 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *H_20 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *H_21 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *f_1 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *f_2 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *f_3 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *f_4 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *f_5 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *f_6 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *f_7 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *f_8 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *f_9 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *f_10 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *f_11 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *f_12 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *f_13 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *f_14 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *f_15 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *f_16 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *f_17 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *f_18 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *f_19 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *f_20 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *f_21 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *C_1 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *C_2 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *C_3 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *C_4 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *C_5 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *C_6 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *C_7 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *C_8 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *C_9 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *C_10 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *C_11 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *C_12 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *C_13 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *C_14 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *C_15 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *C_16 = (const real_T*) ssGetInputPortSignal(S,78);
	const real_T *C_17 = (const real_T*) ssGetInputPortSignal(S,79);
	const real_T *C_18 = (const real_T*) ssGetInputPortSignal(S,80);
	const real_T *C_19 = (const real_T*) ssGetInputPortSignal(S,81);
	const real_T *C_20 = (const real_T*) ssGetInputPortSignal(S,82);
	const real_T *A_2 = (const real_T*) ssGetInputPortSignal(S,83);
	const real_T *A_3 = (const real_T*) ssGetInputPortSignal(S,84);
	const real_T *A_4 = (const real_T*) ssGetInputPortSignal(S,85);
	const real_T *A_5 = (const real_T*) ssGetInputPortSignal(S,86);
	const real_T *A_6 = (const real_T*) ssGetInputPortSignal(S,87);
	const real_T *A_7 = (const real_T*) ssGetInputPortSignal(S,88);
	const real_T *A_8 = (const real_T*) ssGetInputPortSignal(S,89);
	const real_T *A_9 = (const real_T*) ssGetInputPortSignal(S,90);
	const real_T *A_10 = (const real_T*) ssGetInputPortSignal(S,91);
	const real_T *A_11 = (const real_T*) ssGetInputPortSignal(S,92);
	const real_T *A_12 = (const real_T*) ssGetInputPortSignal(S,93);
	const real_T *A_13 = (const real_T*) ssGetInputPortSignal(S,94);
	const real_T *A_14 = (const real_T*) ssGetInputPortSignal(S,95);
	const real_T *A_15 = (const real_T*) ssGetInputPortSignal(S,96);
	const real_T *A_16 = (const real_T*) ssGetInputPortSignal(S,97);
	const real_T *A_17 = (const real_T*) ssGetInputPortSignal(S,98);
	const real_T *A_18 = (const real_T*) ssGetInputPortSignal(S,99);
	const real_T *A_19 = (const real_T*) ssGetInputPortSignal(S,100);
	const real_T *A_20 = (const real_T*) ssGetInputPortSignal(S,101);
	const real_T *A_21 = (const real_T*) ssGetInputPortSignal(S,102);
	const real_T *b_2 = (const real_T*) ssGetInputPortSignal(S,103);
	const real_T *b_3 = (const real_T*) ssGetInputPortSignal(S,104);
	const real_T *b_4 = (const real_T*) ssGetInputPortSignal(S,105);
	const real_T *b_5 = (const real_T*) ssGetInputPortSignal(S,106);
	const real_T *b_6 = (const real_T*) ssGetInputPortSignal(S,107);
	const real_T *b_7 = (const real_T*) ssGetInputPortSignal(S,108);
	const real_T *b_8 = (const real_T*) ssGetInputPortSignal(S,109);
	const real_T *b_9 = (const real_T*) ssGetInputPortSignal(S,110);
	const real_T *b_10 = (const real_T*) ssGetInputPortSignal(S,111);
	const real_T *b_11 = (const real_T*) ssGetInputPortSignal(S,112);
	const real_T *b_12 = (const real_T*) ssGetInputPortSignal(S,113);
	const real_T *b_13 = (const real_T*) ssGetInputPortSignal(S,114);
	const real_T *b_14 = (const real_T*) ssGetInputPortSignal(S,115);
	const real_T *b_15 = (const real_T*) ssGetInputPortSignal(S,116);
	const real_T *b_16 = (const real_T*) ssGetInputPortSignal(S,117);
	const real_T *b_17 = (const real_T*) ssGetInputPortSignal(S,118);
	const real_T *b_18 = (const real_T*) ssGetInputPortSignal(S,119);
	const real_T *b_19 = (const real_T*) ssGetInputPortSignal(S,120);
	const real_T *b_20 = (const real_T*) ssGetInputPortSignal(S,121);
	const real_T *b_21 = (const real_T*) ssGetInputPortSignal(S,122);
	
    real_T *X = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static MPCC_solv_20N_no_warm_no_hard_invitedguest_params params;
	static MPCC_solv_20N_no_warm_no_hard_invitedguest_output output;
	static MPCC_solv_20N_no_warm_no_hard_invitedguest_info info;	
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

	

	

    #if SET_PRINTLEVEL_MPCC_solv_20N_no_warm_no_hard_invitedguest > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = MPCC_solv_20N_no_warm_no_hard_invitedguest_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPCC_solv_20N_no_warm_no_hard_invitedguest > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<252; i++)
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



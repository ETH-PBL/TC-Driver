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

#include "mex.h"
#include "math.h"
#include "../include/MPCC_solv_80N_no_warm_no_hard_invitedguest.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}





/* Some memory for mex-function */
static MPCC_solv_80N_no_warm_no_hard_invitedguest_params params;
static MPCC_solv_80N_no_warm_no_hard_invitedguest_output output;
static MPCC_solv_80N_no_warm_no_hard_invitedguest_info info;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[1] = {"X"};
	const solver_int8_default *infofields[16] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "dobj",  "dgap", "rdgap",  "mu",  "mu_aff",  "sigma",  "lsit_aff",  "lsit_cc",  "step_aff",   "step_cc",  "solvetime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help MPCC_solv_80N_no_warm_no_hard_invitedguest_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help MPCC_solv_80N_no_warm_no_hard_invitedguest_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "c_1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_1 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_1 must be of size [8 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_1,8);

	}
	par = mxGetField(PARAMS, 0, "c_2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_2 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_2 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_2,10);

	}
	par = mxGetField(PARAMS, 0, "c_3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_3 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_3 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_3,10);

	}
	par = mxGetField(PARAMS, 0, "c_4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_4 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_4 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_4,10);

	}
	par = mxGetField(PARAMS, 0, "c_5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_5 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_5 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_5,10);

	}
	par = mxGetField(PARAMS, 0, "c_6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_6 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_6 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_6,10);

	}
	par = mxGetField(PARAMS, 0, "c_7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_7 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_7 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_7,10);

	}
	par = mxGetField(PARAMS, 0, "c_8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_8 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_8 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_8,10);

	}
	par = mxGetField(PARAMS, 0, "c_9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_9 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_9 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_9,10);

	}
	par = mxGetField(PARAMS, 0, "c_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_10 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_10 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_10,10);

	}
	par = mxGetField(PARAMS, 0, "c_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_11 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_11 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_11,10);

	}
	par = mxGetField(PARAMS, 0, "c_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_12 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_12 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_12,10);

	}
	par = mxGetField(PARAMS, 0, "c_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_13 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_13 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_13,10);

	}
	par = mxGetField(PARAMS, 0, "c_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_14 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_14 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_14,10);

	}
	par = mxGetField(PARAMS, 0, "c_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_15 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_15 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_15,10);

	}
	par = mxGetField(PARAMS, 0, "c_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_16 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_16 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_16,10);

	}
	par = mxGetField(PARAMS, 0, "c_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_17 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_17 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_17,10);

	}
	par = mxGetField(PARAMS, 0, "c_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_18 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_18 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_18,10);

	}
	par = mxGetField(PARAMS, 0, "c_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_19 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_19 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_19,10);

	}
	par = mxGetField(PARAMS, 0, "c_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_20 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_20 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_20,10);

	}
	par = mxGetField(PARAMS, 0, "c_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_21 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_21 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_21,10);

	}
	par = mxGetField(PARAMS, 0, "c_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_22 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_22 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_22,10);

	}
	par = mxGetField(PARAMS, 0, "c_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_23 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_23 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_23,10);

	}
	par = mxGetField(PARAMS, 0, "c_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_24 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_24 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_24,10);

	}
	par = mxGetField(PARAMS, 0, "c_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_25 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_25 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_25,10);

	}
	par = mxGetField(PARAMS, 0, "c_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_26 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_26 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_26,10);

	}
	par = mxGetField(PARAMS, 0, "c_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_27 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_27 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_27,10);

	}
	par = mxGetField(PARAMS, 0, "c_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_28 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_28 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_28,10);

	}
	par = mxGetField(PARAMS, 0, "c_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_29 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_29 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_29,10);

	}
	par = mxGetField(PARAMS, 0, "c_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_30 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_30 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_30,10);

	}
	par = mxGetField(PARAMS, 0, "c_31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_31 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_31 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_31,10);

	}
	par = mxGetField(PARAMS, 0, "c_32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_32 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_32 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_32,10);

	}
	par = mxGetField(PARAMS, 0, "c_33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_33 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_33 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_33,10);

	}
	par = mxGetField(PARAMS, 0, "c_34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_34 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_34 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_34,10);

	}
	par = mxGetField(PARAMS, 0, "c_35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_35 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_35 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_35,10);

	}
	par = mxGetField(PARAMS, 0, "c_36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_36 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_36 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_36,10);

	}
	par = mxGetField(PARAMS, 0, "c_37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_37 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_37 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_37,10);

	}
	par = mxGetField(PARAMS, 0, "c_38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_38 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_38 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_38,10);

	}
	par = mxGetField(PARAMS, 0, "c_39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_39 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_39 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_39,10);

	}
	par = mxGetField(PARAMS, 0, "c_40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_40 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_40 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_40,10);

	}
	par = mxGetField(PARAMS, 0, "c_41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_41 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_41 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_41,10);

	}
	par = mxGetField(PARAMS, 0, "c_42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_42 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_42 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_42,10);

	}
	par = mxGetField(PARAMS, 0, "c_43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_43 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_43 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_43,10);

	}
	par = mxGetField(PARAMS, 0, "c_44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_44 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_44 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_44,10);

	}
	par = mxGetField(PARAMS, 0, "c_45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_45 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_45 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_45,10);

	}
	par = mxGetField(PARAMS, 0, "c_46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_46 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_46 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_46,10);

	}
	par = mxGetField(PARAMS, 0, "c_47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_47 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_47 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_47,10);

	}
	par = mxGetField(PARAMS, 0, "c_48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_48 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_48 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_48,10);

	}
	par = mxGetField(PARAMS, 0, "c_49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_49 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_49 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_49,10);

	}
	par = mxGetField(PARAMS, 0, "c_50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_50 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_50 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_50,10);

	}
	par = mxGetField(PARAMS, 0, "c_51");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_51 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_51 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_51 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_51,10);

	}
	par = mxGetField(PARAMS, 0, "c_52");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_52 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_52 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_52 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_52,10);

	}
	par = mxGetField(PARAMS, 0, "c_53");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_53 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_53 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_53 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_53,10);

	}
	par = mxGetField(PARAMS, 0, "c_54");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_54 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_54 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_54 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_54,10);

	}
	par = mxGetField(PARAMS, 0, "c_55");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_55 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_55 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_55 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_55,10);

	}
	par = mxGetField(PARAMS, 0, "c_56");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_56 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_56 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_56 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_56,10);

	}
	par = mxGetField(PARAMS, 0, "c_57");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_57 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_57 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_57 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_57,10);

	}
	par = mxGetField(PARAMS, 0, "c_58");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_58 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_58 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_58 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_58,10);

	}
	par = mxGetField(PARAMS, 0, "c_59");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_59 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_59 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_59 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_59,10);

	}
	par = mxGetField(PARAMS, 0, "c_60");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_60 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_60 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_60 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_60,10);

	}
	par = mxGetField(PARAMS, 0, "c_61");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_61 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_61 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_61 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_61,10);

	}
	par = mxGetField(PARAMS, 0, "c_62");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_62 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_62 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_62 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_62,10);

	}
	par = mxGetField(PARAMS, 0, "c_63");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_63 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_63 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_63 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_63,10);

	}
	par = mxGetField(PARAMS, 0, "c_64");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_64 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_64 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_64 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_64,10);

	}
	par = mxGetField(PARAMS, 0, "c_65");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_65 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_65 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_65 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_65,10);

	}
	par = mxGetField(PARAMS, 0, "c_66");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_66 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_66 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_66 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_66,10);

	}
	par = mxGetField(PARAMS, 0, "c_67");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_67 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_67 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_67 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_67,10);

	}
	par = mxGetField(PARAMS, 0, "c_68");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_68 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_68 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_68 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_68,10);

	}
	par = mxGetField(PARAMS, 0, "c_69");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_69 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_69 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_69 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_69,10);

	}
	par = mxGetField(PARAMS, 0, "c_70");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_70 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_70 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_70 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_70,10);

	}
	par = mxGetField(PARAMS, 0, "c_71");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_71 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_71 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_71 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_71,10);

	}
	par = mxGetField(PARAMS, 0, "c_72");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_72 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_72 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_72 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_72,10);

	}
	par = mxGetField(PARAMS, 0, "c_73");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_73 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_73 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_73 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_73,10);

	}
	par = mxGetField(PARAMS, 0, "c_74");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_74 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_74 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_74 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_74,10);

	}
	par = mxGetField(PARAMS, 0, "c_75");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_75 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_75 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_75 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_75,10);

	}
	par = mxGetField(PARAMS, 0, "c_76");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_76 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_76 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_76 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_76,10);

	}
	par = mxGetField(PARAMS, 0, "c_77");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_77 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_77 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_77 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_77,10);

	}
	par = mxGetField(PARAMS, 0, "c_78");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_78 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_78 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_78 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_78,10);

	}
	par = mxGetField(PARAMS, 0, "c_79");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_79 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_79 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_79 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_79,10);

	}
	par = mxGetField(PARAMS, 0, "c_80");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_80 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_80 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_80 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_80,10);

	}
	par = mxGetField(PARAMS, 0, "c_81");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c_81 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c_81 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_81 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_81,10);

	}
	par = mxGetField(PARAMS, 0, "H_1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_1 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_1 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_1,196);

	}
	par = mxGetField(PARAMS, 0, "H_2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_2 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_2 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_2,196);

	}
	par = mxGetField(PARAMS, 0, "H_3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_3 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_3 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_3,196);

	}
	par = mxGetField(PARAMS, 0, "H_4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_4 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_4 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_4,196);

	}
	par = mxGetField(PARAMS, 0, "H_5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_5 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_5 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_5,196);

	}
	par = mxGetField(PARAMS, 0, "H_6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_6 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_6 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_6,196);

	}
	par = mxGetField(PARAMS, 0, "H_7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_7 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_7 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_7,196);

	}
	par = mxGetField(PARAMS, 0, "H_8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_8 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_8 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_8,196);

	}
	par = mxGetField(PARAMS, 0, "H_9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_9 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_9 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_9,196);

	}
	par = mxGetField(PARAMS, 0, "H_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_10 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_10 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_10,196);

	}
	par = mxGetField(PARAMS, 0, "H_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_11 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_11 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_11,196);

	}
	par = mxGetField(PARAMS, 0, "H_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_12 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_12 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_12,196);

	}
	par = mxGetField(PARAMS, 0, "H_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_13 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_13 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_13,196);

	}
	par = mxGetField(PARAMS, 0, "H_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_14 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_14 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_14,196);

	}
	par = mxGetField(PARAMS, 0, "H_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_15 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_15 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_15,196);

	}
	par = mxGetField(PARAMS, 0, "H_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_16 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_16 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_16,196);

	}
	par = mxGetField(PARAMS, 0, "H_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_17 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_17 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_17,196);

	}
	par = mxGetField(PARAMS, 0, "H_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_18 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_18 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_18,196);

	}
	par = mxGetField(PARAMS, 0, "H_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_19 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_19 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_19,196);

	}
	par = mxGetField(PARAMS, 0, "H_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_20 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_20 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_20,196);

	}
	par = mxGetField(PARAMS, 0, "H_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_21 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_21 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_21,196);

	}
	par = mxGetField(PARAMS, 0, "H_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_22 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_22 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_22,196);

	}
	par = mxGetField(PARAMS, 0, "H_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_23 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_23 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_23,196);

	}
	par = mxGetField(PARAMS, 0, "H_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_24 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_24 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_24,196);

	}
	par = mxGetField(PARAMS, 0, "H_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_25 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_25 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_25,196);

	}
	par = mxGetField(PARAMS, 0, "H_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_26 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_26 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_26,196);

	}
	par = mxGetField(PARAMS, 0, "H_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_27 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_27 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_27,196);

	}
	par = mxGetField(PARAMS, 0, "H_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_28 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_28 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_28,196);

	}
	par = mxGetField(PARAMS, 0, "H_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_29 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_29 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_29,196);

	}
	par = mxGetField(PARAMS, 0, "H_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_30 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_30 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_30,196);

	}
	par = mxGetField(PARAMS, 0, "H_31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_31 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_31 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_31,196);

	}
	par = mxGetField(PARAMS, 0, "H_32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_32 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_32 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_32,196);

	}
	par = mxGetField(PARAMS, 0, "H_33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_33 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_33 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_33,196);

	}
	par = mxGetField(PARAMS, 0, "H_34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_34 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_34 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_34,196);

	}
	par = mxGetField(PARAMS, 0, "H_35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_35 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_35 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_35,196);

	}
	par = mxGetField(PARAMS, 0, "H_36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_36 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_36 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_36,196);

	}
	par = mxGetField(PARAMS, 0, "H_37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_37 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_37 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_37,196);

	}
	par = mxGetField(PARAMS, 0, "H_38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_38 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_38 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_38,196);

	}
	par = mxGetField(PARAMS, 0, "H_39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_39 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_39 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_39,196);

	}
	par = mxGetField(PARAMS, 0, "H_40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_40 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_40 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_40,196);

	}
	par = mxGetField(PARAMS, 0, "H_41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_41 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_41 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_41,196);

	}
	par = mxGetField(PARAMS, 0, "H_42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_42 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_42 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_42,196);

	}
	par = mxGetField(PARAMS, 0, "H_43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_43 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_43 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_43,196);

	}
	par = mxGetField(PARAMS, 0, "H_44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_44 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_44 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_44,196);

	}
	par = mxGetField(PARAMS, 0, "H_45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_45 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_45 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_45,196);

	}
	par = mxGetField(PARAMS, 0, "H_46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_46 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_46 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_46,196);

	}
	par = mxGetField(PARAMS, 0, "H_47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_47 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_47 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_47,196);

	}
	par = mxGetField(PARAMS, 0, "H_48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_48 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_48 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_48,196);

	}
	par = mxGetField(PARAMS, 0, "H_49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_49 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_49 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_49,196);

	}
	par = mxGetField(PARAMS, 0, "H_50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_50 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_50 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_50,196);

	}
	par = mxGetField(PARAMS, 0, "H_51");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_51 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_51 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_51 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_51,196);

	}
	par = mxGetField(PARAMS, 0, "H_52");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_52 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_52 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_52 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_52,196);

	}
	par = mxGetField(PARAMS, 0, "H_53");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_53 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_53 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_53 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_53,196);

	}
	par = mxGetField(PARAMS, 0, "H_54");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_54 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_54 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_54 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_54,196);

	}
	par = mxGetField(PARAMS, 0, "H_55");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_55 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_55 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_55 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_55,196);

	}
	par = mxGetField(PARAMS, 0, "H_56");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_56 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_56 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_56 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_56,196);

	}
	par = mxGetField(PARAMS, 0, "H_57");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_57 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_57 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_57 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_57,196);

	}
	par = mxGetField(PARAMS, 0, "H_58");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_58 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_58 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_58 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_58,196);

	}
	par = mxGetField(PARAMS, 0, "H_59");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_59 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_59 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_59 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_59,196);

	}
	par = mxGetField(PARAMS, 0, "H_60");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_60 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_60 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_60 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_60,196);

	}
	par = mxGetField(PARAMS, 0, "H_61");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_61 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_61 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_61 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_61,196);

	}
	par = mxGetField(PARAMS, 0, "H_62");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_62 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_62 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_62 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_62,196);

	}
	par = mxGetField(PARAMS, 0, "H_63");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_63 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_63 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_63 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_63,196);

	}
	par = mxGetField(PARAMS, 0, "H_64");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_64 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_64 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_64 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_64,196);

	}
	par = mxGetField(PARAMS, 0, "H_65");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_65 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_65 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_65 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_65,196);

	}
	par = mxGetField(PARAMS, 0, "H_66");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_66 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_66 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_66 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_66,196);

	}
	par = mxGetField(PARAMS, 0, "H_67");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_67 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_67 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_67 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_67,196);

	}
	par = mxGetField(PARAMS, 0, "H_68");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_68 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_68 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_68 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_68,196);

	}
	par = mxGetField(PARAMS, 0, "H_69");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_69 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_69 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_69 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_69,196);

	}
	par = mxGetField(PARAMS, 0, "H_70");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_70 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_70 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_70 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_70,196);

	}
	par = mxGetField(PARAMS, 0, "H_71");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_71 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_71 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_71 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_71,196);

	}
	par = mxGetField(PARAMS, 0, "H_72");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_72 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_72 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_72 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_72,196);

	}
	par = mxGetField(PARAMS, 0, "H_73");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_73 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_73 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_73 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_73,196);

	}
	par = mxGetField(PARAMS, 0, "H_74");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_74 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_74 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_74 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_74,196);

	}
	par = mxGetField(PARAMS, 0, "H_75");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_75 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_75 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_75 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_75,196);

	}
	par = mxGetField(PARAMS, 0, "H_76");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_76 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_76 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_76 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_76,196);

	}
	par = mxGetField(PARAMS, 0, "H_77");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_77 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_77 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_77 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_77,196);

	}
	par = mxGetField(PARAMS, 0, "H_78");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_78 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_78 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_78 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_78,196);

	}
	par = mxGetField(PARAMS, 0, "H_79");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_79 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_79 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_79 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_79,196);

	}
	par = mxGetField(PARAMS, 0, "H_80");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_80 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_80 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_80 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_80,196);

	}
	par = mxGetField(PARAMS, 0, "H_81");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H_81 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H_81 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H_81 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H_81,196);

	}
	par = mxGetField(PARAMS, 0, "f_1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_1 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_1 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_1,14);

	}
	par = mxGetField(PARAMS, 0, "f_2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_2 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_2 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_2,14);

	}
	par = mxGetField(PARAMS, 0, "f_3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_3 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_3 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_3,14);

	}
	par = mxGetField(PARAMS, 0, "f_4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_4 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_4 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_4,14);

	}
	par = mxGetField(PARAMS, 0, "f_5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_5 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_5 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_5,14);

	}
	par = mxGetField(PARAMS, 0, "f_6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_6 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_6 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_6,14);

	}
	par = mxGetField(PARAMS, 0, "f_7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_7 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_7 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_7,14);

	}
	par = mxGetField(PARAMS, 0, "f_8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_8 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_8 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_8,14);

	}
	par = mxGetField(PARAMS, 0, "f_9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_9 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_9 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_9,14);

	}
	par = mxGetField(PARAMS, 0, "f_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_10 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_10 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_10,14);

	}
	par = mxGetField(PARAMS, 0, "f_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_11 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_11 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_11,14);

	}
	par = mxGetField(PARAMS, 0, "f_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_12 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_12 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_12,14);

	}
	par = mxGetField(PARAMS, 0, "f_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_13 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_13 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_13,14);

	}
	par = mxGetField(PARAMS, 0, "f_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_14 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_14 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_14,14);

	}
	par = mxGetField(PARAMS, 0, "f_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_15 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_15 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_15,14);

	}
	par = mxGetField(PARAMS, 0, "f_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_16 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_16 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_16,14);

	}
	par = mxGetField(PARAMS, 0, "f_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_17 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_17 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_17,14);

	}
	par = mxGetField(PARAMS, 0, "f_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_18 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_18 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_18,14);

	}
	par = mxGetField(PARAMS, 0, "f_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_19 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_19 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_19,14);

	}
	par = mxGetField(PARAMS, 0, "f_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_20 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_20 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_20,14);

	}
	par = mxGetField(PARAMS, 0, "f_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_21 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_21 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_21,14);

	}
	par = mxGetField(PARAMS, 0, "f_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_22 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_22 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_22,14);

	}
	par = mxGetField(PARAMS, 0, "f_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_23 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_23 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_23,14);

	}
	par = mxGetField(PARAMS, 0, "f_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_24 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_24 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_24,14);

	}
	par = mxGetField(PARAMS, 0, "f_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_25 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_25 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_25,14);

	}
	par = mxGetField(PARAMS, 0, "f_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_26 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_26 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_26,14);

	}
	par = mxGetField(PARAMS, 0, "f_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_27 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_27 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_27,14);

	}
	par = mxGetField(PARAMS, 0, "f_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_28 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_28 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_28,14);

	}
	par = mxGetField(PARAMS, 0, "f_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_29 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_29 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_29,14);

	}
	par = mxGetField(PARAMS, 0, "f_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_30 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_30 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_30,14);

	}
	par = mxGetField(PARAMS, 0, "f_31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_31 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_31 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_31,14);

	}
	par = mxGetField(PARAMS, 0, "f_32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_32 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_32 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_32,14);

	}
	par = mxGetField(PARAMS, 0, "f_33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_33 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_33 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_33,14);

	}
	par = mxGetField(PARAMS, 0, "f_34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_34 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_34 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_34,14);

	}
	par = mxGetField(PARAMS, 0, "f_35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_35 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_35 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_35,14);

	}
	par = mxGetField(PARAMS, 0, "f_36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_36 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_36 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_36,14);

	}
	par = mxGetField(PARAMS, 0, "f_37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_37 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_37 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_37,14);

	}
	par = mxGetField(PARAMS, 0, "f_38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_38 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_38 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_38,14);

	}
	par = mxGetField(PARAMS, 0, "f_39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_39 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_39 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_39,14);

	}
	par = mxGetField(PARAMS, 0, "f_40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_40 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_40 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_40,14);

	}
	par = mxGetField(PARAMS, 0, "f_41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_41 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_41 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_41,14);

	}
	par = mxGetField(PARAMS, 0, "f_42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_42 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_42 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_42,14);

	}
	par = mxGetField(PARAMS, 0, "f_43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_43 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_43 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_43,14);

	}
	par = mxGetField(PARAMS, 0, "f_44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_44 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_44 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_44,14);

	}
	par = mxGetField(PARAMS, 0, "f_45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_45 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_45 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_45,14);

	}
	par = mxGetField(PARAMS, 0, "f_46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_46 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_46 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_46,14);

	}
	par = mxGetField(PARAMS, 0, "f_47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_47 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_47 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_47,14);

	}
	par = mxGetField(PARAMS, 0, "f_48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_48 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_48 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_48,14);

	}
	par = mxGetField(PARAMS, 0, "f_49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_49 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_49 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_49,14);

	}
	par = mxGetField(PARAMS, 0, "f_50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_50 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_50 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_50,14);

	}
	par = mxGetField(PARAMS, 0, "f_51");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_51 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_51 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_51 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_51,14);

	}
	par = mxGetField(PARAMS, 0, "f_52");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_52 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_52 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_52 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_52,14);

	}
	par = mxGetField(PARAMS, 0, "f_53");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_53 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_53 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_53 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_53,14);

	}
	par = mxGetField(PARAMS, 0, "f_54");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_54 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_54 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_54 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_54,14);

	}
	par = mxGetField(PARAMS, 0, "f_55");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_55 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_55 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_55 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_55,14);

	}
	par = mxGetField(PARAMS, 0, "f_56");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_56 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_56 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_56 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_56,14);

	}
	par = mxGetField(PARAMS, 0, "f_57");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_57 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_57 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_57 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_57,14);

	}
	par = mxGetField(PARAMS, 0, "f_58");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_58 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_58 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_58 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_58,14);

	}
	par = mxGetField(PARAMS, 0, "f_59");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_59 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_59 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_59 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_59,14);

	}
	par = mxGetField(PARAMS, 0, "f_60");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_60 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_60 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_60 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_60,14);

	}
	par = mxGetField(PARAMS, 0, "f_61");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_61 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_61 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_61 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_61,14);

	}
	par = mxGetField(PARAMS, 0, "f_62");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_62 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_62 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_62 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_62,14);

	}
	par = mxGetField(PARAMS, 0, "f_63");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_63 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_63 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_63 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_63,14);

	}
	par = mxGetField(PARAMS, 0, "f_64");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_64 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_64 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_64 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_64,14);

	}
	par = mxGetField(PARAMS, 0, "f_65");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_65 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_65 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_65 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_65,14);

	}
	par = mxGetField(PARAMS, 0, "f_66");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_66 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_66 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_66 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_66,14);

	}
	par = mxGetField(PARAMS, 0, "f_67");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_67 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_67 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_67 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_67,14);

	}
	par = mxGetField(PARAMS, 0, "f_68");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_68 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_68 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_68 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_68,14);

	}
	par = mxGetField(PARAMS, 0, "f_69");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_69 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_69 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_69 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_69,14);

	}
	par = mxGetField(PARAMS, 0, "f_70");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_70 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_70 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_70 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_70,14);

	}
	par = mxGetField(PARAMS, 0, "f_71");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_71 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_71 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_71 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_71,14);

	}
	par = mxGetField(PARAMS, 0, "f_72");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_72 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_72 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_72 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_72,14);

	}
	par = mxGetField(PARAMS, 0, "f_73");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_73 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_73 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_73 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_73,14);

	}
	par = mxGetField(PARAMS, 0, "f_74");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_74 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_74 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_74 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_74,14);

	}
	par = mxGetField(PARAMS, 0, "f_75");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_75 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_75 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_75 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_75,14);

	}
	par = mxGetField(PARAMS, 0, "f_76");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_76 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_76 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_76 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_76,14);

	}
	par = mxGetField(PARAMS, 0, "f_77");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_77 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_77 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_77 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_77,14);

	}
	par = mxGetField(PARAMS, 0, "f_78");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_78 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_78 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_78 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_78,14);

	}
	par = mxGetField(PARAMS, 0, "f_79");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_79 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_79 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_79 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_79,14);

	}
	par = mxGetField(PARAMS, 0, "f_80");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_80 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_80 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_80 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_80,14);

	}
	par = mxGetField(PARAMS, 0, "f_81");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f_81 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f_81 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f_81 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f_81,14);

	}
	par = mxGetField(PARAMS, 0, "C_1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_1 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_1 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_1,140);

	}
	par = mxGetField(PARAMS, 0, "C_2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_2 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_2 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_2,140);

	}
	par = mxGetField(PARAMS, 0, "C_3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_3 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_3 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_3,140);

	}
	par = mxGetField(PARAMS, 0, "C_4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_4 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_4 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_4,140);

	}
	par = mxGetField(PARAMS, 0, "C_5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_5 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_5 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_5,140);

	}
	par = mxGetField(PARAMS, 0, "C_6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_6 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_6 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_6,140);

	}
	par = mxGetField(PARAMS, 0, "C_7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_7 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_7 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_7,140);

	}
	par = mxGetField(PARAMS, 0, "C_8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_8 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_8 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_8,140);

	}
	par = mxGetField(PARAMS, 0, "C_9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_9 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_9 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_9,140);

	}
	par = mxGetField(PARAMS, 0, "C_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_10 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_10 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_10,140);

	}
	par = mxGetField(PARAMS, 0, "C_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_11 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_11 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_11,140);

	}
	par = mxGetField(PARAMS, 0, "C_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_12 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_12 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_12,140);

	}
	par = mxGetField(PARAMS, 0, "C_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_13 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_13 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_13,140);

	}
	par = mxGetField(PARAMS, 0, "C_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_14 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_14 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_14,140);

	}
	par = mxGetField(PARAMS, 0, "C_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_15 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_15 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_15,140);

	}
	par = mxGetField(PARAMS, 0, "C_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_16 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_16 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_16,140);

	}
	par = mxGetField(PARAMS, 0, "C_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_17 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_17 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_17,140);

	}
	par = mxGetField(PARAMS, 0, "C_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_18 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_18 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_18,140);

	}
	par = mxGetField(PARAMS, 0, "C_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_19 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_19 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_19,140);

	}
	par = mxGetField(PARAMS, 0, "C_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_20 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_20 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_20,140);

	}
	par = mxGetField(PARAMS, 0, "C_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_21 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_21 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_21,140);

	}
	par = mxGetField(PARAMS, 0, "C_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_22 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_22 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_22,140);

	}
	par = mxGetField(PARAMS, 0, "C_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_23 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_23 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_23,140);

	}
	par = mxGetField(PARAMS, 0, "C_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_24 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_24 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_24,140);

	}
	par = mxGetField(PARAMS, 0, "C_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_25 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_25 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_25,140);

	}
	par = mxGetField(PARAMS, 0, "C_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_26 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_26 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_26,140);

	}
	par = mxGetField(PARAMS, 0, "C_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_27 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_27 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_27,140);

	}
	par = mxGetField(PARAMS, 0, "C_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_28 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_28 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_28,140);

	}
	par = mxGetField(PARAMS, 0, "C_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_29 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_29 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_29,140);

	}
	par = mxGetField(PARAMS, 0, "C_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_30 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_30 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_30,140);

	}
	par = mxGetField(PARAMS, 0, "C_31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_31 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_31 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_31,140);

	}
	par = mxGetField(PARAMS, 0, "C_32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_32 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_32 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_32,140);

	}
	par = mxGetField(PARAMS, 0, "C_33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_33 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_33 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_33,140);

	}
	par = mxGetField(PARAMS, 0, "C_34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_34 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_34 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_34,140);

	}
	par = mxGetField(PARAMS, 0, "C_35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_35 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_35 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_35,140);

	}
	par = mxGetField(PARAMS, 0, "C_36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_36 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_36 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_36,140);

	}
	par = mxGetField(PARAMS, 0, "C_37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_37 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_37 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_37,140);

	}
	par = mxGetField(PARAMS, 0, "C_38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_38 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_38 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_38,140);

	}
	par = mxGetField(PARAMS, 0, "C_39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_39 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_39 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_39,140);

	}
	par = mxGetField(PARAMS, 0, "C_40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_40 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_40 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_40,140);

	}
	par = mxGetField(PARAMS, 0, "C_41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_41 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_41 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_41,140);

	}
	par = mxGetField(PARAMS, 0, "C_42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_42 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_42 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_42,140);

	}
	par = mxGetField(PARAMS, 0, "C_43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_43 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_43 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_43,140);

	}
	par = mxGetField(PARAMS, 0, "C_44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_44 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_44 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_44,140);

	}
	par = mxGetField(PARAMS, 0, "C_45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_45 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_45 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_45,140);

	}
	par = mxGetField(PARAMS, 0, "C_46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_46 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_46 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_46,140);

	}
	par = mxGetField(PARAMS, 0, "C_47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_47 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_47 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_47,140);

	}
	par = mxGetField(PARAMS, 0, "C_48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_48 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_48 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_48,140);

	}
	par = mxGetField(PARAMS, 0, "C_49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_49 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_49 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_49,140);

	}
	par = mxGetField(PARAMS, 0, "C_50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_50 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_50 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_50,140);

	}
	par = mxGetField(PARAMS, 0, "C_51");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_51 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_51 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_51 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_51,140);

	}
	par = mxGetField(PARAMS, 0, "C_52");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_52 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_52 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_52 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_52,140);

	}
	par = mxGetField(PARAMS, 0, "C_53");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_53 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_53 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_53 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_53,140);

	}
	par = mxGetField(PARAMS, 0, "C_54");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_54 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_54 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_54 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_54,140);

	}
	par = mxGetField(PARAMS, 0, "C_55");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_55 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_55 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_55 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_55,140);

	}
	par = mxGetField(PARAMS, 0, "C_56");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_56 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_56 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_56 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_56,140);

	}
	par = mxGetField(PARAMS, 0, "C_57");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_57 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_57 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_57 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_57,140);

	}
	par = mxGetField(PARAMS, 0, "C_58");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_58 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_58 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_58 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_58,140);

	}
	par = mxGetField(PARAMS, 0, "C_59");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_59 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_59 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_59 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_59,140);

	}
	par = mxGetField(PARAMS, 0, "C_60");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_60 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_60 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_60 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_60,140);

	}
	par = mxGetField(PARAMS, 0, "C_61");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_61 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_61 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_61 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_61,140);

	}
	par = mxGetField(PARAMS, 0, "C_62");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_62 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_62 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_62 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_62,140);

	}
	par = mxGetField(PARAMS, 0, "C_63");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_63 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_63 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_63 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_63,140);

	}
	par = mxGetField(PARAMS, 0, "C_64");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_64 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_64 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_64 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_64,140);

	}
	par = mxGetField(PARAMS, 0, "C_65");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_65 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_65 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_65 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_65,140);

	}
	par = mxGetField(PARAMS, 0, "C_66");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_66 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_66 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_66 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_66,140);

	}
	par = mxGetField(PARAMS, 0, "C_67");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_67 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_67 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_67 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_67,140);

	}
	par = mxGetField(PARAMS, 0, "C_68");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_68 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_68 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_68 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_68,140);

	}
	par = mxGetField(PARAMS, 0, "C_69");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_69 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_69 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_69 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_69,140);

	}
	par = mxGetField(PARAMS, 0, "C_70");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_70 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_70 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_70 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_70,140);

	}
	par = mxGetField(PARAMS, 0, "C_71");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_71 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_71 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_71 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_71,140);

	}
	par = mxGetField(PARAMS, 0, "C_72");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_72 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_72 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_72 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_72,140);

	}
	par = mxGetField(PARAMS, 0, "C_73");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_73 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_73 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_73 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_73,140);

	}
	par = mxGetField(PARAMS, 0, "C_74");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_74 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_74 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_74 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_74,140);

	}
	par = mxGetField(PARAMS, 0, "C_75");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_75 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_75 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_75 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_75,140);

	}
	par = mxGetField(PARAMS, 0, "C_76");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_76 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_76 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_76 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_76,140);

	}
	par = mxGetField(PARAMS, 0, "C_77");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_77 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_77 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_77 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_77,140);

	}
	par = mxGetField(PARAMS, 0, "C_78");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_78 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_78 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_78 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_78,140);

	}
	par = mxGetField(PARAMS, 0, "C_79");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_79 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_79 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_79 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_79,140);

	}
	par = mxGetField(PARAMS, 0, "C_80");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C_80 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C_80 must be a double.");
    }
    if( mxGetM(par) != 10 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C_80 must be of size [10 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C_80,140);

	}
	par = mxGetField(PARAMS, 0, "A_2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_2 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_2 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_2,28);

	}
	par = mxGetField(PARAMS, 0, "A_3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_3 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_3 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_3,28);

	}
	par = mxGetField(PARAMS, 0, "A_4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_4 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_4 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_4,28);

	}
	par = mxGetField(PARAMS, 0, "A_5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_5 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_5 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_5,28);

	}
	par = mxGetField(PARAMS, 0, "A_6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_6 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_6 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_6,28);

	}
	par = mxGetField(PARAMS, 0, "A_7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_7 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_7 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_7,28);

	}
	par = mxGetField(PARAMS, 0, "A_8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_8 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_8 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_8,28);

	}
	par = mxGetField(PARAMS, 0, "A_9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_9 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_9 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_9,28);

	}
	par = mxGetField(PARAMS, 0, "A_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_10 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_10 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_10,28);

	}
	par = mxGetField(PARAMS, 0, "A_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_11 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_11 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_11,28);

	}
	par = mxGetField(PARAMS, 0, "A_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_12 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_12 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_12,28);

	}
	par = mxGetField(PARAMS, 0, "A_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_13 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_13 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_13,28);

	}
	par = mxGetField(PARAMS, 0, "A_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_14 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_14 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_14,28);

	}
	par = mxGetField(PARAMS, 0, "A_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_15 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_15 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_15,28);

	}
	par = mxGetField(PARAMS, 0, "A_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_16 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_16 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_16,28);

	}
	par = mxGetField(PARAMS, 0, "A_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_17 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_17 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_17,28);

	}
	par = mxGetField(PARAMS, 0, "A_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_18 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_18 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_18,28);

	}
	par = mxGetField(PARAMS, 0, "A_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_19 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_19 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_19,28);

	}
	par = mxGetField(PARAMS, 0, "A_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_20 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_20 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_20,28);

	}
	par = mxGetField(PARAMS, 0, "A_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_21 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_21 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_21,28);

	}
	par = mxGetField(PARAMS, 0, "A_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_22 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_22 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_22,28);

	}
	par = mxGetField(PARAMS, 0, "A_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_23 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_23 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_23,28);

	}
	par = mxGetField(PARAMS, 0, "A_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_24 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_24 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_24,28);

	}
	par = mxGetField(PARAMS, 0, "A_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_25 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_25 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_25,28);

	}
	par = mxGetField(PARAMS, 0, "A_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_26 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_26 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_26,28);

	}
	par = mxGetField(PARAMS, 0, "A_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_27 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_27 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_27,28);

	}
	par = mxGetField(PARAMS, 0, "A_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_28 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_28 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_28,28);

	}
	par = mxGetField(PARAMS, 0, "A_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_29 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_29 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_29,28);

	}
	par = mxGetField(PARAMS, 0, "A_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_30 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_30 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_30,28);

	}
	par = mxGetField(PARAMS, 0, "A_31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_31 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_31 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_31,28);

	}
	par = mxGetField(PARAMS, 0, "A_32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_32 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_32 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_32,28);

	}
	par = mxGetField(PARAMS, 0, "A_33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_33 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_33 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_33,28);

	}
	par = mxGetField(PARAMS, 0, "A_34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_34 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_34 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_34,28);

	}
	par = mxGetField(PARAMS, 0, "A_35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_35 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_35 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_35,28);

	}
	par = mxGetField(PARAMS, 0, "A_36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_36 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_36 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_36,28);

	}
	par = mxGetField(PARAMS, 0, "A_37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_37 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_37 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_37,28);

	}
	par = mxGetField(PARAMS, 0, "A_38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_38 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_38 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_38,28);

	}
	par = mxGetField(PARAMS, 0, "A_39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_39 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_39 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_39,28);

	}
	par = mxGetField(PARAMS, 0, "A_40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_40 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_40 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_40,28);

	}
	par = mxGetField(PARAMS, 0, "A_41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_41 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_41 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_41,28);

	}
	par = mxGetField(PARAMS, 0, "A_42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_42 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_42 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_42,28);

	}
	par = mxGetField(PARAMS, 0, "A_43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_43 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_43 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_43,28);

	}
	par = mxGetField(PARAMS, 0, "A_44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_44 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_44 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_44,28);

	}
	par = mxGetField(PARAMS, 0, "A_45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_45 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_45 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_45,28);

	}
	par = mxGetField(PARAMS, 0, "A_46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_46 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_46 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_46,28);

	}
	par = mxGetField(PARAMS, 0, "A_47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_47 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_47 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_47,28);

	}
	par = mxGetField(PARAMS, 0, "A_48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_48 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_48 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_48,28);

	}
	par = mxGetField(PARAMS, 0, "A_49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_49 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_49 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_49,28);

	}
	par = mxGetField(PARAMS, 0, "A_50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_50 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_50 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_50,28);

	}
	par = mxGetField(PARAMS, 0, "A_51");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_51 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_51 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_51 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_51,28);

	}
	par = mxGetField(PARAMS, 0, "A_52");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_52 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_52 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_52 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_52,28);

	}
	par = mxGetField(PARAMS, 0, "A_53");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_53 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_53 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_53 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_53,28);

	}
	par = mxGetField(PARAMS, 0, "A_54");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_54 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_54 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_54 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_54,28);

	}
	par = mxGetField(PARAMS, 0, "A_55");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_55 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_55 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_55 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_55,28);

	}
	par = mxGetField(PARAMS, 0, "A_56");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_56 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_56 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_56 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_56,28);

	}
	par = mxGetField(PARAMS, 0, "A_57");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_57 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_57 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_57 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_57,28);

	}
	par = mxGetField(PARAMS, 0, "A_58");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_58 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_58 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_58 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_58,28);

	}
	par = mxGetField(PARAMS, 0, "A_59");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_59 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_59 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_59 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_59,28);

	}
	par = mxGetField(PARAMS, 0, "A_60");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_60 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_60 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_60 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_60,28);

	}
	par = mxGetField(PARAMS, 0, "A_61");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_61 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_61 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_61 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_61,28);

	}
	par = mxGetField(PARAMS, 0, "A_62");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_62 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_62 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_62 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_62,28);

	}
	par = mxGetField(PARAMS, 0, "A_63");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_63 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_63 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_63 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_63,28);

	}
	par = mxGetField(PARAMS, 0, "A_64");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_64 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_64 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_64 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_64,28);

	}
	par = mxGetField(PARAMS, 0, "A_65");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_65 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_65 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_65 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_65,28);

	}
	par = mxGetField(PARAMS, 0, "A_66");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_66 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_66 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_66 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_66,28);

	}
	par = mxGetField(PARAMS, 0, "A_67");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_67 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_67 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_67 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_67,28);

	}
	par = mxGetField(PARAMS, 0, "A_68");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_68 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_68 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_68 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_68,28);

	}
	par = mxGetField(PARAMS, 0, "A_69");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_69 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_69 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_69 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_69,28);

	}
	par = mxGetField(PARAMS, 0, "A_70");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_70 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_70 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_70 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_70,28);

	}
	par = mxGetField(PARAMS, 0, "A_71");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_71 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_71 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_71 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_71,28);

	}
	par = mxGetField(PARAMS, 0, "A_72");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_72 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_72 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_72 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_72,28);

	}
	par = mxGetField(PARAMS, 0, "A_73");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_73 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_73 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_73 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_73,28);

	}
	par = mxGetField(PARAMS, 0, "A_74");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_74 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_74 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_74 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_74,28);

	}
	par = mxGetField(PARAMS, 0, "A_75");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_75 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_75 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_75 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_75,28);

	}
	par = mxGetField(PARAMS, 0, "A_76");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_76 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_76 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_76 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_76,28);

	}
	par = mxGetField(PARAMS, 0, "A_77");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_77 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_77 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_77 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_77,28);

	}
	par = mxGetField(PARAMS, 0, "A_78");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_78 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_78 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_78 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_78,28);

	}
	par = mxGetField(PARAMS, 0, "A_79");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_79 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_79 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_79 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_79,28);

	}
	par = mxGetField(PARAMS, 0, "A_80");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_80 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_80 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_80 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_80,28);

	}
	par = mxGetField(PARAMS, 0, "A_81");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A_81 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A_81 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A_81 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A_81,28);

	}
	par = mxGetField(PARAMS, 0, "b_2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_2 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_2 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_2,2);

	}
	par = mxGetField(PARAMS, 0, "b_3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_3 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_3 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_3,2);

	}
	par = mxGetField(PARAMS, 0, "b_4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_4 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_4 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_4,2);

	}
	par = mxGetField(PARAMS, 0, "b_5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_5 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_5 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_5,2);

	}
	par = mxGetField(PARAMS, 0, "b_6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_6 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_6 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_6,2);

	}
	par = mxGetField(PARAMS, 0, "b_7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_7 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_7 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_7,2);

	}
	par = mxGetField(PARAMS, 0, "b_8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_8 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_8 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_8,2);

	}
	par = mxGetField(PARAMS, 0, "b_9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_9 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_9 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_9,2);

	}
	par = mxGetField(PARAMS, 0, "b_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_10 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_10 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_10,2);

	}
	par = mxGetField(PARAMS, 0, "b_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_11 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_11 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_11,2);

	}
	par = mxGetField(PARAMS, 0, "b_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_12 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_12 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_12,2);

	}
	par = mxGetField(PARAMS, 0, "b_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_13 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_13 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_13,2);

	}
	par = mxGetField(PARAMS, 0, "b_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_14 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_14 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_14,2);

	}
	par = mxGetField(PARAMS, 0, "b_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_15 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_15 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_15,2);

	}
	par = mxGetField(PARAMS, 0, "b_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_16 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_16 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_16,2);

	}
	par = mxGetField(PARAMS, 0, "b_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_17 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_17 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_17,2);

	}
	par = mxGetField(PARAMS, 0, "b_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_18 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_18 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_18,2);

	}
	par = mxGetField(PARAMS, 0, "b_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_19 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_19 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_19,2);

	}
	par = mxGetField(PARAMS, 0, "b_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_20 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_20 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_20,2);

	}
	par = mxGetField(PARAMS, 0, "b_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_21 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_21 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_21,2);

	}
	par = mxGetField(PARAMS, 0, "b_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_22 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_22 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_22,2);

	}
	par = mxGetField(PARAMS, 0, "b_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_23 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_23 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_23,2);

	}
	par = mxGetField(PARAMS, 0, "b_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_24 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_24 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_24,2);

	}
	par = mxGetField(PARAMS, 0, "b_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_25 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_25 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_25,2);

	}
	par = mxGetField(PARAMS, 0, "b_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_26 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_26 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_26,2);

	}
	par = mxGetField(PARAMS, 0, "b_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_27 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_27 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_27,2);

	}
	par = mxGetField(PARAMS, 0, "b_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_28 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_28 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_28,2);

	}
	par = mxGetField(PARAMS, 0, "b_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_29 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_29 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_29,2);

	}
	par = mxGetField(PARAMS, 0, "b_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_30 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_30 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_30,2);

	}
	par = mxGetField(PARAMS, 0, "b_31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_31 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_31 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_31,2);

	}
	par = mxGetField(PARAMS, 0, "b_32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_32 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_32 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_32,2);

	}
	par = mxGetField(PARAMS, 0, "b_33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_33 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_33 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_33,2);

	}
	par = mxGetField(PARAMS, 0, "b_34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_34 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_34 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_34,2);

	}
	par = mxGetField(PARAMS, 0, "b_35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_35 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_35 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_35,2);

	}
	par = mxGetField(PARAMS, 0, "b_36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_36 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_36 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_36,2);

	}
	par = mxGetField(PARAMS, 0, "b_37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_37 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_37 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_37,2);

	}
	par = mxGetField(PARAMS, 0, "b_38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_38 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_38 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_38,2);

	}
	par = mxGetField(PARAMS, 0, "b_39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_39 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_39 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_39,2);

	}
	par = mxGetField(PARAMS, 0, "b_40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_40 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_40 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_40,2);

	}
	par = mxGetField(PARAMS, 0, "b_41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_41 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_41 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_41,2);

	}
	par = mxGetField(PARAMS, 0, "b_42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_42 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_42 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_42,2);

	}
	par = mxGetField(PARAMS, 0, "b_43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_43 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_43 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_43,2);

	}
	par = mxGetField(PARAMS, 0, "b_44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_44 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_44 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_44,2);

	}
	par = mxGetField(PARAMS, 0, "b_45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_45 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_45 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_45,2);

	}
	par = mxGetField(PARAMS, 0, "b_46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_46 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_46 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_46,2);

	}
	par = mxGetField(PARAMS, 0, "b_47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_47 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_47 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_47,2);

	}
	par = mxGetField(PARAMS, 0, "b_48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_48 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_48 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_48,2);

	}
	par = mxGetField(PARAMS, 0, "b_49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_49 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_49 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_49,2);

	}
	par = mxGetField(PARAMS, 0, "b_50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_50 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_50 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_50,2);

	}
	par = mxGetField(PARAMS, 0, "b_51");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_51 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_51 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_51 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_51,2);

	}
	par = mxGetField(PARAMS, 0, "b_52");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_52 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_52 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_52 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_52,2);

	}
	par = mxGetField(PARAMS, 0, "b_53");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_53 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_53 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_53 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_53,2);

	}
	par = mxGetField(PARAMS, 0, "b_54");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_54 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_54 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_54 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_54,2);

	}
	par = mxGetField(PARAMS, 0, "b_55");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_55 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_55 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_55 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_55,2);

	}
	par = mxGetField(PARAMS, 0, "b_56");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_56 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_56 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_56 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_56,2);

	}
	par = mxGetField(PARAMS, 0, "b_57");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_57 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_57 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_57 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_57,2);

	}
	par = mxGetField(PARAMS, 0, "b_58");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_58 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_58 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_58 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_58,2);

	}
	par = mxGetField(PARAMS, 0, "b_59");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_59 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_59 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_59 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_59,2);

	}
	par = mxGetField(PARAMS, 0, "b_60");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_60 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_60 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_60 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_60,2);

	}
	par = mxGetField(PARAMS, 0, "b_61");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_61 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_61 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_61 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_61,2);

	}
	par = mxGetField(PARAMS, 0, "b_62");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_62 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_62 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_62 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_62,2);

	}
	par = mxGetField(PARAMS, 0, "b_63");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_63 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_63 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_63 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_63,2);

	}
	par = mxGetField(PARAMS, 0, "b_64");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_64 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_64 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_64 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_64,2);

	}
	par = mxGetField(PARAMS, 0, "b_65");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_65 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_65 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_65 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_65,2);

	}
	par = mxGetField(PARAMS, 0, "b_66");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_66 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_66 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_66 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_66,2);

	}
	par = mxGetField(PARAMS, 0, "b_67");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_67 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_67 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_67 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_67,2);

	}
	par = mxGetField(PARAMS, 0, "b_68");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_68 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_68 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_68 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_68,2);

	}
	par = mxGetField(PARAMS, 0, "b_69");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_69 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_69 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_69 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_69,2);

	}
	par = mxGetField(PARAMS, 0, "b_70");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_70 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_70 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_70 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_70,2);

	}
	par = mxGetField(PARAMS, 0, "b_71");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_71 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_71 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_71 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_71,2);

	}
	par = mxGetField(PARAMS, 0, "b_72");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_72 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_72 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_72 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_72,2);

	}
	par = mxGetField(PARAMS, 0, "b_73");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_73 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_73 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_73 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_73,2);

	}
	par = mxGetField(PARAMS, 0, "b_74");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_74 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_74 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_74 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_74,2);

	}
	par = mxGetField(PARAMS, 0, "b_75");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_75 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_75 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_75 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_75,2);

	}
	par = mxGetField(PARAMS, 0, "b_76");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_76 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_76 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_76 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_76,2);

	}
	par = mxGetField(PARAMS, 0, "b_77");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_77 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_77 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_77 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_77,2);

	}
	par = mxGetField(PARAMS, 0, "b_78");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_78 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_78 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_78 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_78,2);

	}
	par = mxGetField(PARAMS, 0, "b_79");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_79 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_79 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_79 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_79,2);

	}
	par = mxGetField(PARAMS, 0, "b_80");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_80 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_80 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_80 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_80,2);

	}
	par = mxGetField(PARAMS, 0, "b_81");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b_81 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b_81 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b_81 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b_81,2);

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

	/* call solver */
	exitflag = MPCC_solv_80N_no_warm_no_hard_invitedguest_solve(&params, &output, &info, fp);
	
	#if SET_PRINTLEVEL_MPCC_solv_80N_no_warm_no_hard_invitedguest > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 1, outputnames);
		outvar = mxCreateDoubleMatrix(972, 1, mxREAL);
	copyCArrayToM_double( output.X, mxGetPr(outvar), 972);
	mxSetField(plhs[0], 0, "X", outvar);



	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 16, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dobj;
		mxSetField(plhs[2], 0, "dobj", outvar);

		/* dgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dgap;
		mxSetField(plhs[2], 0, "dgap", outvar);

		/* rdgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rdgap;
		mxSetField(plhs[2], 0, "rdgap", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* mu_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu_aff;
		mxSetField(plhs[2], 0, "mu_aff", outvar);

		/* sigma */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.sigma;
		mxSetField(plhs[2], 0, "sigma", outvar);

		/* lsit_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_aff;
		mxSetField(plhs[2], 0, "lsit_aff", outvar);

		/* lsit_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_cc;
		mxSetField(plhs[2], 0, "lsit_cc", outvar);

		/* step_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_aff;
		mxSetField(plhs[2], 0, "step_aff", outvar);

		/* step_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_cc;
		mxSetField(plhs[2], 0, "step_cc", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

	}
}

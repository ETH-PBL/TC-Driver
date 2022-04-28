/*
MPCC_solv_20N_no_warm_hard_invitedguest : A fast customized optimization solver.

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
#include "../include/MPCC_solv_20N_no_warm_hard_invitedguest.h"
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

/* copy functions */

void copyCArrayToM_solver_int32_unsigned(solver_int32_unsigned *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_solver_int32_unsigned(double *src, solver_int32_unsigned *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (solver_int32_unsigned) (*src++) ;
    }
}

void copyMValueToC_solver_int32_unsigned(double * src, solver_int32_unsigned * dest)
{
	*dest = (solver_int32_unsigned) *src;
}





/* Some memory for mex-function */
static MPCC_solv_20N_no_warm_hard_invitedguest_params params;
static MPCC_solv_20N_no_warm_hard_invitedguest_output output;
static MPCC_solv_20N_no_warm_hard_invitedguest_info info;

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
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help MPCC_solv_20N_no_warm_hard_invitedguest_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help MPCC_solv_20N_no_warm_hard_invitedguest_mex' for details.");
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
    if( mxGetM(par) != 10 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c_1 must be of size [10 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c_1,10);

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
	par = mxGetField(PARAMS, 0, "num_of_threads");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_solver_int32_unsigned(mxGetPr(par), &params.num_of_threads);

	}




	#if SET_PRINTLEVEL_MPCC_solv_20N_no_warm_hard_invitedguest > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */
	exitflag = MPCC_solv_20N_no_warm_hard_invitedguest_solve(&params, &output, &info, fp);
	
	#if SET_PRINTLEVEL_MPCC_solv_20N_no_warm_hard_invitedguest > 0
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
		outvar = mxCreateDoubleMatrix(252, 1, mxREAL);
	copyCArrayToM_double( output.X, mxGetPr(outvar), 252);
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

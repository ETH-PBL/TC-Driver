/*
MPCC_solv_20N_no_warm_no_hard_gnone : A fast customized optimization solver.

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

/* Generated by FORCESPRO v5.1.0 on Monday, April 25, 2022 at 3:33:13 PM */
#ifndef MPCC_solv_20N_no_warm_no_hard_gnone_H
#define MPCC_solv_20N_no_warm_no_hard_gnone_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif


#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double MPCC_solv_20N_no_warm_no_hard_gnone_float;


typedef double MPCC_solv_20N_no_warm_no_hard_gnoneinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_MPCC_solv_20N_no_warm_no_hard_gnone
#define MISRA_C_MPCC_solv_20N_no_warm_no_hard_gnone (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_MPCC_solv_20N_no_warm_no_hard_gnone
#define RESTRICT_CODE_MPCC_solv_20N_no_warm_no_hard_gnone (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_MPCC_solv_20N_no_warm_no_hard_gnone
#define SET_PRINTLEVEL_MPCC_solv_20N_no_warm_no_hard_gnone    (2)
#endif

/* timing */
#ifndef SET_TIMING_MPCC_solv_20N_no_warm_no_hard_gnone
#define SET_TIMING_MPCC_solv_20N_no_warm_no_hard_gnone    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_MPCC_solv_20N_no_warm_no_hard_gnone         (200)	

/* scaling factor of line search (affine direction) */
#define SET_LS_SCALE_AFF_MPCC_solv_20N_no_warm_no_hard_gnone  (MPCC_solv_20N_no_warm_no_hard_gnone_float)(0.9)      

/* scaling factor of line search (combined direction) */
#define SET_LS_SCALE_MPCC_solv_20N_no_warm_no_hard_gnone      (MPCC_solv_20N_no_warm_no_hard_gnone_float)(0.95)  

/* minimum required step size in each iteration */
#define SET_LS_MINSTEP_MPCC_solv_20N_no_warm_no_hard_gnone    (MPCC_solv_20N_no_warm_no_hard_gnone_float)(1E-08)

/* maximum step size (combined direction) */
#define SET_LS_MAXSTEP_MPCC_solv_20N_no_warm_no_hard_gnone    (MPCC_solv_20N_no_warm_no_hard_gnone_float)(0.995)

/* desired relative duality gap */
#define SET_ACC_RDGAP_MPCC_solv_20N_no_warm_no_hard_gnone     (MPCC_solv_20N_no_warm_no_hard_gnone_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_MPCC_solv_20N_no_warm_no_hard_gnone     (MPCC_solv_20N_no_warm_no_hard_gnone_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_MPCC_solv_20N_no_warm_no_hard_gnone   (MPCC_solv_20N_no_warm_no_hard_gnone_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_MPCC_solv_20N_no_warm_no_hard_gnone  (MPCC_solv_20N_no_warm_no_hard_gnone_float)(1E-06)

/* desired maximum violation of stationarity (only checked if value is > 0) */
#define SET_ACC_KKTSTAT_MPCC_solv_20N_no_warm_no_hard_gnone  (MPCC_solv_20N_no_warm_no_hard_gnone_float)(-1)

/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_MPCC_solv_20N_no_warm_no_hard_gnone      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_MPCC_solv_20N_no_warm_no_hard_gnone (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_MPCC_solv_20N_no_warm_no_hard_gnone   (2)

/* no progress in line search possible */
#define NOPROGRESS_MPCC_solv_20N_no_warm_no_hard_gnone   (-7)

/* fatal internal error - nans occurring */
#define NAN_MPCC_solv_20N_no_warm_no_hard_gnone  (-10)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_MPCC_solv_20N_no_warm_no_hard_gnone   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_MPCC_solv_20N_no_warm_no_hard_gnone   (-12)

/* thread error */
#define THREAD_FAILURE_MPCC_solv_20N_no_warm_no_hard_gnone  (-98)

/* locking mechanism error */
#define LOCK_FAILURE_MPCC_solv_20N_no_warm_no_hard_gnone  (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_MPCC_solv_20N_no_warm_no_hard_gnone  (-100)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct MPCC_solv_20N_no_warm_no_hard_gnone_params
{
    /* vector of size 8 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_1[8];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_2[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_3[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_4[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_5[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_6[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_7[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_8[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_9[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_10[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_11[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_12[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_13[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_14[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_15[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_16[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_17[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_18[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_19[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_20[10];

    /* vector of size 10 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float c_21[10];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_1[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_2[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_3[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_4[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_5[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_6[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_7[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_8[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_9[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_10[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_11[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_12[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_13[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_14[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_15[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_16[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_17[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_18[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_19[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_20[196];

    /* matrix of size [14 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float H_21[196];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_1[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_2[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_3[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_4[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_5[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_6[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_7[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_8[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_9[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_10[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_11[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_12[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_13[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_14[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_15[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_16[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_17[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_18[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_19[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_20[14];

    /* vector of size 14 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float f_21[14];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_1[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_2[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_3[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_4[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_5[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_6[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_7[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_8[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_9[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_10[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_11[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_12[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_13[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_14[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_15[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_16[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_17[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_18[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_19[140];

    /* matrix of size [10 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float C_20[140];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_2[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_3[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_4[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_5[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_6[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_7[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_8[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_9[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_10[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_11[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_12[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_13[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_14[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_15[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_16[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_17[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_18[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_19[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_20[28];

    /* matrix of size [2 x 14] (column major format) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float A_21[28];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_2[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_3[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_4[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_5[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_6[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_7[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_8[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_9[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_10[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_11[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_12[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_13[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_14[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_15[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_16[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_17[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_18[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_19[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_20[2];

    /* vector of size 2 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float b_21[2];

} MPCC_solv_20N_no_warm_no_hard_gnone_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct MPCC_solv_20N_no_warm_no_hard_gnone_output
{
    /* vector of size 252 */
    MPCC_solv_20N_no_warm_no_hard_gnone_float X[252];

} MPCC_solv_20N_no_warm_no_hard_gnone_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct MPCC_solv_20N_no_warm_no_hard_gnone_info
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    MPCC_solv_20N_no_warm_no_hard_gnone_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    MPCC_solv_20N_no_warm_no_hard_gnone_float res_ineq;

    /* primal objective */
    MPCC_solv_20N_no_warm_no_hard_gnone_float pobj;	
	
    /* dual objective */
    MPCC_solv_20N_no_warm_no_hard_gnone_float dobj;	

    /* duality gap := pobj - dobj */
    MPCC_solv_20N_no_warm_no_hard_gnone_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    MPCC_solv_20N_no_warm_no_hard_gnone_float rdgap;		

	/* infinity norm of gradient of Lagrangian*/
	MPCC_solv_20N_no_warm_no_hard_gnone_float gradient_lag_norm;

    /* duality measure */
    MPCC_solv_20N_no_warm_no_hard_gnone_float mu;

	/* duality measure (after affine step) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float mu_aff;
	
    /* centering parameter */
    MPCC_solv_20N_no_warm_no_hard_gnone_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float step_aff;
    
    /* step size (combined direction) */
    MPCC_solv_20N_no_warm_no_hard_gnone_float step_cc;    

	/* solvertime */
	MPCC_solv_20N_no_warm_no_hard_gnone_float solvetime;   

} MPCC_solv_20N_no_warm_no_hard_gnone_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Monday, April 25, 2022 3:33:14 PM */
/* User License expires on: (UTC) Thursday, October 20, 2022 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Thursday, October 20, 2022 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 93d8ca35-a031-433d-b3bd-307376303f25 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif
extern solver_int32_default MPCC_solv_20N_no_warm_no_hard_gnone_solve(MPCC_solv_20N_no_warm_no_hard_gnone_params *params, MPCC_solv_20N_no_warm_no_hard_gnone_output *output, MPCC_solv_20N_no_warm_no_hard_gnone_info *info, FILE *fs);

;



#ifdef __cplusplus
}
#endif

#endif

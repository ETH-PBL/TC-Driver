/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FORCESNLPsolver_fast.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FORCESNLPsolver_fast_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolver_fast_callback_float *data, FORCESNLPsolver_fast_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolver_fast_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void FORCESNLPsolver_fast_casadi2forces(FORCESNLPsolver_fast_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_fast_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_fast_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_fast_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_fast_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_fast_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_fast_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_fast_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_fast_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_fast_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_fast_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const FORCESNLPsolver_fast_callback_float *in[4];
    FORCESNLPsolver_fast_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	FORCESNLPsolver_fast_float w[27];
	
    /* temporary storage for CasADi sparse output */
    FORCESNLPsolver_fast_callback_float this_f;
    FORCESNLPsolver_fast_float nabla_f_sparse[4];
    FORCESNLPsolver_fast_float h_sparse[4];
    FORCESNLPsolver_fast_float nabla_h_sparse[10];
    FORCESNLPsolver_fast_float c_sparse[8];
    FORCESNLPsolver_fast_float nabla_c_sparse[26];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 19))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolver_fast_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolver_fast_objective_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_fast_objective_0_sparsity_out(1)[1];
			colind = FORCESNLPsolver_fast_objective_0_sparsity_out(1) + 2;
			row = FORCESNLPsolver_fast_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FORCESNLPsolver_fast_dynamics_0(in, out, NULL, w, 0);
		if( c )
		{
			nrow = FORCESNLPsolver_fast_dynamics_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_fast_dynamics_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_fast_dynamics_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_fast_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		if( nabla_c )
		{
			nrow = FORCESNLPsolver_fast_dynamics_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_fast_dynamics_0_sparsity_out(1)[1];
			colind = FORCESNLPsolver_fast_dynamics_0_sparsity_out(1) + 2;
			row = FORCESNLPsolver_fast_dynamics_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_fast_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolver_fast_inequalities_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_fast_inequalities_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_fast_inequalities_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_fast_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolver_fast_inequalities_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_fast_inequalities_0_sparsity_out(1)[1];
			colind = FORCESNLPsolver_fast_inequalities_0_sparsity_out(1) + 2;
			row = FORCESNLPsolver_fast_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((20 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolver_fast_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolver_fast_objective_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_fast_objective_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_fast_objective_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_fast_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_fast_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolver_fast_inequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_fast_inequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_fast_inequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_fast_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolver_fast_inequalities_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_fast_inequalities_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_fast_inequalities_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_fast_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolver_fast_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

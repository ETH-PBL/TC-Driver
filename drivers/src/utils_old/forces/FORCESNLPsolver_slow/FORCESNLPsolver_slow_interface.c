/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FORCESNLPsolver_slow.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FORCESNLPsolver_slow_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolver_slow_callback_float *data, FORCESNLPsolver_slow_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolver_slow_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void FORCESNLPsolver_slow_casadi2forces(FORCESNLPsolver_slow_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_slow_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_slow_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_slow_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_slow_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_slow_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_slow_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_slow_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_slow_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_slow_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_slow_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const FORCESNLPsolver_slow_callback_float *in[4];
    FORCESNLPsolver_slow_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	FORCESNLPsolver_slow_float w[45];
	
    /* temporary storage for CasADi sparse output */
    FORCESNLPsolver_slow_callback_float this_f;
    FORCESNLPsolver_slow_float nabla_f_sparse[4];
    FORCESNLPsolver_slow_float h_sparse[4];
    FORCESNLPsolver_slow_float nabla_h_sparse[10];
    FORCESNLPsolver_slow_float c_sparse[8];
    FORCESNLPsolver_slow_float nabla_c_sparse[27];
            
    
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
		FORCESNLPsolver_slow_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolver_slow_objective_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_slow_objective_0_sparsity_out(1)[1];
			colind = FORCESNLPsolver_slow_objective_0_sparsity_out(1) + 2;
			row = FORCESNLPsolver_slow_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FORCESNLPsolver_slow_dynamics_0(in, out, NULL, w, 0);
		if( c )
		{
			nrow = FORCESNLPsolver_slow_dynamics_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_slow_dynamics_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_slow_dynamics_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_slow_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		if( nabla_c )
		{
			nrow = FORCESNLPsolver_slow_dynamics_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_slow_dynamics_0_sparsity_out(1)[1];
			colind = FORCESNLPsolver_slow_dynamics_0_sparsity_out(1) + 2;
			row = FORCESNLPsolver_slow_dynamics_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_slow_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolver_slow_inequalities_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_slow_inequalities_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_slow_inequalities_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_slow_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolver_slow_inequalities_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_slow_inequalities_0_sparsity_out(1)[1];
			colind = FORCESNLPsolver_slow_inequalities_0_sparsity_out(1) + 2;
			row = FORCESNLPsolver_slow_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((20 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolver_slow_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolver_slow_objective_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_slow_objective_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_slow_objective_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_slow_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_slow_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolver_slow_inequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_slow_inequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_slow_inequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_slow_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolver_slow_inequalities_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_slow_inequalities_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_slow_inequalities_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_slow_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolver_slow_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

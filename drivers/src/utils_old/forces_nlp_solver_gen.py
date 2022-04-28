import yaml
import casadi
import numpy as np
import forcespro
from forcespro import nlp as fnlp

import nlp_functions
import index

Index = index.NlIndex

def main(gen_slow: bool):
    with open('/home/invitedguest/edoardo-ghignone/drivers/src/config/MPC_params.yaml', 'r') as conf:
        MPC_params = yaml.safe_load(conf)

    with open('/home/invitedguest/edoardo-ghignone/drivers/src/config/MPCC_debug.yaml', 'r') as conf:
        DEBUG = yaml.safe_load(conf)

    model = fnlp.SymbolicModel(MPC_params['N']+1)

    model.nvar = MPC_params['n_z'] 
    model.neq = 8
    model.nh = 4
    model.npar = len(index.ParameterIndex) # constraints as [F1 f1 F2 f2] + hard constraints + coordinates(theta) + angle(theta) + derivatives + theta

    model.objective = nlp_functions.eval_obj()

    if gen_slow:
        model.eq = nlp_functions.eval_dynamics_slow()
    else:
        model.eq = nlp_functions.eval_dynamics_fast()

    model.E = np.eye(MPC_params['n_x']-1, MPC_params['n_z'], Index.S_X)

    model.xinitidx = range(Index.S_X, Index.THETA+1)

    model.ineq = nlp_functions.eval_const()
    model.hu = [0, 0, 0, 0]
    model.hl = [-float("inf"), -float("inf"), -float("inf"), -float("inf")]

    states_name = MPC_params['nlp_states_name']
    low_b = []
    upp_b = []
    for key in states_name:
        upp_b.append(MPC_params['upper_bound'][key])
        low_b.append(MPC_params['lower_bound'][key])

    # number of stage variables 
    # number of equality constraints 
    # number of nonlinear inequality constraints 
    # number of runtime parameters
    if gen_slow:
        options = forcespro.CodeOptions('FORCESNLPsolver_slow')
    else:
        options = forcespro.CodeOptions('FORCESNLPsolver_fast')

    options.nlp.stack_parambounds = True
    model.lb = np.tile(low_b, (model.N,))
    model.ub = np.tile(upp_b, (model.N,))

    options.noVariableElimination = 1

    options.optlevel = 3 # 3 maximum, 0 minimum but compilation faster
    options.printlevel = 1

    options.sse = 1 # should help on x86 platforms
    options.parallel = 0 # multithreading/ 0: deactivated, 1: use all possible, n>1: use n threads

    options.nlp.TolStat = 1e-5
    options.nlp.TolEq = 1e-6
    options.nlp.TolIneq = 1e-6
    options.nlp.TolComp = 1e-6

    # options.solvemethod = "SQP_NLP"
    # options.nlp.hessian_approximation = 'gauss-newton'
    # options.sqp_nlp.reg_hessian = 5e-7
    # print(options)

    solver = model.generate_solver(options)

if __name__ == "__main__":

    main(False)
    main(True)

import os
import yaml
import numpy as np
import forcespro

cur_machine = os.environ['USER']

with open('/home/{}/edoardo-ghignone/drivers/src/config/MPC_params.yaml'.format(cur_machine), 'r') as conf:
    MPC_params = yaml.safe_load(conf)

with open('/home/{}/edoardo-ghignone/drivers/src/config/MPCC_debug.yaml'.format(cur_machine), 'r') as conf:
    DEBUG = yaml.safe_load(conf)

lower_bounds = np.array([MPC_params['lower_bound'][key] for key in MPC_params['states_name']])
upper_bounds = np.array([MPC_params['upper_bound'][key] for key in MPC_params['states_name']])

N = MPC_params['N']+1
n_z = MPC_params['n_z']
n_x = MPC_params['n_x']
n_u = MPC_params['n_u']
n_tot = n_z + n_u # we need to add two states to implement the difference penalty

stages = forcespro.MultistageProblem(N)

for i in range(stages.N):
    stages.dims[i]['n'] = n_tot

    if i==0:
        stages.newParam('c_'+str(i+1), [i+1], 'eq.c') 
        stages.eq[i]['D'] = np.eye(7+1, n_tot)
        stages.dims[i]['r'] = 7+1
    else:
        stages.newParam('c_'+str(i+1), [i+1], 'eq.c')
        stages.newParam('C_'+str(i), [i], 'eq.C')
        stages.eq[i]['D'] = np.concatenate((np.eye(7+1, n_tot), np.eye(n_u, n_tot, n_z)), axis=0)
        stages.dims[i]['r'] = 7+1+n_u

    stages.newParam('H_'+str(i+1), [i+1], 'cost.H')
    stages.newParam('f_'+str(i+1), [i+1], 'cost.f')

    
    stages.dims[i]['l'] = n_z
    stages.dims[i]['u'] = n_z 
    stages.ineq[i]['b']['lbidx'] = range(1,n_z+1)
    stages.ineq[i]['b']['lb'] = lower_bounds
    stages.ineq[i]['b']['ubidx'] = range(1,n_z+1)
    stages.ineq[i]['b']['ub'] = upper_bounds
    
    if i==0:
        stages.dims[i]['p'] = 0
    else:
        stages.dims[i]['p'] = 2 # boundary constraints
        stages.newParam('A_'+str(i+1), [i+1], 'ineq.p.A')
        stages.newParam('b_'+str(i+1), [i+1], 'ineq.p.b')

    stages.dims[i]['q'] = 0

stages.newOutput('X', range(1, N+1), range(1, n_z+1))

if DEBUG['warm_start']:
    ws_string='warm'
else:
    ws_string='no_warm'

if DEBUG['hard_track_constr']:
    hard_str='hard'
else:
    hard_str='no_hard'

options = forcespro.CodeOptions('MPCC_solv_{}N_{}_{}_{}'.format(MPC_params['N'], ws_string, hard_str, cur_machine))
options.optlevel = 3 # 3 maximum, 0 minimum but compilation faster
options.printlevel = 2

# x86 stuff
options.sse = 1
options.avx = 1

stages.codeoptions = options

# stages.codeoptions.accuracy.eq = 1e-4
# stages.codeoptions.accuracy.ineq = 1e-4
# stages.codeoptions.accuracy.rdgap = 1e-2
# stages.codeoptions.accuracy.mu = 1e-4

#stages.codeoptions.linesearch.minstep = 1e-9
#stages.codeoptions.linesearch.maxstep = 0.95

#stages.codeoptions.parallel = 1 # just a flag to activate multi threading
#stages.codeoptions.linesearch.factor_aff = 0.8
#stages.codeoptions.linesearch.factor_cc = 0.85

# options.solvemethod = "SQP_NLP"

if DEBUG['warm_start']:
    stages.codeoptions.init = 2
else:
    stages.codeoptions.init = 0
#import get_userid
#stages.generateCode(get_userid.userid)
stages.generateCode()


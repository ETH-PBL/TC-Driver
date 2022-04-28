import pathlib
from typing import List

import yaml
import numpy as np
from numba.core import types
from numba.typed import Dict
from numpy import sin, cos
from scipy import sparse

from splinify.splinify import SplineTrack

from .index import Index
from . import comp_utils

cur_dir = str(pathlib.Path(__file__).resolve().parents[2])
config_dir = cur_dir + '/config/'

with open(config_dir+"MPCC_debug.yaml", "r") as conf:
    DEBUG = yaml.safe_load(conf) 

with open(config_dir+"MPC_params.yaml", "r") as conf:
    MPC_params = yaml.safe_load(conf) 


SLOW_THRESHOLD = MPC_params['slow_threshold']


def linearise_error(X: List[float], track: SplineTrack, error_type: str):
    """
    Function that linearises the error function, basically e_c(theta) = e_c(theta_0) + (de_c/dtheta | theta_0) theta but matrices
    I took great inspiration from A. Liniger matlab code: https://github.com/alexliniger/MPCC/blob/master/Matlab/getMPCmatrices.m

    Args:
        X: the state at which we linearise, it is the state of the mpc formulation already so \in R^10

    Returns:
        Nabla_e_t0 : gradient
        ec_t0 : e_c(theta_0)
        error_type: can be either 'lag' or 'cont' (for contouring) 
    """

    n_x = MPC_params['n_x']
    x1 = X[Index.S_X]
    x2 = X[Index.S_Y]
    theta = X[Index.THETA]
    X_ref, Y_ref = track.get_coordinate(theta)
    dx_dth, dy_dth = track.get_derivative(theta)
    phi = track.get_angle(theta)
    dphi_dtheta = track.get_dphi_dtheta(theta)

    if error_type == 'cont':
        Nabla_e_t0, ec_t0 = comp_utils.linearise_contouring_error(
            x1, x2, X_ref, Y_ref, phi, dphi_dtheta, dx_dth, dy_dth, n_x
            )
    
    elif error_type == 'lag':
        Nabla_e_t0, ec_t0 = comp_utils.linearise_lag_error(
            x1, x2, X_ref, Y_ref, phi, dphi_dtheta, dx_dth, dy_dth, n_x
            )

    if np.isnan(Nabla_e_t0).any() or np.isnan(ec_t0).any():
        raise ValueError("NaNs in error linearisation!!!")

    return Nabla_e_t0, ec_t0

def get_track_constraints(track: SplineTrack, theta: float):
    """
    Returns the n_x-d matrices corresponding to the track limits for a given theta
    """

    n_x = MPC_params['n_x']

    int_point = np.array(track.get_coordinate(theta, line='int'))
    out_point = np.array(track.get_coordinate(theta, line='out'))
    slope_ang = track.get_angle(theta)

    norm_vec = np.array([cos(slope_ang- np.pi/2), sin(slope_ang - np.pi/2)])
    
    q_int = np.dot(int_point, norm_vec)
    q_out = np.dot(out_point, norm_vec)

    F = np.zeros((2, n_x))
    f = np.zeros((2,))
    
    if q_out <=q_int:
        # if the outer point of the track is on this side of the half-plane
        # then the inequality for the interior point must include it

        F[0, :Index.S_Y+1] = norm_vec
        f[0] = q_int
        F[1, :Index.S_Y+1] = -norm_vec
        f[1] = -q_out
    else:
        F[0, :Index.S_Y+1] = -norm_vec
        f[0] = -q_int
        F[1, :Index.S_Y+1] = norm_vec
        f[1] = q_out

    return F, f

def get_MPC_matrices(X, U, Z_hist, track:SplineTrack, track_hard: SplineTrack, car_params, slow_threshold):
    """
    Generates the matrices of the MPC problem already expressed as QP

    Args:
        X: current state, 9-D
        U: previous input at current timestep
        Z_hist: previous solution of future timesteps, Nxn_z dimensional 
    """

    n_x = MPC_params['n_x']
    n_u = MPC_params['n_u']
    n_z = n_x+n_u
    N = MPC_params['N']
    gamma = MPC_params['gamma']
    T_s = MPC_params['T_s']
    nu_soft = MPC_params['nu_soft']
    R = np.diag(MPC_params['R'])

    # check slower velocity, to decide which model to use 
    vels = np.concatenate((np.array([X[Index.V]]), Z_hist[:, Index.V]))
    thresh_func = lambda x : x < slow_threshold
    use_slow = thresh_func(vels).any()
    max_vel = np.max(vels)

    A_len = n_x
    if DEBUG['track_constr']:
        A_len += 2*N
    if DEBUG['hard_track_constr']:
        A_len += 2*N
    if DEBUG['theta_dyn']:
        A_len += N
    if DEBUG['sys_dyn']:
        A_len += 7*N
    if DEBUG['general_constr']:
        A_len += 11*N + 2

    P = np.zeros((n_z*(N+1), n_z*(N+1)))
    q = np.zeros((n_z*(N+1), 1))

    # initial setup of constraints, with x_0 and theta_0
    A = np.zeros((A_len, n_z*(N+1)))
    u = np.zeros(A_len)
    u[:n_x] = X[:n_x]
    l = np.zeros(A_len)
    l[:n_x] = X[:n_x]
    
    # just track constraint, for debugging
    F_constr = np.zeros((1, 2))
    f_constr = np.array([0])

    id_mat = np.eye(n_x)

    for i, row in enumerate(id_mat):
        if DEBUG['orig_liniger']:
            if i != Index.V_K:
                A[i,:] = np.pad(row, (0, n_z*(N)+n_u), 'constant')
        else:
            A[i,:n_x] = row #np.pad(row, (0, n_z*(N)+n_u), 'constant')

    ind_A = n_x

    q_c = MPC_params['q_c']
    q_l = MPC_params['q_l']
    for k in range(N):
        # COST FUNCTION
        # error approximations
        if DEBUG["error_approx"]:
            E_c, e_c = linearise_error(Z_hist[k, :n_x], track, 'cont')
            E_l, e_l = linearise_error(Z_hist[k, :n_x], track, 'lag')
            tmp = q_c*E_c@E_c.T + q_l*E_l@E_l.T 
            P[n_z*(k+1):n_z*(k+1) + n_x, n_z*(k+1):n_z*(k+1)+n_x] += tmp
            tmp_prod_c, = Z_hist[k, :n_x]@E_c
            tmp_prod_l, = Z_hist[k, :n_x]@E_l
            if k == N:
                q[n_z*(k+1):n_z*(k+1)+n_x] = 20*q_c*(e_c*E_c - tmp_prod_c*E_c) + 2*q_l*(e_l*E_l - tmp_prod_l*E_l)
            else:
                q[n_z*(k+1):n_z*(k+1)+n_x] = 2*q_c*(e_c*E_c - tmp_prod_c*E_c) + 2*q_l*(e_l*E_l - tmp_prod_l*E_l)
            
        # advancement part
        if DEBUG['orig_liniger']:
            q[n_z*k+Index.V_K] += -gamma*T_s
        else: 
            q[n_z*k+Index.V] += -gamma*T_s

        # cost on variation of inputs
        if DEBUG['var_inputs']:
            mod = np.zeros((2*n_z, n_u+1))
            mod[Index.V, 0] = -1
            mod[n_z + Index.V, 0] = 1
            mod[Index.V_DELTA, 1] = -1
            mod[n_z + Index.V_DELTA, 1] = 1
            mod[Index.ACC, 2] = -1
            mod[n_z + Index.ACC, 2] = 1
            matr = mod @ R @ mod.T 
            P[k*n_z:(k+2)*n_z, k*n_z:(k+2)*n_z] += matr

        # additional penalty on slip angle 
        #P[(k+1)*n_z + Index.BETA, (k+1)*n_z + Index.BETA] += 10

        # soft constraints
        soft_mult = 1 # + max_vel*1e2
        q[(k+1)*n_z+Index.SOFT] = soft_mult*nu_soft 


        # INEQUALITY CONSTRAINTS
        # track constraints
        if DEBUG['track_constr']:
            F, f = get_track_constraints(track, Z_hist[k,7])
            
            A[ind_A:ind_A+2, (k+1)*n_z:(k+1)*n_z+n_x] = F
            A[ind_A:ind_A+2, (k+1)*n_z+Index.SOFT] = -1
            #A_row = np.pad(F[0,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            #A_row[(k+1)*n_z+Index.SOFT] = -1
            #A = np.append(A, [A_row], axis=0)

            #A_row = np.pad(F[1,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            #A_row[(k+1)*n_z+Index.SOFT] = -1
            #A = np.append(A, [A_row], axis=0)

            u[ind_A:ind_A+2] = f #np.append(u, f, axis=0)
            l[ind_A:ind_A+2] = -np.inf*np.ones(shape=f.shape)#np.append(l, -np.inf*np.ones(shape=f.shape), axis=0)
            ind_A += 2

        if DEBUG['hard_track_constr']:
            F, f = get_track_constraints(track_hard, Z_hist[k,Index.THETA])

            A[ind_A:ind_A+2, (k+1)*n_z:(k+1)*n_z+n_x] = F
            u[ind_A:ind_A+2] = f 
            l[ind_A:ind_A+2] = -np.inf*np.ones(shape=f.shape)
            ind_A += 2

            F_constr = np.append(F_constr, F[:, :2], axis=0)
            f_constr = np.append(f_constr, f, axis=0)
            
            #A_row = np.pad(F[0,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            #A = np.append(A, [A_row], axis=0)


            #A_row = np.pad(F[1,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            #A = np.append(A, [A_row], axis=0)
            

            #u = np.append(u, f, axis=0)
            #l = np.append(l, -np.inf*np.ones(shape=f.shape), axis=0)


        # EQUALITY CONSTRAINTS
        # theta dynamic
        if DEBUG['theta_dyn']:
            A_row = np.zeros((n_z*(N+1), 1))
            
            A_row[n_z*(k+1)+Index.THETA] = 1
            A_row[n_z*k+Index.THETA] = -1 
            if DEBUG["orig_liniger"]:
                A_row[n_z*k+Index.V] = -T_s # in the original paper it actually says 1/T_s
            else: 
                if DEBUG['new_theta_dyn']:
                    # this new version considers an improved theta dynamics
                    if k==0:
                        cur_th = X[Index.THETA]
                        cur_phi = X[Index.PHI]
                        cur_v = X[Index.V]
                    else:
                        cur_th = Z_hist[k-1, Index.THETA]
                        cur_phi = Z_hist[k-1, Index.PHI]
                        cur_v = Z_hist[k-1, Index.V]

                    track_angle = track.get_angle(cur_th)
                    ang_diff = (track_angle - cur_phi)%(2*np.pi)
                    A_row[n_z*k+Index.THETA] += T_s*cur_v*np.sin(ang_diff)*track.get_dphi_dtheta(cur_th)
                    A_row[n_z*k+Index.V] += -T_s*np.cos(ang_diff)
                    A_row[n_z*k+Index.PHI] += T_s*cur_v*np.sin(ang_diff)

                    const_lin_theta = cur_th + T_s*cur_v*np.cos(ang_diff) - (1 - T_s*cur_v*np.sin(ang_diff)*track.get_dphi_dtheta(cur_th))*cur_th - (T_s*np.cos(ang_diff)*cur_v) - (-T_s*cur_v*np.sin(ang_diff)*cur_phi)
                else:
                    A_row[n_z*k+Index.V] = -T_s # this part is different from the original paper
           
            
            A[ind_A, :] = A_row.T #np.append(A, A_row.T, axis=0)
            theta_const = 0
            if DEBUG['theta_dyn']:
                if DEBUG['new_theta_dyn']:
                    theta_const = const_lin_theta
            
            u[ind_A] = theta_const #np.append(u, [theta_const], axis=0)
            l[ind_A] = theta_const #np.append(l, [theta_const], axis=0)
            ind_A += 1

        # dynamic
        if k == 0:
            A_l, B_l, g_l = comp_utils.linearise_dynamics(X[:7], U, use_slow)
        else:
            A_l, B_l, g_l = comp_utils.linearise_dynamics(Z_hist[k-1,:7], Z_hist[k-1,n_x:], use_slow)

        A_k, B_k, g_k = comp_utils.discretise_dynamics(A_l, B_l, g_l, T_s)

        if DEBUG['sys_dyn']: 
            A[ind_A:ind_A+7, (k+1)*n_z:(k+1)*n_z+7] = np.eye(7)
            A[ind_A:ind_A+7, k*n_z:k*n_z+7] = -A_k
            A[ind_A:ind_A+7, k*n_z+n_x:(k+1)*n_z] = -B_k
            u[ind_A:ind_A+7] = g_k
            l[ind_A:ind_A+7] = g_k
            ind_A += 7
            """
            for i, row in enumerate(A_k):
                A_row = np.zeros((n_z*(N+1), 1))

                A_row[(k+1)*n_z+i] = 1
                A_row[k*n_z:k*n_z+7] = -row.reshape((7, 1))
                A_row[k*n_z+n_x:(k+1)*n_z] = -B_k[i,:].reshape((2,1))
                A[ind_A, :] = A_row.T
                u[ind_A] = g_k[i]
                l[ind_A] =  g_k[i]
                ind_A += 1
                
                #A = np.append(A, A_row.T, axis=0)
                #u = np.append(u, [g_k[i]], axis=0)
                #l = np.append(l, [g_k[i]], axis=0)
            """

        # general boundaries on states
        if DEBUG['general_constr']:
            # TODO change this with those from config file
            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*k+Index.S_X] = 1
            A[ind_A, n_z*(k+1)+Index.S_X] = 1 # = np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['s_x'] #np.append(u, [MPC_params['upper_bound']['s_x']], axis=0)
            l[ind_A] = MPC_params['lower_bound']['s_x'] #np.append(l, [MPC_params['lower_bound']['s_x']], axis=0)
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.S_Y] = 1
            A[ind_A, n_z*(k+1)+Index.S_Y] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['s_y']
            l[ind_A] = MPC_params['lower_bound']['s_y']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.DELTA] = 1
            A[ind_A, n_z*(k+1)+Index.DELTA] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['delta']
            l[ind_A] = MPC_params['lower_bound']['delta']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.V] = 1
            A[ind_A, n_z*(k+1)+Index.V] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['v']
            l[ind_A] = MPC_params['lower_bound']['v']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.PHI] = 1
            A[ind_A, n_z*(k+1)+Index.PHI] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['phi']
            l[ind_A] = MPC_params['lower_bound']['phi']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.PHI_DOT] = 1
            A[ind_A, n_z*(k+1)+Index.PHI_DOT] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['phi_dot']
            l[ind_A] = MPC_params['lower_bound']['phi_dot']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.BETA] = 1
            A[ind_A, n_z*(k+1)+Index.BETA] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['beta']
            l[ind_A] = MPC_params['lower_bound']['beta']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.THETA] = 1
            A[ind_A, n_z*(k+1)+Index.THETA] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['theta']
            l[ind_A] = MPC_params['lower_bound']['theta']
            ind_A += 1

            # soft constr positivity
            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.SOFT] = 1
            A[ind_A, n_z*(k+1)+Index.SOFT] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = np.inf
            l[ind_A] = MPC_params['lower_bound']['soft']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.ACC] = 1
            A[ind_A, n_z*(k+1)+Index.ACC] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['acc']
            l[ind_A] = MPC_params['lower_bound']['acc']
            ind_A += 1

            #A_row = np.zeros((n_z*(N+1), 1))
            #A_row[n_z*(k+1)+Index.V_DELTA] = 1
            A[ind_A, n_z*(k+1)+Index.V_DELTA] = 1 #np.append(A, A_row.T, axis=0)
            u[ind_A] = MPC_params['upper_bound']['v_delta']
            l[ind_A] = MPC_params['lower_bound']['v_delta']
            ind_A += 1


    # boundary on input, first timestep
    A[ind_A, Index.ACC] = 1 #np.append(A, A_row.T, axis=0)
    u[ind_A] = MPC_params['upper_bound']['acc']
    l[ind_A] = MPC_params['lower_bound']['acc']
    ind_A += 1

    #A_row = np.zeros((n_z*(N+1), 1))
    #A_row[n_z*(k+1)+Index.V_DELTA] = 1
    A[ind_A, Index.V_DELTA] = 1 #np.append(A, A_row.T, axis=0)
    u[ind_A] = MPC_params['upper_bound']['v_delta']
    l[ind_A] = MPC_params['lower_bound']['v_delta']
    ind_A += 1
    
    
    # COST FUNCTION
    # advancement part
    q[n_z*N+Index.V] += -gamma*T_s

    # sparsification
    P = sparse.csc_matrix(0.5*(P+P.T)) 
    #A = sparse.csc_matrix(A)
    #print(A.getnnz())
    
    F_constr = F_constr[1:, :]
    f_constr = f_constr[1:]

    return P, q, A, l, u, F_constr, f_constr

def get_initial_warmstart(N:int, v_x0:float, MPC_params, track:SplineTrack, start_point, start_theta, a_0: float = 0):
    """
    Obtains the initial warmstart. Similarly as in the MATLAB implementation
    by A. Linigier, a constant velocity is assumed 
    """

    in_coord = np.array(start_point[:2])
    theta = track.find_theta(in_coord, start_theta)
    coords = track.get_coordinate(theta)
    x = np.zeros(MPC_params['n_z']*(MPC_params['N']+1))

    x[Index.S_X] = coords[0]
    x[Index.S_Y] = coords[1]
    x[Index.V] = v_x0
    prev_phi = start_point[2]
    x[Index.PHI] = prev_phi
    x[Index.THETA] = theta
    x[Index.V_K] = v_x0

    for k in range(MPC_params['N']):
        theta += v_x0*MPC_params['T_s']
        coords = track.get_coordinate(theta)
        new_x = np.zeros(MPC_params['n_z'])
        
        new_x[Index.S_X] = coords[0]
        new_x[Index.S_Y] = coords[1]
        new_x[Index.V] = v_x0
        new_phi = track.get_angle(theta)
        if np.abs(new_phi - prev_phi) > np.pi:
            if new_phi > prev_phi:
                new_phi -= 2*np.pi
            else:
                new_phi += 2*np.pi
        prev_phi = new_phi

        new_x[Index.PHI] = new_phi 
        new_x[Index.THETA] = theta
        new_x[Index.V_K] = v_x0
        
        x[(k+1)*MPC_params['n_z']:(k+2)*MPC_params['n_z']] = new_x.copy()
        v_x0 += MPC_params['T_s']*a_0

    return x

def get_serious_init_ws(track:SplineTrack, start_point):
    """
    Obtains the initial warmstart by assuming a costant steering of 0 and a constant acceleration
    """
    a0 = 5
    N = MPC_params['N']
    n_x = MPC_params['n_x']
    n_z = MPC_params['n_z']
    T_s = MPC_params['T_s']

    
    in_coord = np.array(start_point[:2])
    theta = track.find_theta(in_coord, MPC_params['start_par'])
    x = np.empty((N+1)*n_z)
    coords = track.get_coordinate(theta)
    x[:n_z] = [coords[0], coords[1], 0, 0, start_point[2], 0, 0, theta, 0, 0, a0]
    for k in range(MPC_params['N']):
        new_x = comp_utils.dynamics(x[k*n_z:(k)*n_z+n_x], x[k*n_z + n_x:(k+1)*n_z], T_s, x[k*n_z+Index.V]<0.1)
        x[(k+1)*n_z:(k+1)*n_z+n_x-2] = new_x
        x[(k+1)*n_z+n_x-2:(k+1)*n_z+n_x] = [track.find_theta(x[(k+1)*n_z:(k+1)*n_z+2], x[(k)*n_z+Index.THETA]+x[(k)*n_z+Index.V]*T_s), 0]
        x[(k+1)*n_z+n_x:(k+2)*n_z] = [0, a0]
    
    return x

def update_warm_start(prev_result, car_params, track, slow_threshold):
    """
    Updates the warm start used in solving the MPC.

    Args: 
        prev_result: vector of dimension Nxn_z containing previous MPC solution
        warmstart: a N*n_z vector

    """

    x = prev_result
    N = MPC_params['N']
    n_z = MPC_params['n_z']
    n_x = MPC_params['n_x']
    n_u = MPC_params['n_u']
    mpc_ts = MPC_params['T_s']
    # safe_input = np.array([0, 0.1])
    safe_input = prev_result[-n_z-n_u:-n_z] # penultimate input

    thresh_func = lambda x : x < slow_threshold
    use_slow = thresh_func(x[Index.V::n_z]).any()

    # print("PHIS, BETAS")
    # print(",".join(prev_result[Index.PHI::12].astype(str)))
    # print(",".join(prev_result[Index.BETA::12].astype(str)))

    
    # if the car timestep (period of solving) is higher than the MPC horizon timestep 
    # better car_ts is a multiple of mpc_ts

    new_warm_start = np.zeros((N+1)*n_z)
    mult = int(car_params['T_s']/mpc_ts)
    rest = car_params['T_s']%mpc_ts
    
    if mult != 0:
        # we shift the esisting solution
        new_warm_start[:-(mult*n_z)] = prev_result[(mult*n_z):]
        new_warm_start[-((mult+1) * n_z)+Index.V_K] = new_warm_start[-n_z+Index.V]

        # we generate a new tail with a "safe" input
        for i in range(mult):
            next_st, cur_safe_input = constr_dynamics(
                new_warm_start[-((mult-i+1) * n_z):-((mult-i+1) * n_z) + 7], 
                safe_input, 
                mpc_ts, 
                use_slow,
            )
            new_warm_start[-((mult-i)*n_z)-n_u:-((mult-i)*n_z)] = cur_safe_input
            new_warm_start[-((mult-i) * n_z):-((mult-i) * n_z) + 7] = next_st
            new_warm_start[-((mult-i) * n_z)+Index.THETA] = new_warm_start[-((mult-i+1) * n_z)+Index.THETA] + mpc_ts*new_warm_start[-((mult-i+1) * n_z)+Index.V]
            new_warm_start[-((mult-i) * n_z)+Index.V_K] = new_warm_start[-((mult-i) * n_z)+Index.V]
    
    # we shift everything by the additional rest of timestep 
    if rest > 0.001:
        if mult == 0:
            # if mult is zero we must fill new_warm_start
            new_warm_start = prev_result
        
        for i in range(N+1):
            # update state part:
            next_st, cur_safe_input = constr_dynamics(
                new_warm_start[i*n_z:i*n_z + 7],
                new_warm_start[i*n_z + n_x:(i+1)*n_z],
                rest,
                use_slow,
            )
            new_warm_start[i*n_z:i*n_z + 7] = next_st
            new_warm_start[i*n_z+Index.THETA] += rest*new_warm_start[i*n_z+Index.V]
            if i != 0:
                new_warm_start[i*n_z - n_u:i*n_z] = cur_safe_input

    if np.isnan(new_warm_start).any():
        print(new_warm_start)
        raise ValueError("Some nannnnnnssss")        

    return new_warm_start

def update_warm_start_safe(inputs, prev_ws, car_params, track, slow_threshold):
    """
    Updates the warmstart used in the MPC when a good enough solution is not found
    
    """
    N = MPC_params['N']
    n_x = MPC_params['n_x']
    n_z = MPC_params['n_z']
    T_s = MPC_params['T_s']

    thresh_func = lambda x : x < slow_threshold
    use_slow = thresh_func(prev_ws[Index.V::n_z]).any()

    new_warm_start = np.zeros(n_z*(N+1))
    vel = prev_ws[Index.V]

    print("Safe ws")
    for i in range(N+1):
        if i==0:
            new_warm_start[:7] = comp_utils.dynamics(prev_ws[:n_x], inputs, T_s=car_params['T_s'], use_slow=use_slow)
            theta = prev_ws[Index.THETA] + car_params['T_s']*vel
            # find_theta(new_warm_start[:2], track, prev_ws[Index.THETA+n_z])
            # assert theta > prev_theta
            # prev_theta = theta
            new_warm_start[Index.THETA] = theta
            new_warm_start[Index.V_K] = prev_ws[Index.V_K]
            new_warm_start[n_x:n_z] = inputs
        elif i!=N:
            new_warm_start[i*n_z:7+i*n_z] = comp_utils.dynamics(new_warm_start[(i-1)*n_z:7+(i-1)*n_z], prev_ws[(i)*n_z+n_x:(i+1)*n_z], T_s=T_s, use_slow=use_slow)
            theta = prev_ws[i*n_z+Index.THETA]+ car_params['T_s']*vel
            #find_theta(new_warm_start[i*n_z:i*n_z+2], track, prev_ws[Index.THETA+n_z*(i+1)])
            # assert theta > prev_theta
            # prev_theta = theta
            new_warm_start[i*n_z + Index.THETA] = theta
            new_warm_start[i*n_z + Index.V_K] = prev_ws[i*n_z + Index.V_K]
            new_warm_start[i*n_z+n_x:(i+1)*n_z] = prev_ws[i*n_z+n_x:(i+1)*n_z]
        else:
            new_warm_start[i*n_z:7+i*n_z] = comp_utils.dynamics(new_warm_start[(i-1)*n_z:7+(i-1)*n_z], prev_ws[(i-1)*n_z+n_x:(i)*n_z], T_s=T_s, use_slow=use_slow)
            theta = prev_ws[Index.THETA+n_z*(i)] + car_params['T_s']*vel
            #find_theta(new_warm_start[i*n_z:i*n_z+2], track, prev_ws[Index.THETA+n_z*(i)]+T_s*new_warm_start[i*n_z+Index.V])
            # assert theta > prev_theta
            # prev_theta = theta
            new_warm_start[i*n_z + Index.V_K] = prev_ws[i*n_z + Index.V_K]
            new_warm_start[i*n_z+Index.THETA] = theta
    #print(new_warm_start)
    return new_warm_start

def constr_dynamics(X: np.array, U: np.array, T_s: float, use_slow: bool):
    """
    Obtains the next state considering the limits on steering and speed

    Returns:
        next_st: the 7-d next state
        U: constrained input
    """
    
    next_st = comp_utils.dynamics(X, U, T_s, use_slow)

    gen_new = False
    if next_st[Index.V] > MPC_params['upper_bound']['v'] or next_st[Index.V] < MPC_params['lower_bound']['v']:
        U[1] = 0
        gen_new = True

    if next_st[Index.DELTA] > MPC_params['upper_bound']['delta'] or next_st[Index.DELTA] < MPC_params['lower_bound']['delta']:
        U[0] = 0
        gen_new = True

    if gen_new:
        next_st = comp_utils.dynamics(X, U, T_s, use_slow)
    
    return next_st, U

def get_inputs(acc:float, steer_vel:float, cur_steer:float, cur_vel:float):
    """
    Obtains the proper inputs to feed into the gym environment
    """
    max_a = MPC_params['upper_bound']['acc']
    max_v = MPC_params['upper_bound']['v']
    min_v = MPC_params['lower_bound']['v']

    steer = cur_steer + steer_vel*0.01

    #print(cur_vel)
    if cur_vel >= 0.:
        if (acc > 0):
            # accelerate
            kp = 2.0 * max_a / max_v
        else:
            # braking
            kp = 2.0 * max_a / (-min_v)
    # currently backwards
    else:
        if (acc > 0):
            # braking
            kp = 2.0 * max_a / max_v
        else:
            # accelerating
            kp = 2.0 * max_a / (-min_v)
        
    vel_diff = acc/kp
    vel = cur_vel + vel_diff

    return steer, vel

def get_forces_params(prev_inputs, X, U, Z_hist, track, track_hard, slow_threshold):

    params = {}
    F_constr = np.empty((0, 2))
    f_constr = np.empty((0))

    use_slow = np.min((X[Index.V], np.min(Z_hist[:, Index.V]))) < slow_threshold
    N = MPC_params['N']
    n_x = MPC_params['n_x']
    n_u = MPC_params['n_u']
    n_z = MPC_params['n_z']
    n_tot=n_z+n_u
    T_s = MPC_params['T_s']
    q_c = MPC_params['q_c']
    q_l = MPC_params['q_l']
    gamma = MPC_params['gamma']
    nu_soft = MPC_params['nu_soft']
    R = np.diag(MPC_params['R'])
    max_A_el = -1

    for i in range(N+1):
        if DEBUG['warm_start']:
            key = 'z_init_{0:02d}'.format(i)
            value = np.empty((n_tot))
            if i==0:
                value[:n_x] = X
                value[n_x:n_z] = U
                value[n_z:n_tot] = prev_inputs
                
            if i==1:
                value[:n_z] = Z_hist[i-1, :]
                value[n_z:n_tot] = U
            else:
                value[:n_z] = Z_hist[i-1, :]
                value[n_z:n_tot] = Z_hist[i-2, n_x:]
            params[key] = value
        
        num_str = str(i+1)
        
        theta_dynamic = np.zeros(n_tot)

        # THETA AND STATE DYNAMIC
        theta_dynamic[Index.THETA] =  -1
        theta_dynamic[Index.V_K] = -T_s 
        const_lin_theta = 0
        
        key = 'c_' + num_str
        if i==0:
            params[key] = X[:Index.THETA+1]
        else:
            if i==1:
                A_l, B_l, g_l = comp_utils.linearise_dynamics(
                    X[:7], 
                    U,
                    use_slow,
                    )
            else:
                A_l, B_l, g_l = comp_utils.linearise_dynamics(
                    Z_hist[i-2, :7], 
                    Z_hist[i-2, n_x:],
                    use_slow,
                    )
            A_k, B_k, g_k = comp_utils.discretise_dynamics(A_l, B_l, g_l, T_s)

            max_A_el= max(np.abs(A_k).max(), max_A_el)
            
            state_const = np.zeros(A_k.shape[0]+1+n_u)
            state_const[:7] = g_k
            state_const[Index.THETA] = const_lin_theta
            params[key] = state_const

            key = 'C_' + str(i)

            # do not change 0-th dim, it's equal to n_x only by chance
            state_matr = np.zeros((A_k.shape[0]+1+n_u, n_tot)) 
            state_matr[:7, :7] = -A_k
            state_matr[:7, n_x:n_z] = -B_k
            state_matr[Index.THETA, :] = theta_dynamic.T
            state_matr[8:, :] = -np.eye(n_u, n_tot, n_x)
            params[key] = state_matr

        # COST
        H = np.zeros((n_tot, n_tot))
        f = np.zeros(n_tot)

        if DEBUG["error_approx"]:
            E_c, e_c = linearise_error(Z_hist[i-1, :n_x], track, 'cont')
            E_l, e_l = linearise_error(Z_hist[i-1, :n_x], track, 'lag')
            tmp = q_c*E_c@E_c.T + q_l*E_l@E_l.T 

            H[:n_x, :n_x] += tmp
            tmp_prod_c, = Z_hist[i-1, :n_x] @ E_c
            tmp_prod_l, = Z_hist[i-1, :n_x] @ E_l
            if i == N:
                f[:n_x] += np.array(
                        20*q_c*(e_c*E_c - tmp_prod_c*E_c) \
                            + 2*q_l*(e_l*E_l - tmp_prod_l*E_l)
                    ).reshape(n_x,)
            else:
                f[:n_x] += np.array(
                    2*q_c*(e_c*E_c - tmp_prod_c*E_c) \
                        + 2*q_l*(e_l*E_l - tmp_prod_l*E_l)
                    ).reshape(n_x,)
        
        # H[Index.BETA, Index.BETA] += 0.1
        # H[Index.DELTA, Index.DELTA] += 0.0001
        # H[Index.V_DELTA, Index.V_DELTA] += 0.00001

        f[Index.V_K] += -gamma*T_s

        if DEBUG['var_inputs']: 
            red_R = R[1:,1:]
            mod = np.zeros((n_tot, n_u))
            mod[Index.V_DELTA, 0] = -1
            mod[Index.V_DELTA+n_u, 0] = 1
            mod[Index.ACC, 1] = -1
            mod[Index.ACC+n_u, 1] = 1
            matr = mod @ red_R @ mod.T 
            H += matr

        # SOFT COST
        soft_mult = 1 # + max_vel*1e2
        f[Index.SOFT] = soft_mult*nu_soft 
        H[Index.SOFT, Index.SOFT] += nu_soft*10

        # another personal addition
        # should enforce alignment of last state with track
        # Liniger confirmed it's sensible
        # if i==N:
        #     coeff = 1
        #     H[Index.PHI, Index.PHI] += coeff
        #     f[Index.PHI] += -2*track.get_angle(
        #           Z_hist[i-1, Index.THETA]
        #         )*coeff

        key = 'H_' + num_str
        params[key] = 2*H
        key = 'f_' + num_str
        params[key] = f

        if i>0 :
            # track constraints
            F, f = get_track_constraints(track, Z_hist[i-1, Index.THETA])
            F[:, Index.SOFT] = -1
            F_constr = np.append(F_constr, F[:, :2], axis=0)
            f_constr = np.append(f_constr, f, axis=0)

            if DEBUG['hard_track_constr']:
                A_constr = F
                b_constr = f
                pass
                """F_hard, f_hard = get_track_constraints(
                        track_hard, Z_hist[i-1, Index.THETA]
                    )

                F_constr = np.append(F_constr, F_hard[:, :2], axis=0)
                f_constr = np.append(f_constr, f_hard, axis=0)
            
                A_constr = np.concatenate((F, F_hard), axis=0)
                b_constr = np.concatenate((f, f_hard), axis=None)"""
            else:
                A_constr = F
                b_constr = f

            constr_matr = np.zeros((A_constr.shape[0], n_tot))
            constr_matr[:, :n_x] = A_constr
            #print("A constr, ", A_constr) 

            key = 'A_' + num_str
            params[key] = constr_matr
            key = 'b_' + num_str
            params[key] = b_constr
    
    return params, F_constr, f_constr, max_A_el

from typing import List
from enum import IntEnum

import numpy as np
from numpy import arctan, sin, cos, tan
from scipy import sparse

from f110_gym.envs.base_classes import RaceCar
from pkg.src.pkg.utils.splinify import SplineTrack

VEL_THRESH_SLOW_DYN = 30

DEBUG = {
    "error_approx": bool(1),
    "track_constr" : bool(1),
    "hard_track_constr": bool(1),
    "theta_dyn": bool(1),
    "sys_dyn": bool(1),
    "general_constr": bool(1),
    "var_inputs": bool(1),
    "orig_liniger": bool(0)
}

class Index(IntEnum):
    S_X = 0
    S_Y = 1
    DELTA = 2
    V = 3
    PHI = 4
    PHI_DOT = 5
    BETA = 6
    THETA = 7
    V_K = 8
    SOFT = 9
    V_DELTA = 10
    ACC = 11

def linearise_dynamics(X: List[float], U: List[float], car_params, use_slow, g:float = 9.81):
    """
    Function that linearises the dynamics; since the dynamics is defined in terms of absolute x,
    there is also a constant.
    A kinematic model is used for slow speeds, the single-track model for higher speeds.
    Both models can be found in section 7 of the following document:
        https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf

    Args:
        X: state at which we linearise \in R^7
        U: input at which we linearise \in R^2
        car: RaceCar object used to get the car's spcifics
        g: acceleration of a whale falling down, in absence of friction sources, on earth

    Returns:
        A: the linearised state to state matrix, it is 7-dimensional, does not consider the added states used for MPC
        B: lin. input to state matrix, \in R^(7x2)
        const_term: the f(x_0) part in the linearisation, if we consider f(x) = f(x_0) + (df/dx|x0)(x)
    """

    x1 = X[Index.S_X]
    x2 = X[Index.S_Y]
    x3 = X[Index.DELTA]
    x4 = X[Index.V]
    x5 = X[Index.PHI]
    x6 = X[Index.PHI_DOT]
    x7 = X[Index.BETA]

    u1 = U[0]
    u2 = U[1]

    l_r = car_params['lr']
    l_f = car_params['lf']
    l_wb = car_params['lr'] + car_params['lf']
    mu = car_params['mu']
    C_sf = car_params['C_sf']
    C_sr = car_params['C_sr']
    I_z = car_params['I']
    m = car_params['m']
    h_cg = car_params['h']


    p1 = C_sf*(g*l_r - u2*h_cg)
    p2 = C_sr*(g*l_f + u2*h_cg) + C_sf*(g*l_r - u2*h_cg) 
    p3 = C_sr*(g*l_f + u2*h_cg)*l_r - C_sf*(g*l_r - u2*h_cg)*l_f

    if not use_slow:
        print("using fast!!!")
        A = np.array([[0, 0, 0, cos(x5+x7), -x4*sin(x5+x7), 0, -x4*sin(x5+x7)],
        [0, 0, 0, sin(x5+x7), x4*cos(x5+x7), 0, x4*cos(x5+x7)],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,1,0],
        [0,0,
        (l_f*C_sf*(g*l_r-u2*h_cg))*(mu*m)/(I_z*l_wb), 
        ((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f+u2*h_cg))*(mu*m)*x6/(I_z*l_wb*(x4**2)),
        0,
        -((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f+u2*h_cg))*(mu*m)/(I_z*l_wb*x4),
        (l_r*C_sr*(g*l_f + u2*h_cg) - l_f*C_sf*(g*l_r - u2*h_cg))*(mu*m)/(I_z*l_wb)],
        [0,0,
        (p1)*(mu)/(l_wb*x4),
        -((p1)*x3-(p2)*x7+(p3)*x6/x4)*(mu)/(l_wb*(x4**2)) - p3*(mu*x6)/(l_wb*(x4**3)),
        0,
        (p3)*(mu)/(l_wb*(x4**2))-1,
        -p2*(mu)/(l_wb*x4)]])

        B = np.array([[0,0],
        [0,0],
        [1, 0],
        [0, 1],
        [0, 0],
        [0, (-l_f*C_sf*h_cg*x3 + l_r*C_sr*h_cg*x7 + l_f*C_sf*h_cg*x7 + (l_f**2)*C_sf*h_cg*x6/x4 - (l_r**2)*C_sr*h_cg*x6/x4)*(mu*m)/(I_z*l_wb)],
        [0, (-C_sf*h_cg*x3 -C_sr*h_cg*x7 + C_sf*h_cg*x7 + l_r*C_sr*h_cg*x6/x4 + l_f*C_sf*h_cg*x6/x4)*(mu)/(l_wb*x4)]])

        const_term = [x4*cos(x5+x7),
        x4*sin(x5+x7),
        u1,
        u2,
        x6,
        (l_f*p1*x3 + (l_r*C_sr*(g*l_f + u2*h_cg) - l_f*C_sf*(g*l_r - u2*h_cg))*x7 - ((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f + u2*h_cg))*x6/x4)*mu*m/(I_z*l_wb),
        (p1*x3 - p2*x7 + p3*x6/x4)*mu/(x4*l_wb)-x6]

    else:
        # slow dynamics linearisation
        # hand-written notes in /Documents/Notes/Linearization_slow_dyn.pdf
        # reference to non linear model up in docstring

        const_term = [x4*cos(x5+x7),
        x4*sin(x5+x7),
        u1,
        u2,
        x4*cos(x7)*tan(x3)/l_wb,
        (1/l_wb)*(u2*cos(x7)*tan(x3)-x4*sin(x7)*tan(x3)*(l_r*u1/((1+(tan(x3*l_r/l_wb)**2))*l_wb*(cos(x3)**2))) + u1*x4*cos(x7)/(cos(x3)**2)),
        l_r*u1/((1+(tan(x3*l_r/l_wb)**2))*l_wb*(cos(x3)**2))]

        dx7dx3 = -(2*tan(x3)*(l_r**3)*u1)/(((1 + (tan(x3)*l_r/l_wb)**2)**2)*(l_wb**3)*(cos(x3)**2))+(2*u1*l_r*(sin(x3)))/((1 + (tan(x3)*l_r/l_wb)**2)*l_wb*(cos(x3)**3))

        A = np.array([[0, 0, 0, cos(x5+x7), -x4*sin(x5+x7), 0, -x4*sin(x5+x7)],
        [0, 0, 0, sin(x5+x7), x4*cos(x5+x7), 0, x4*cos(x5+x7)],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,x4*cos(x7)/(l_wb*(cos(x3)**2)),cos(x7)*tan(x3)/l_wb,0,0,-x4*tan(x3)*sin(x7)/l_wb],
        [0,0,(1/l_wb)*(u2*cos(x7)/(cos(x3)**2) -x4*sin(x7)*(const_term[Index.BETA]/(cos(x3)**2) + tan(x3)*dx7dx3) + x4*cos(x7)*u1*2*sin(x3)/(cos(x3)**3)),(1/l_wb)*(-sin(x7)*tan(x3)*const_term[Index.BETA] + cos(x7)*u1/(cos(x3)**2)),0,0,(1/l_wb)*(-u2*tan(x3)*sin(x7) - x4*tan(x3)*const_term[Index.BETA]*cos(x7) - u1*x4*sin(x7)/(cos(x3)**2))],
        [0,0,dx7dx3,0,0,0,0]])

        B = np.array([[0,0],
        [0,0],
        [1, 0],
        [0, 1],
        [0, 0],
        [(x4*cos(x7))/(l_wb*(cos(x3)**2)), (cos(x7) * tan(x3))/(l_wb)],
        [l_r/((1 + (tan(x3)*l_r/l_wb)**2)*l_wb*(cos(x3)**2)), 0]])

    const_term = np.array(const_term)
    const_term = const_term - np.dot(A, X[:7]) - np.dot(B, U)

    if np.isnan(A).any() or np.isnan(B).any() or np.isnan(const_term).any():
        raise ValueError("NAN IN SYS LINEARIZATION")

    #print("discr result")
    #print(A, B, const_term)
    return A, B, const_term

def discretise_dynamics(A_l, B_l, g_l, T_sampl):
    """
    Returns the Euler discretisation of the matrices of a linear system like 
    x_dot(t) = A_l*x(t) + B_l*u(t) + g_l
    """
    A_k = np.eye(len(A_l)) 
    A_k += T_sampl*A_l
    B_k = T_sampl * B_l
    g_k = T_sampl * g_l
    return A_k, B_k, g_k 

def linearise_error(X: List[float], track: SplineTrack, error_type: str):
    """
    Function that linearises the error function, basically e_c(theta) = e_c(theta_0) + (de_c/dtheta | theta_0) theta but matrices
    I took great inspiration from A. Liniger matlab code. 

    Args:
        X: the state at which we linearise, it is the state of the mpc formulation already so \in R^10

    Returns:
        Nabla_e_t0 : gradient
        ec_t0 : e_c(theta_0)
        error_type: can be either 'lag' or 'cont' (for contouring) 
    """

    x1 = X[Index.S_X]
    x2 = X[Index.S_Y]
    theta = X[Index.THETA]
    X_ref, Y_ref = track.get_coordinate(theta)
    dx_dth, dy_dth = track.get_derivative(theta)
    phi = track.get_angle(theta)
    dphi_dtheta = track.get_dphi_dtheta(theta)

    left = np.array([dphi_dtheta, 1])
    right = np.array([cos(phi), sin(phi)]).reshape((2,))

    if error_type == 'cont':
        MC = np.array([[x1-X_ref, x2-Y_ref], [dy_dth, -dx_dth]])
        Nabla_e_t0 = np.array([sin(phi), -cos(phi),0,0,0,0,0,np.matmul(np.matmul(left, MC), right) ,0,0]).reshape((10,1)) 
        ec_t0 = sin(phi)*(x1-X_ref) - cos(phi) * (x2 - Y_ref)
    elif error_type == 'lag':
        ML = np.array([[-x2+Y_ref, x1-X_ref ], [dx_dth, dy_dth]])
        Nabla_e_t0 = np.array([-cos(phi), -sin(phi),0,0,0,0,0, np.matmul(np.matmul(left, ML), right) ,0,0]).reshape((10,1)) 
        ec_t0 = -cos(phi)*(x1-X_ref) - sin(phi) * (x2 - Y_ref)

    if np.isnan(Nabla_e_t0).any() or np.isnan(ec_t0).any():
        raise ValueError("NAns in error linearisation!!!!!")

    return Nabla_e_t0, ec_t0

def get_track_constraints(track: SplineTrack, theta: float):
    """
    Returns the 10-d matrices corresponding to the track limits for a given theta
    """

    int_point = np.array(track.get_coordinate(theta, line='int'))
    out_point = np.array(track.get_coordinate(theta, line='out'))
    slope_ang = track.get_angle(theta)

    norm_vec = np.array([cos(slope_ang- np.pi/2), sin(slope_ang - np.pi/2)])
    
    q_int = np.dot(int_point, norm_vec)
    q_out = np.dot(out_point, norm_vec)
	
    if q_out <=q_int:
        # if the outer point of the track is on this side of the half-plane
        # then the inequality for the interior point must include it

        F_int = np.pad(norm_vec, (0, 8), 'constant')
        f_int = np.pad(q_int, (0, 8), 'constant')

        F_out = np.pad(-norm_vec, (0, 8), 'constant')
        f_out = np.pad(-q_out, (0, 8), 'constant')
    else:
        F_int = np.pad(-norm_vec, (0, 8), 'constant')
        f_int = np.pad(-q_int, (0, 8), 'constant')

        F_out = np.pad(norm_vec, (0, 8), 'constant')
        f_out = np.pad(q_out, (0, 8), 'constant')

    F = np.concatenate(([F_int], [F_out]), axis=0)
    f = np.concatenate(([f_int], [f_out]), axis=0)

    return F, f
        
def get_MPC_matrices(X, U, Z_hist, MPC_params, track:SplineTrack, track_hard: SplineTrack, car_params):
    """
    Generates the matrices of the MPC problem already expressed as QP

    Args:
        X: current state, 10-D
        U: previous input at current timestep
        Z_hist: previous solution of future timesteps, Nx12 dimensional 
    """

    n_x = MPC_params['n_x']
    n_u = MPC_params['n_u']
    n_z = n_x+n_u
    N = MPC_params['N']
    gamma = MPC_params['gamma']
    T_s = MPC_params['T_s']
    nu_soft = MPC_params['nu_soft']
    R = MPC_params['R']

    # check slower velocity, to decide which model to use 
    vels = np.concatenate((np.array([X[Index.V]]), Z_hist[:, Index.V]))
    thresh_func = lambda x : x < VEL_THRESH_SLOW_DYN
    use_slow = thresh_func(vels).any()
    max_vel = np.max(vels)

    P = np.zeros((n_z*(N+1), n_z*(N+1)))
    q = np.zeros((n_z*(N+1), 1))

    # initial setup of constraints, with x_0 and theta_0
    A = np.zeros((n_x, n_z*(N+1)))
    u = X[:n_x]
    l = X[:n_x]
    
    # just track constraint, for debugging
    F_constr = np.zeros((1, 2))
    f_constr = np.array([0])

    id_mat = np.eye(n_x)

    for i, row in enumerate(id_mat):
        A[i,:] = np.pad(row, (0, n_z*(N)+n_u), 'constant')

    q_c = MPC_params['q_c']
    q_l = MPC_params['q_l']
    for k in range(N):
        # COST FUNCTION
        # error approximations
        if DEBUG["error_approx"]:
            E_c, e_c = linearise_error(Z_hist[k, :n_x], track, 'cont')
            E_l, e_l = linearise_error(Z_hist[k, :n_x], track, 'lag')
            P[n_z*(k+1):n_z*(k+1) + n_x, n_z*(k+1):n_z*(k+1)+n_x] = q_c*E_c*E_c.T + q_l*E_l*E_l.T
            tmp_prod_c = np.dot(Z_hist[k, :n_x], E_c)
            tmp_prod_l = np.dot(Z_hist[k, :n_x], E_l)
            if k == N:
            	q_c *= 10
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
            matr = np.matmul(np.matmul(mod, R), mod.T)
            P[k*n_z:(k+2)*n_z, k*n_z:(k+2)*n_z] += matr

        # soft constraints
        soft_mult = 1  + max_vel*1e2
        q[(k+1)*n_z+Index.SOFT] = soft_mult*nu_soft 


        # INEQUALITY CONSTRAINTS
        # track constraints
        if DEBUG['track_constr']:
            F, f = get_track_constraints(track, Z_hist[k,7])
            
            A_row = np.pad(F[0,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            A_row[(k+1)*n_z+Index.SOFT] = -1
            A = np.append(A, [A_row], axis=0)

            A_row = np.pad(F[1,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            A_row[(k+1)*n_z+Index.SOFT] = -1
            A = np.append(A, [A_row], axis=0)

            u = np.append(u, f, axis=0)
            l = np.append(l, -np.inf*np.ones(shape=f.shape), axis=0)

        if DEBUG['hard_track_constr']:
            F, f = get_track_constraints(track_hard, Z_hist[k,7])

            F_constr = np.append(F_constr, F[:, :2], axis=0)
            f_constr = np.append(f_constr, f, axis=0)
            
            A_row = np.pad(F[0,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            A = np.append(A, [A_row], axis=0)


            A_row = np.pad(F[1,:], ((k+1)*n_z, (N-1-k)*n_z + n_u), 'constant')
            A = np.append(A, [A_row], axis=0)
            

            u = np.append(u, f, axis=0)
            l = np.append(l, -np.inf*np.ones(shape=f.shape), axis=0)


        # EQUALITY CONSTRAINTS
        # theta dynamic
        if DEBUG['theta_dyn']:
            A_row = np.zeros((n_z*(N+1), 1))
            
            A_row[n_z*(k+1)+Index.THETA] = 1
            A_row[n_z*k+Index.THETA] = -1 
            if DEBUG["orig_liniger"]:
                A_row[n_z*k+Index.V_K] = -T_s 
            else: 
                A_row[n_z*k+Index.V] = -T_s # this part is different from the original paper
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [0], axis=0)
            l = np.append(l, [0], axis=0)

        # dynamic
        if k == 0:
            A_l, B_l, g_l = linearise_dynamics(X[:7], U, car_params, use_slow)
        else:
            A_l, B_l, g_l = linearise_dynamics(Z_hist[k-1,:7], Z_hist[k-1,n_x:], car_params, use_slow)

        A_k, B_k, g_k = discretise_dynamics(A_l, B_l, g_l, T_s)
            
        if DEBUG['sys_dyn']: 
            for i, row in enumerate(A_k):
                A_row = np.zeros((n_z*(N+1), 1))

                A_row[(k+1)*n_z+i] = 1
                A_row[k*n_z:k*n_z+7] = -row.reshape((7, 1))
                A_row[k*n_z+n_x:(k+1)*n_z] = -B_k[i,:].reshape((2,1))
                A = np.append(A, A_row.T, axis=0)
                u = np.append(u, [g_k[i]], axis=0)
                l = np.append(l, [g_k[i]], axis=0)


        # general boundaries on states
        if DEBUG['general_constr']:
            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.S_X] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [500], axis=0)
            l = np.append(l, [-500], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.S_Y] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [500], axis=0)
            l = np.append(l, [-500], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.DELTA] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [0.4189], axis=0)
            l = np.append(l, [-0.4189], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.V] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [15], axis=0)
            l = np.append(l, [-5], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.PHI] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [2*np.pi], axis=0)
            l = np.append(l, [-2*np.pi], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.PHI_DOT] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [3.2], axis=0)
            l = np.append(l, [-3.2], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.BETA] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [2*np.pi], axis=0)
            l = np.append(l, [-2*np.pi], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.THETA] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [5*track.track_length], axis=0)
            l = np.append(l, [-track.track_length], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.V_K] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [15], axis=0)
            l = np.append(l, [-5], axis=0)

            # soft constr positivity
            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.SOFT] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [np.inf], axis=0)
            l = np.append(l, [0], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.ACC] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [9.51], axis=0)
            l = np.append(l, [-9.51], axis=0)

            A_row = np.zeros((n_z*(N+1), 1))
            A_row[n_z*k+Index.V_DELTA] = 1
            A = np.append(A, A_row.T, axis=0)
            u = np.append(u, [3.2], axis=0)
            l = np.append(l, [-3.2], axis=0)

    # general boundaries on states, at final timestep
    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.S_X] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [500], axis=0)
    l = np.append(l, [-500], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.S_Y] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [500], axis=0)
    l = np.append(l, [-500], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.DELTA] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [0.4189], axis=0)
    l = np.append(l, [-0.4189], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.V] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [15], axis=0)
    l = np.append(l, [-5], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.PHI] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [2*np.pi], axis=0)
    l = np.append(l, [-2*np.pi], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.PHI_DOT] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [3.2], axis=0)
    l = np.append(l, [-3.2], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.BETA] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [2*np.pi], axis=0)
    l = np.append(l, [-2*np.pi], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.THETA] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [5*track.track_length], axis=0)
    l = np.append(l, [-track.track_length], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.V_K] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [15], axis=0)
    l = np.append(l, [-5], axis=0)

    # soft constr positivity
    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.SOFT] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [np.inf], axis=0)
    l = np.append(l, [0], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.ACC] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [9.51], axis=0)
    l = np.append(l, [-9.51], axis=0)

    A_row = np.zeros((n_z*(N+1), 1))
    A_row[n_z*N+Index.V_DELTA] = 1
    A = np.append(A, A_row.T, axis=0)
    u = np.append(u, [3.2], axis=0)
    l = np.append(l, [-3.2], axis=0)

    # COST FUNCTION
    # advancement part
    if DEBUG['orig_liniger']:
        q[n_z*N+Index.V_K] += -gamma*T_s
    else:
        q[n_z*N+Index.V] += -gamma*T_s

    # sparsification
    P = sparse.csc_matrix(0.5*(P+P.T)) 
    A = sparse.csc_matrix(A)
    
    F_constr = F_constr[1:, :]
    f_constr = f_constr[1:]

    return P, q, A, l, u, F_constr, f_constr

def get_initial_warmstart(N:int, v_x0:float, MPC_params, track:SplineTrack):
    """
    Obtains the initial warmstart. Similarly as in the MATLAB implementation
    by A. Linigier, a constant velocity is assumed 
    """

    in_coord = np.array(MPC_params['start_point'][:2])
    theta = find_theta(in_coord, track, 0)
    x = np.array([track.get_coordinate(theta)[0], track.get_coordinate(theta)[1], 0, v_x0, MPC_params['start_point'][2], 0, 0, theta, v_x0, 0, 0, 0])
    for k in range(MPC_params['N']):
        theta += v_x0*MPC_params['T_s']
        new_x = [track.get_coordinate(theta)[0], 
        track.get_coordinate(theta)[1], 
        0,
        v_x0,
        arctan(track.get_derivative(theta)[0]/track.get_derivative(theta)[1]),
        0,
        0,
        theta, 
        v_x0,
        0,
        0,
        0]
        new_x = np.array(new_x)
        x = np.append(x, new_x, axis=None)

    #print(",".join(x[Index.S_X::12].astype(str)))
    #print(",".join(x[Index.S_Y::12].astype(str)))

    return x

def find_theta(coord: np.ndarray, track: SplineTrack, theta_est, eps: float = 0.01):
    """
    Find the parameter of the track nearest to the point given an approximation of it.

    Args:
        coord: coordinate of point projected around, as np.array
        track: splinified track 
        theta_est: best guess at theta
        eps: precision at which the algorithm looks around for theta
    """

    found = False
    max_dist = track.track_length/20

    dist = np.linalg.norm(coord - np.array(track.get_coordinate(theta_est)))
    min_dist = dist
    min_theta = theta_est
    theta_i = theta_est

    while not found:
        theta_i += eps
        dist = np.linalg.norm(coord - np.array(track.get_coordinate(theta_i)))

        if dist <= min_dist:
            min_dist = dist
            min_theta = theta_i
        else:
            found = True

        if theta_i - theta_est >= max_dist:
            found = True

        #print(theta_i)
    
    # now try to look backwards
    found = False
    theta_i = theta_est
    while not found:
        theta_i -= eps

        dist = np.linalg.norm(coord - np.array(track.get_coordinate(theta_i)))

        if dist <= min_dist:
            min_dist = dist
            min_theta = theta_i
        else:
            found = True

        if theta_est - theta_i >= max_dist:
            found = True
        
        #print(theta_i)

    return min_theta

def find_theta_slow(coord: np.ndarray, track: SplineTrack, eps: float = 0.1):
    """
    Find the parameter of the track nearest to the point without an approximation of it.
    Could be slower

    Args:
        coord: coordinate of point projected around, as np.array
        track: splinified track 
        eps: precision at which the algorithm looks around for theta
    """

    found = False
    dist = np.linalg.norm(coord - np.array(track.get_coordinate(0)))
    
    min_dist = dist
    min_theta = 0
    theta_i = 0

    while theta_i <= track.track_length:
        theta_i += eps
        dist = np.linalg.norm(coord - np.array(track.get_coordinate(theta_i)))

        if dist <= min_dist:
            min_dist = dist
            min_theta = theta_i

        #print(theta_i)

    return min_theta    

def update_warm_start(prev_result, MPC_params, car_params):
    """
    Updates the warm start used in solving the MPC.

    Args: 
        prev_result: it's the result coming from osqp, primal solution is in prev_result.x
        warmstart: a N*n_z vector

    """

    if True:
        x = prev_result.x 
    else:
        x = prev_result
    #print("X")
    #print(x)
    n_z = MPC_params['n_z']
    n_x = MPC_params['n_x']
    n_u = MPC_params['n_u']
    T_s = MPC_params['T_s']
    new_warm_start = x[n_z:]

    #print("trajectory")
    #print(",".join(new_warm_start[Index.S_X::12].astype(str)))
    #print(",".join(new_warm_start[Index.S_Y::12].astype(str)))
    #raise ValueError()

    new_warm_start[-n_u:] = x[-2*n_z+n_x:-2*n_z+n_x+n_u] # adjust input of previous last step, since last one is not really useful

    new_last_state = dynamics(new_warm_start[-n_z:-n_z+n_x], new_warm_start[-n_u:], car_params, T_s) 
    n_st = len(new_last_state)

    new_warm_start = np.append(new_warm_start, new_last_state, axis = None)
    # my idea
    if DEBUG['orig_liniger']:
        new_warm_start = np.append(new_warm_start, np.array([x[-n_z+Index.THETA]+x[-2*n_z+Index.V]/MPC_params['T_s'], 0, 0]), axis = None)
    else:
        new_warm_start = np.append(new_warm_start, np.array([x[-n_z+Index.THETA]+x[-n_z+Index.V]*MPC_params['T_s'], new_last_state[Index.V], 0]), axis = None)
    
    new_warm_start = np.append(new_warm_start, np.array(np.zeros((n_u))), axis = None) 

    if np.isnan(new_warm_start).any():
        raise ValueError("Some nannnnnnssss")

    return new_warm_start

def dynamics(X, U, car_params, T_s):
    """
    Simulates the dynamics, with the full model

    Args:
        X: 10-D
        U: 2-D

    Returns:
        nxt_st: the 7-D next state
    """
    x1 = X[Index.S_X]
    x2 = X[Index.S_Y]
    x3 = X[Index.DELTA]
    x4 = X[Index.V]
    x5 = X[Index.PHI]
    x6 = X[Index.PHI_DOT]
    x7 = X[Index.BETA]

    u1 = U[0]
    u2 = U[1]

    l_r = car_params['lr']
    l_f = car_params['lf']
    l_wb = car_params['lr'] + car_params['lf']
    mu = car_params['mu']
    C_sf = car_params['C_sf']
    C_sr = car_params['C_sr']
    I_z = car_params['I']
    m = car_params['m']
    h_cg = car_params['h']
    g = 9.81

    p1 = C_sf*(g*l_r - u2*h_cg)
    p2 = C_sr*(g*l_f + u2*h_cg) + C_sf*(g*l_r - u2*h_cg) 
    p3 = C_sr*(g*l_f + u2*h_cg)*l_r - C_sf*(g*l_r - u2*h_cg)*l_f

    if x4 >= VEL_THRESH_SLOW_DYN:
        next_st = np.array([x4*cos(x5+x7),
            x4*sin(x5+x7),
            u1,
            u2,
            x6,
            (l_f*p1*x3 + (l_r*C_sr*(g*l_f + u2*h_cg) - l_f*C_sf*(g*l_r - u2*h_cg))*x7 - ((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f + u2*h_cg))*x6/x4)*mu*m/(I_z*l_wb),
            (p1*x3 - p2*x7 + p3*x6/x4)*mu/(x4*l_wb)-x6])
    else:
        next_st = np.array([x4*cos(x5+x7),
            x4*sin(x5+x7),
            u1,
            u2,
            x4*cos(x7)*tan(x3)/l_wb,
            (1/l_wb)*(u2*cos(x7)*tan(x3)-x4*sin(x7)*tan(x3)*(l_r*u1/((1+(tan(x3*l_r/l_wb)**2))*l_wb*(cos(x3)**2))) + u1*x4*cos(x7)/(cos(x3)**2)),
            l_r*u1/((1+(tan(x3*l_r/l_wb)**2))*l_wb*(cos(x3)**2))])

    # actually an approximation
    next_st = X[:7] + next_st*T_s    

    return next_st

def get_inputs(acc:float, steer_vel:float, cur_steer:float, cur_vel:float, max_a, max_v, min_v):
    """
    Obtains the proper inputs to feed into the gym environment
    """

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

import pathlib
from typing import List

import yaml
import numpy as np
from numba import jit
from numba.core import types
from numba.typed import Dict
from numpy import arctan, sin, cos, tan
from scipy import sparse

from splinify.splinify import SplineTrack


cur_dir = str(pathlib.Path(__file__).parent.resolve())
config_dir = cur_dir + '/../config/'

with open(config_dir+"car_params.yaml", "r") as conf:
    car_params = yaml.safe_load(conf)
    car_params_nb = Dict.empty(
        key_type = types.unicode_type,
        value_type = types.float64
    )
    for k, v in car_params.items():
        car_params_nb[k] = v


@jit(nopython=True)
def dynamics(X: np.array, U: np.array, T_s: float, use_slow: bool):
    """
    Simulates the dynamics, with the full model

    Args:
        X: 9-D
        U: 2-D

    Returns:
        nxt_st: the 7-D next state
    """
    x3 = X[2]
    x4 = X[3]
    x5 = X[4]
    x6 = X[5]
    x7 = X[6]

    u1 = U[0]
    u2 = U[1]

    l_r = 0.17145
    l_f = 0.15875
    l_wb = l_r+l_f
    mu = 1.0489
    C_sf = 4.718
    C_sr = 5.4562
    I_z = 0.04712
    m = 3.74
    h_cg = 0.074
    g = 9.81
    

    p1 = C_sf*(g*l_r - u2*h_cg)
    p2 = C_sr*(g*l_f + u2*h_cg) + C_sf*(g*l_r - u2*h_cg) 
    p3 = C_sr*(g*l_f + u2*h_cg)*l_r - C_sf*(g*l_r - u2*h_cg)*l_f

    if not use_slow:
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

@jit(nopython=True)
def linearise_dynamics(X: np.array, U: np.array, use_slow: bool):
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

    x3 = X[2]
    x4 = X[3]
    x5 = X[4]
    x6 = X[5]
    x7 = X[6]

    u1 = U[0]
    u2 = U[1]

    l_r = 0.17145
    l_f = 0.15875
    l_wb = l_r+l_f
    mu = 1.0489
    C_sf = 4.718
    C_sr = 5.4562
    I_z = 0.04712
    m = 3.74
    h_cg = 0.074
    g = 9.81


    p1 = C_sf*(g*l_r - u2*h_cg)
    p2 = C_sr*(g*l_f + u2*h_cg) + C_sf*(g*l_r - u2*h_cg) 
    p3 = C_sr*(g*l_f + u2*h_cg)*l_r - C_sf*(g*l_r - u2*h_cg)*l_f

    if not use_slow:
        A = np.array([[0.0, 0, 0, cos(x5+x7), -x4*sin(x5+x7), 0, -x4*sin(x5+x7)],
        [0.0, 0, 0, sin(x5+x7), x4*cos(x5+x7), 0, x4*cos(x5+x7)],
        [0.0,0,0,0,0,0,0],
        [0.0,0,0,0,0,0,0],
        [0.0,0,0,0,0,1,0],
        [0.0,0,
        (l_f*C_sf*(g*l_r-u2*h_cg))*(mu*m)/(I_z*l_wb), 
        ((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f+u2*h_cg))*(mu*m)*x6/(I_z*l_wb*(x4**2)),
        0,
        -((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f+u2*h_cg))*(mu*m)/(I_z*l_wb*x4),
        (l_r*C_sr*(g*l_f + u2*h_cg) - l_f*C_sf*(g*l_r - u2*h_cg))*(mu*m)/(I_z*l_wb)],
        [0.0,0,
        (p1)*(mu)/(l_wb*x4),
        -((p1)*x3-(p2)*x7+(p3)*x6/x4)*(mu)/(l_wb*(x4**2)) - p3*(mu*x6)/(l_wb*(x4**3)),
        0,
        (p3)*(mu)/(l_wb*(x4**2))-1,
        -p2*(mu)/(l_wb*x4)]])

        B = np.array([[0.0,0],
        [0.0,0],
        [1, 0.0],
        [0.0, 1],
        [0.0, 0],
        [0.0, (-l_f*C_sf*h_cg*x3 + l_r*C_sr*h_cg*x7 + l_f*C_sf*h_cg*x7 + (l_f**2)*C_sf*h_cg*x6/x4 - (l_r**2)*C_sr*h_cg*x6/x4)*(mu*m)/(I_z*l_wb)],
        [0.0, (-C_sf*h_cg*x3 -C_sr*h_cg*x7 + C_sf*h_cg*x7 + l_r*C_sr*h_cg*x6/x4 + l_f*C_sf*h_cg*x6/x4)*(mu)/(l_wb*x4)]])

        const_term = np.array([x4*cos(x5+x7),
        x4*sin(x5+x7),
        u1,
        u2,
        x6,
        (l_f*p1*x3 + (l_r*C_sr*(g*l_f + u2*h_cg) - l_f*C_sf*(g*l_r - u2*h_cg))*x7 - ((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f + u2*h_cg))*x6/x4)*mu*m/(I_z*l_wb),
        (p1*x3 - p2*x7 + p3*x6/x4)*mu/(x4*l_wb)-x6])


    else:
        # slow dynamics linearisation
        # hand-written notes in /Documents/Notes/Linearization_slow_dyn.pdf
        # reference to non linear model up in docstring

        const_term = np.array([x4*cos(x5+x7),
        x4*sin(x5+x7),
        u1,
        u2,
        x4*cos(x7)*tan(x3)/l_wb,
        (1/l_wb)*(u2*cos(x7)*tan(x3)-x4*sin(x7)*tan(x3)*(l_r*u1/((1+(tan(x3*l_r/l_wb)**2))*l_wb*(cos(x3)**2))) + u1*x4*cos(x7)/(cos(x3)**2)),
        l_r*u1/((1+(tan(x3*l_r/l_wb)**2))*l_wb*(cos(x3)**2))])

        dx7dx3 = -(2*tan(x3)*(l_r**3)*u1)/(((1 + (tan(x3)*l_r/l_wb)**2)**2)*(l_wb**3)*(cos(x3)**2))+(2*u1*l_r*(sin(x3)))/((1 + (tan(x3)*l_r/l_wb)**2)*l_wb*(cos(x3)**3))

        A = np.array([[0.0, 0, 0, cos(x5+x7), -x4*sin(x5+x7), 0, -x4*sin(x5+x7)],
        [0.0, 0, 0, sin(x5+x7), x4*cos(x5+x7), 0, x4*cos(x5+x7)],
        [0.0,0,0,0,0,0,0],
        [0.0,0,0,0,0,0,0],
        [0.0,0,x4*cos(x7)/(l_wb*(cos(x3)**2)),cos(x7)*tan(x3)/l_wb,0,0,-x4*tan(x3)*sin(x7)/l_wb],
        [0.0,0,(1/l_wb)*(u2*cos(x7)/(cos(x3)**2) -x4*sin(x7)*(const_term[6]/(cos(x3)**2) + tan(x3)*dx7dx3) + x4*cos(x7)*u1*2*sin(x3)/(cos(x3)**3)),(1/l_wb)*(-sin(x7)*tan(x3)*const_term[6] + cos(x7)*u1/(cos(x3)**2)),0,0,(1/l_wb)*(-u2*tan(x3)*sin(x7) - x4*tan(x3)*const_term[6]*cos(x7) - u1*x4*sin(x7)/(cos(x3)**2))],
        [0.0,0,dx7dx3,0,0,0,0]])

        B = np.array([[0.0,0],
        [0.0,0],
        [1, 0.0],
        [0.0, 1],
        [0.0, 0],
        [(x4*cos(x7))/(l_wb*(cos(x3)**2)), (cos(x7) * tan(x3))/(l_wb)],
        [l_r/((1 + (tan(x3)*l_r/l_wb)**2)*l_wb*(cos(x3)**2)), 0.0]])

    const_term = const_term - np.dot(A, X[:7]) - np.dot(B, U)

    return A, B, const_term

@jit(nopython=True)
def discretise_dynamics(
    A_l: np.array, 
    B_l: np.array, 
    g_l: np.array, 
    T_sampl: float,
    ):
    """
    Returns the Euler discretisation of the matrices of a linear system like 
    x_dot(t) = A_l*x(t) + B_l*u(t) + g_l
    """
    A_k = np.eye(len(A_l)) 
    A_k += T_sampl*A_l
    B_k = T_sampl * B_l
    g_k = T_sampl * g_l
    return A_k, B_k, g_k 

@jit(nopython=True)
def linearise_contouring_error(
    x1: np.float64, 
    x2: np.float64, 
    X_ref: np.float64, 
    Y_ref: np.float64,
    phi: np.float64,
    dphi_dtheta: np.float64,
    dx_dth: np.float64,
    dy_dth: np.float64,
    n_x: np.int8
    ):
    """
    Linearise the contouring error defined in A.Linieger's paper
    'Optimization-Based Autonomous Racing of 1:43 Scale RC Cars'
    """

    dec_dth = (x1-X_ref)*cos(phi)*dphi_dtheta \
        + (-dx_dth)*sin(phi) \
        + (x2-Y_ref)*sin(phi)*dphi_dtheta \
        + (-cos(phi))*(-dy_dth)

    Nabla_e_t0 = np.zeros((n_x,1))
    Nabla_e_t0[0] = sin(phi)
    Nabla_e_t0[1] = -cos(phi)
    Nabla_e_t0[7] = dec_dth

    ec_t0 = sin(phi)*(x1-X_ref) - cos(phi) * (x2 - Y_ref)

    return Nabla_e_t0, ec_t0

@jit(nopython=True)
def linearise_lag_error(
    x1: np.float64, 
    x2: np.float64, 
    X_ref: np.float64, 
    Y_ref: np.float64,
    phi: np.float64,
    dphi_dtheta: np.float64,
    dx_dth: np.float64,
    dy_dth: np.float64,
    n_x: np.int8
    ):
    """
    Linearise the lag error defined in A.Liniger's paper
    'Optimization-Based Autonomous Racing of 1:43 Scale RC Cars'
    """

    del_dth = (x1-X_ref)*sin(phi)*dphi_dtheta \
        + (-dx_dth)*(-cos(phi)) \
        + (x2-Y_ref)*(-cos(phi))*dphi_dtheta \
        + (-sin(phi))*(-dy_dth)

    Nabla_e_t0 = np.zeros((n_x,1))
    Nabla_e_t0[0] = -cos(phi)
    Nabla_e_t0[1] = -sin(phi)
    Nabla_e_t0[7] = del_dth

    ec_t0 = -cos(phi)*(x1-X_ref) - sin(phi) * (x2 - Y_ref)

    return Nabla_e_t0, ec_t0

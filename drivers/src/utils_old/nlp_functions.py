import yaml
import index

import casadi as cs
from casadi import Function, SX

Index = index.NlIndex
ParInd = index.ParameterIndex

with open('/home/invitedguest/edoardo-ghignone/drivers/src/config/MPC_params.yaml', 'r') as conf:
    MPC_params = yaml.safe_load(conf)

with open('/home/invitedguest/edoardo-ghignone/drivers/src/config/car_params.yaml', 'r') as conf:
    car_params = yaml.safe_load(conf)

n_par = len(ParInd)


def eval_obj():
    """
    Nonlinear cost function
    """
    z = SX.sym('z', MPC_params['n_z'])
    p = SX.sym('p', n_par) 

    phi_appr = p[ParInd.PHITH] #+ (z[Index.THETA]-p[ParInd.THETA])*p[ParInd.dPHI]
    x_appr = p[ParInd.XTH] #+ (z[Index.THETA]-p[ParInd.THETA])*p[ParInd.dX]
    y_appr = p[ParInd.YTH] #+ (z[Index.THETA]-p[ParInd.THETA])*p[ParInd.dY]

    advancement = -z[Index.V_K]*MPC_params['T_s']*MPC_params['gamma']
    contouring = cs.sin(phi_appr)*(z[Index.S_X] - x_appr) - cs.cos(phi_appr)*(z[Index.S_Y] - y_appr)
    lag = -cs.cos(phi_appr)*(z[Index.S_X] - x_appr) - cs.sin(phi_appr)*(z[Index.S_Y] - y_appr)
    soft_cost = MPC_params['nu_soft']*z[Index.SOFT] + 0.1*MPC_params['nu_soft']*z[Index.SOFT]**2

    f = Function('eval_obj', [z, p], [MPC_params['q_c']*contouring**2 + MPC_params['q_l']*lag**2 + advancement + soft_cost])
    return f

def eval_dynamics_slow():

    z = SX.sym('z', MPC_params['n_z'])
    p = SX.sym('p', n_par)

    x1 = z[Index.S_X]
    x2 = z[Index.S_Y]
    x3 = z[Index.DELTA]
    x4 = z[Index.V]
    x5 = z[Index.PHI]
    x6 = z[Index.PHI_DOT]
    x7 = z[Index.BETA]

    u1 = z[Index.V_DELTA]
    u2 = z[Index.ACC]

    l_r = car_params['lr']
    l_wb = car_params['lr'] + car_params['lf']
    T_s = MPC_params['T_s']
    
    next_st = [x1 + T_s*(x4*cs.cos(x5+x7)),
            x2 + T_s*(x4*cs.sin(x5+x7)),
            x3 + T_s*(u1),
            x4 + T_s*(u2),
            x5 + T_s*(x4*cs.cos(x7)*cs.tan(x3)/l_wb),
            x6 + T_s*((1/l_wb)*(u2*cs.cos(x7)*cs.tan(x3)-x4*cs.sin(x7)*cs.tan(x3)*(l_r*u1/((1+(cs.tan(x3*l_r/l_wb)**2))*l_wb*(cs.cos(x3)**2))) + u1*x4*cs.cos(x7)/(cs.cos(x3)**2))),
            x7 + T_s*(l_r*u1/((1+(cs.tan(x3*l_r/l_wb)**2))*l_wb*(cs.cos(x3)**2))),
            z[Index.THETA] + T_s*z[Index.V_K]]


    # actually an approximation    
    f = Function('eval_dynamics', [z, p], next_st)

    return f

def eval_dynamics_fast():

    z = SX.sym('z', MPC_params['n_z'])
    p = SX.sym('p', n_par)

    x1 = z[Index.S_X]
    x2 = z[Index.S_Y]
    x3 = z[Index.DELTA]
    x4 = z[Index.V]
    x5 = z[Index.PHI]
    x6 = z[Index.PHI_DOT]
    x7 = z[Index.BETA]

    u1 = z[Index.V_DELTA]
    u2 = z[Index.ACC]

    l_r = car_params['lr']
    l_f = car_params['lf']
    l_wb = car_params['lr'] + car_params['lf']
    mu = car_params['mu']
    C_sf = car_params['C_sf']
    C_sr = car_params['C_sr']
    I_z = car_params['I']
    m = car_params['m']
    h_cg = car_params['h']
    T_s = MPC_params['T_s']
    g = 9.81

    p1 = C_sf*(g*l_r - u2*h_cg)
    p2 = C_sr*(g*l_f + u2*h_cg) + C_sf*(g*l_r - u2*h_cg) 
    p3 = C_sr*(g*l_f + u2*h_cg)*l_r - C_sf*(g*l_r - u2*h_cg)*l_f

    
    next_st = [x1 + T_s*(x4*cs.cos(x5+x7)),
            x2 + T_s*(x4*cs.sin(x5+x7)),
            x3 + T_s*(u1),
            x4 + T_s*(u2),
            x5 + T_s*(x6),
            x6 + T_s*((l_f*p1*x3 + (l_r*C_sr*(g*l_f + u2*h_cg) - l_f*C_sf*(g*l_r - u2*h_cg))*x7 - ((l_f**2)*C_sf*(g*l_r - u2*h_cg) + (l_r**2)*C_sr*(g*l_f + u2*h_cg))*x6/x4)*mu*m/(I_z*l_wb)),
            x7 + T_s*((p1*x3 - p2*x7 + p3*x6/x4)*mu/(x4*l_wb)-x6),
            z[Index.THETA] + T_s*z[Index.V_K]]

    # actually an approximation    
    f = Function('eval_dynamics', [z, p], next_st)

    return f

def eval_const():
    z = SX.sym('z', MPC_params['n_z'])
    p = SX.sym('p', n_par)

    const_one = z[Index.S_X]*p[ParInd.FXINT] + z[Index.S_Y]*p[ParInd.FYINT] - p[ParInd.fINT] - z[Index.SOFT]
    const_two = z[Index.S_X]*p[ParInd.FXOUT] + z[Index.S_Y]*p[ParInd.FYOUT] - p[ParInd.fOUT] - z[Index.SOFT]
    const_one_hard = z[Index.S_X]*p[ParInd.FXINT] + z[Index.S_Y]*p[ParInd.FYINT] - p[ParInd.fINTHARD]
    const_two_hard = z[Index.S_X]*p[ParInd.FXOUT] + z[Index.S_Y]*p[ParInd.FYOUT] - p[ParInd.fOUTHARD]

    f = Function('eval_const', [z, p], [const_one, const_two, const_one_hard, const_two_hard])

    return f

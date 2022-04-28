import yaml
import pathlib
import numpy as np

from splinify.splinify import SplineTrack

from index import ParameterIndex, Index, NlIndex
import MPC_utils


cur_dir = str(pathlib.Path(__file__).parent.resolve())
config_dir = cur_dir + '/../config/'

with open(config_dir+"MPC_params.yaml", "r") as conf:
    MPC_params = yaml.safe_load(conf) 


def get_nlp_forces_params(warm_start, track: SplineTrack, track_hard: SplineTrack):
    """
    Obtains the parameters for the nlp solver generated with forces. 
    The parameters are, for each timestep: 
      - first six define the track contraints
      - then coordinates and angle corresponding to a theta
      - then a flag that decides whether to use the fast model or not
    """
    
    n_z = MPC_params['n_z']
    n_par = len(ParameterIndex)
    N = MPC_params['N']
    
    all_params=np.empty((N+1)*n_par)

    for i in range(N+1):
        if i==0:
            F, f = MPC_utils.get_track_constraints(track, warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.FXINT] = F[0,Index.S_X]
            all_params[i*n_par+ParameterIndex.FYINT] = F[0,Index.S_Y]
            all_params[i*n_par+ParameterIndex.fINT] = f[0]
            all_params[i*n_par+ParameterIndex.FXOUT] = F[1,Index.S_X]
            all_params[i*n_par+ParameterIndex.FYOUT] = F[1, Index.S_Y]
            all_params[i*n_par+ParameterIndex.fOUT] = f[1]

            _, f_hard = MPC_utils.get_track_constraints(track_hard, warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.fINTHARD] = f_hard[0]
            all_params[i*n_par+ParameterIndex.fOUTHARD] = f_hard[1]

            coords_th = track.get_coordinate(warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.XTH] = coords_th[0]
            all_params[i*n_par+ParameterIndex.YTH] = coords_th[1]

            all_params[i*n_par+ParameterIndex.PHITH] = track.get_angle(warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.dPHI] = track.get_dphi_dtheta(warm_start[i*n_z+Index.THETA])
            dx, dy = track.get_derivative(warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.dX] = dx
            all_params[i*n_par+ParameterIndex.dY] = dy

            all_params[i*n_par+ParameterIndex.dY] = warm_start[i*n_z+Index.THETA]
            
        else:
            F, f = MPC_utils.get_track_constraints(track, warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.FXINT] = F[0,Index.S_X]
            all_params[i*n_par+ParameterIndex.FYINT] = F[0,Index.S_Y]
            all_params[i*n_par+ParameterIndex.fINT] = f[0]
            all_params[i*n_par+ParameterIndex.FXOUT] = F[1,Index.S_X]
            all_params[i*n_par+ParameterIndex.FYOUT] = F[1, Index.S_Y]
            all_params[i*n_par+ParameterIndex.fOUT] = f[1]

            _, f_hard = MPC_utils.get_track_constraints(track_hard, warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.fINTHARD] = f_hard[0]
            all_params[i*n_par+ParameterIndex.fOUTHARD] = f_hard[1]

            coords_th = track.get_coordinate(warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.XTH] = coords_th[0]
            all_params[i*n_par+ParameterIndex.YTH] = coords_th[1]

            all_params[i*n_par+ParameterIndex.PHITH] = track.get_angle(warm_start[i*n_z+Index.THETA])
            all_params[i*n_par+ParameterIndex.dY] = warm_start[i*n_z+Index.THETA]

    return all_params

def ind_changer(arr, mode):
    """
    Changes the order of the array because the non linear solver is a special solver and prefes another order
    
    Args:
        arr: the array of size (N+1)*n_z whose elements we want to switch
        mode: 'norm2nl' or 'nl2norm'
    """
    out_arr = arr.copy()

    n_z = MPC_params['n_z']

    if mode=='norm2nl':
        changer = l2nl
    else:
        changer = nl2l
    
    for i in range(MPC_params['N']+1):
            out_arr[i*n_z:(i+1)*n_z] = changer(out_arr[i*n_z:(i+1)*n_z]) 
    
    return out_arr

def nl2l(arr):
    return np.concatenate((
        arr[NlIndex.S_X:NlIndex.THETA+1], 
        arr[NlIndex.SOFT:NlIndex.V_K+1], 
        arr[NlIndex.V_DELTA:NlIndex.ACC+1]
        ), axis=0)

def l2nl(arr):
    return np.concatenate((
        arr[Index.SOFT:Index.V_K+1], 
        arr[Index.V_DELTA:Index.ACC+1], 
        arr[Index.S_X:Index.THETA+1]), 
        axis=0)

def get_nlp_params(X, U, warm_start, track, track_hard):
    """
    Obtains the dictionary used to configure the nlp solver. Returns a flag that 
    is used to decide whether slow or fast model is used.
    """
    problem_params = {"x0": ind_changer(np.concatenate((X, U, warm_start[MPC_params["n_z"] :]), axis=None), mode="norm2nl")}
    problem_params["xinit"] = X[: Index.THETA + 1]
    problem_params["all_parameters"] = get_nlp_forces_params(
        np.concatenate((X, U, warm_start[MPC_params["n_z"] :]), axis=None), track, track_hard
    )

    vels = warm_start[Index.V::MPC_params['n_z']]
    use_slow = any([v < SLOW_THRESHOLD for v in vels])

    return problem_params, use_slow

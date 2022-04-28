from gym.logger import info
import numpy as np

def get_wandb_log(steer_v, acc, predictions, cur_speed, cur_angle, cur_steer, cur_theta, F_constr, f_constr, cur_max_P, soft_cons,info_solv, n_z):
    """
    Prepares the dictionary for logging on wandb
    """
    wandb_log = {"steering_vel":steer_v,
                    "acceleration":acc,
                    "current_speeed":cur_speed,
                    "current_angle":cur_angle,
                    "current_steer":cur_steer,
                    "cur_param":cur_theta}

    # constraint violation part
    wandb_log['max_soft_constr_slack'] = max(soft_cons)
    all_real_viol_lx = [(predictions[:2,i] @ F_constr[2*i,:]) - f_constr[2*i] for i in range(int(len(f_constr)/2))]
    all_real_viol_rx = [(predictions[:2,i] @ F_constr[2*i+1,:]) - f_constr[2*i+1] for i in range(int(len(f_constr)/2))]
    for i, (a, b) in enumerate(zip(all_real_viol_lx, all_real_viol_rx)):
        wandb_log['viol_lx_{}'.format(i)] = a 
        wandb_log['viol_rx_{}'.format(i)] = b
    wandb_log['avg_real_constr_viol'] = max([np.mean(all_real_viol_lx), np.mean(all_real_viol_rx)])
    wandb_log['real_constr_viol'] = max([max(all_real_viol_lx), max(all_real_viol_rx)])
    wandb_log['max_matrix_element'] = cur_max_P
    for k, v in info_solv.items():
        wandb_log[k] = v

    return wandb_log
    
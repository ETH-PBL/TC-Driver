import yaml
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from splinify.splinify import SplineTrack


def main(experiment_name: str):
    with open('./log/{}/run_data.txt'.format(experiment_name), 'r') as file:
        run_data = yaml.safe_load(file)

    data_df = pd.read_csv('./log/{}/data.txt'.format(experiment_name), index_col=0)
    data_df['vx'] = data_df['velocity']*np.cos(data_df['slip_angle'] + data_df['yaw'])
    data_df['vy'] = data_df['velocity']*np.sin(data_df['slip_angle'] + data_df['yaw'])
    n_data = len(data_df)
    data_df['acc_rel_x'] = data_df['vx'].diff().fillna(0)/(0.01)
    data_df['acc_rel_y'] = data_df['vy'].diff().fillna(0)/(0.01)

    #print(data_df.head())

    # METRIC 1 : laptime
    print("Time for lap: {:.2f} s".format(n_data/100.0))

    # METRIC 2 : acceleration plot
    fig = plt.figure(0, figsize=(10, 10))
    ax_handle = fig.add_subplot(1,1,1)
    ax_handle.set_title("Acceleration relative to the car frame")
    ax_handle.set_xlim([-10, 10])
    ax_handle.set_ylim([-12, 12])
    plt.plot(data_df['acc_rel_y'][1:], data_df['acc_rel_x'][1:], 'o')
    ax_handle.set_xlabel('Lateral acceleration [m/s^2]')
    ax_handle.set_ylabel('Longitudinal acceleration [m/s^2]')
    
    # METRIC 3 : constraint violation
    constr_viol_time = data_df['out_of_soft'].sum()
    print("Constraint violation time (percentage): {:.2f} s ({:.2f}%)".format(constr_viol_time, 1e4*constr_viol_time/n_data))

    # METRIC 4 : speed histogram
    fig = plt.figure(1, figsize=(10, 5))
    spehist_handle = fig.add_subplot(1,1,1)
    data_df['velocity'].plot.hist(ax = spehist_handle, alpha = 0.5, bins = 20)
    spehist_handle.set_title('Velocity histogram')    
    spehist_handle.yaxis.grid('on')
    spehist_handle.set_xlabel('Velocity [m/s]')

    # METRIC 5 : computation time (histogram can also be good)
    avg_compt_time = data_df['comp_time'].mean()
    print("Average computation time (s): {}".format(avg_compt_time))
    fig = plt.figure(2, figsize=(10, 5))
    comp_hist = fig.add_subplot(1,1,1)

    data_df['comp_time'] *= 1000 # convert to ms

    # first one is removed since it jit compiles a function, las one is also removed as it is jusst a garbage value
    data_df['comp_time'][1:-1].plot.hist(ax = comp_hist, alpha = 0.5, bins = 20)
    comp_hist.set_title('Computation time histogram')    
    comp_hist.yaxis.grid('on')
    comp_hist.set_xlabel('Time [ms]')

    # METRIC 6 : trajectory (with speed profile ?)
    track = SplineTrack('/home/gnone/edoardo-ghignone/pbl_f110_gym/configs/{}_waypoints.txt'.format(run_data['track_name']), safety_margin=2.3*0.31)
    n_points = 1000
    int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, n_points)])
    out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, n_points)])
    fig = plt.figure(3, figsize=(10, 10))
    track_handle = fig.add_subplot(1,1,1)
    track_handle.plot(int_line[:, 0], int_line[:, 1], 'k')
    track_handle.plot(out_line[:, 0], out_line[:, 1], 'k')
    track_handle.plot(data_df['s_x'], data_df['s_y'], 'r')

    

    # METRIC 7: 
    # TODO use speed and steer in MPC too?
    
    plt.show()

if __name__ == '__main__':
    exp_name = 'MPCC_0'
    main(exp_name)
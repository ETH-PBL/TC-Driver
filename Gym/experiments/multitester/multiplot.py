import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from splinify.splinify import SplineTrack

orig_data = pd.read_csv('./coords_orig.csv')
trtr_data = pd.read_csv('./coords_trtr.csv')
mpc_data = pd.read_csv('./coords_mpc.csv')
orig_data_run = pd.read_csv('./runs_data_orig.csv')
trtr_data_run = pd.read_csv('./runs_data_trtr.csv')
mpc_data_run = pd.read_csv('./runs_data_mpc.csv')



with open('/home/gnone/edoardo-ghignone/pbl_f110_gym/configs/SOCHI_trajectory_MPCC.txt', 'r') as file:
    # remove first line
    file.readline()

    coords = file.readlines()
    def parse(string):
        return [float(el) for el in string.split(",")]
    coords = [parse(el) for el in coords]
    coords = np.array(coords)
    coords = np.concatenate((coords, [coords[0]]))

track = SplineTrack('/home/gnone/edoardo-ghignone/pbl_f110_gym/configs/SOCHI_waypoints.txt')

N_RUNS = 13

fig = plt.figure(figsize=(17.4,5.4))
ax_mpc = fig.add_subplot(1,3,1)
ax_orig = fig.add_subplot(1,3,2)
ax_trtr = fig.add_subplot(1,3,3)

h_orig = [0 for _ in range(N_RUNS)]
h_mpc = [0 for _ in range(N_RUNS)]
h_trtr = [0 for _ in range(N_RUNS)]

for i in range(N_RUNS):
    indx = "x_{}".format(i)
    indy = "y_{}".format(i)
    or_len = int(orig_data_run.loc[i, "times"]*100)
    tr_len = int(trtr_data_run.loc[i, "times"]*100)
    mp_len = int(mpc_data_run.loc[i, "times"]*100)
    h_orig[i], = ax_orig.plot(orig_data.loc[:or_len, indx], orig_data.loc[:or_len, indy], color=(0.75,0,0.75,0.1)) # 'm' but transparent
    h_mpc[i], = ax_mpc.plot(mpc_data.loc[:mp_len, indx], mpc_data.loc[:mp_len, indy], color=(1,0,0,0.1)) # 'red' but transparent
    h_trtr[i], = ax_trtr.plot(trtr_data.loc[:tr_len, indx], trtr_data.loc[:tr_len, indy], color=(0.5,0,0.5,0.1)) # 'purple' but transparent

mpc_handle_trtr, = ax_trtr.plot(coords[:,0], coords[:,1], 'gray')
mpc_handle_orig, = ax_orig.plot(coords[:,0], coords[:,1], 'gray')
mpc_handle_mpc, = ax_mpc.plot(coords[:,0], coords[:,1], 'gray')

n_points = 1000
int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, n_points)])
out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, n_points)])

ax_orig.plot(int_line[:, 0], int_line[:, 1], 'k')
ax_mpc.plot(int_line[:, 0], int_line[:, 1], 'k')
ax_trtr.plot(int_line[:, 0], int_line[:, 1], 'k')
ax_orig.plot(out_line[:, 0], out_line[:, 1], 'k')
ax_mpc.plot(out_line[:, 0], out_line[:, 1], 'k')
ax_trtr.plot(out_line[:, 0], out_line[:, 1], 'k')
ax_orig.legend([mpc_handle_orig, h_orig[6]], ['MPC w/o noise', 'end2end w/ noise'], loc=2)
ax_mpc.legend([mpc_handle_mpc, h_mpc[6]], ['MPC w/o noise', 'mpc w/ noise'], loc=2)
ax_trtr.legend([mpc_handle_trtr, h_trtr[6]], ['MPC w/o noise', 'trajectory tracking w/ noise'], loc=2)

ax_trtr.set_xlim(-22,7)
ax_orig.set_xlim(-22,7)
ax_mpc.set_xlim(-22,7)
ax_orig.set_ylim(-12,15)
ax_mpc.set_ylim(-12,15)
ax_trtr.set_ylim(-12,15)

ax_trtr.tick_params(bottom=False,   top=False, left= False, right = False, direction='in')
ax_orig.tick_params(bottom=False,   top=False, left= False, right = False, direction='in')
ax_mpc.tick_params(bottom=False,   top=False, left= False, right = False, direction='in')
ax_trtr.xaxis.set_ticklabels([])
ax_trtr.yaxis.set_ticklabels([])
ax_orig.xaxis.set_ticklabels([])
ax_orig.yaxis.set_ticklabels([])
ax_mpc.xaxis.set_ticklabels([])
ax_mpc.yaxis.set_ticklabels([])
ax_trtr.axis("off")
ax_orig.axis("off")
ax_mpc.axis("off")

plt.tick_params(bottom=False,   top=False, left= False, right = False)
fig.tight_layout(pad=0)

# find advancement
adv_avg_orig = 0
adv_avg_trtr = 0
adv_avg_mpc = 0

for i in range(N_RUNS):
    adv_avg_orig += track.find_theta_slow([orig_data.iloc[int(orig_data_run.loc[i, 'times']*100)-1, i], orig_data.iloc[int(orig_data_run.loc[i, 'times']*100)-1, i+13]])
    adv_avg_trtr += track.find_theta_slow([mpc_data.iloc[int(mpc_data_run.loc[i, 'times']*100)-1, i], mpc_data.iloc[int(mpc_data_run.loc[i, 'times']*100)-1, i+13]])
    adv_avg_mpc += track.find_theta_slow([trtr_data.iloc[int(trtr_data_run.loc[i, 'times']*100)-1, i], trtr_data.iloc[int(trtr_data_run.loc[i, 'times']*100)-1, i+13]])

lentr = track.track_length
print("average advancement of mpc: {}".format(adv_avg_mpc/(13*lentr)))
print("average advancement of orig: {}".format(adv_avg_orig/(13*lentr)))
print("average advancement of trtr: {}".format(adv_avg_trtr/(13*lentr)))

plt.show()

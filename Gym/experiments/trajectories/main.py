import os, pathlib

import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from splinify.splinify import SplineTrack


cur_dir = pathlib.Path(__file__).resolve()
git_dir = cur_dir.parents[2]
maps_dir= os.path.join(git_dir, 'configs/')

track = SplineTrack(os.path.join(maps_dir, "SOCHI_waypoints.txt"), os.path.join(maps_dir, "SOCHI_trajectory_MPCC.txt"))

mpc_len = track.trajectory.track_length-4.8
mpc_coords = np.linspace(0, mpc_len, num=1000)
mpc_coords = np.array([track.trajectory.get_coordinate(th) for th in mpc_coords])


pos_x = pd.read_csv("rl_x.csv")
pos_y = pd.read_csv("rl_y.csv")

orig_x = np.array(pos_x["restful-moon-491 - position_x"], dtype=np.float32)
orig_y = np.array(pos_y["restful-moon-491 - position_y"], dtype=np.float32)
trtr_x = np.array(pos_x["kind-glitter-492 - position_x"], dtype=np.float32)
trtr_y = np.array(pos_y["kind-glitter-492 - position_y"], dtype=np.float32)
orig_l = 0
for i, (x, y) in enumerate(zip(orig_x, orig_y)):
    if i!=0:
        orig_l += np.linalg.norm((x-prev_x, y-prev_y))
    prev_x = x
    prev_y = y
trtr_l = 0
for i, (x, y) in enumerate(zip(trtr_x, trtr_y)):
    if i!=0 and i<5011:
        trtr_l += np.linalg.norm((x-prev_x, y-prev_y))
    prev_x = x
    prev_y = y

# PLOT
fig = plt.figure(figsize=(20,20))
ax = fig.add_subplot(1,1,1)

trtr_handle, = ax.plot(trtr_x, trtr_y, color=(0.5,0,0.5,0.7))
orig_handle, = ax.plot(orig_x, orig_y, color=(0.75,0,0.75,0.7), ls=':')
mpc_handle, = ax.plot(mpc_coords[:,0], mpc_coords[:,1], color=(1,0,0,1))

int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, 1000)])
out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, 1000)])
ax.plot(int_line[:, 0], int_line[:, 1], 'k')
ax.plot(out_line[:, 0], out_line[:, 1], 'k')
ax.legend([mpc_handle, orig_handle, trtr_handle], ['MPC | distance: {:.1f}m'.format(mpc_len), 'end2end | distance: {:.1f}m'.format(orig_l), 'Frenet_trajectory | distance: {:.1f}m'.format(trtr_l)])

ax.tick_params(bottom=False,   top=False, left= False, right = False, direction='in')

ax.xaxis.set_ticklabels([])
ax.yaxis.set_ticklabels([])
ax.axis("off")

plt.tick_params(bottom=False,   top=False, left= False, right = False)
fig.tight_layout(pad=0)

plt.show()
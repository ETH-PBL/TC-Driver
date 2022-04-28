import time
import splinify
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

track = splinify.SplineTrack('/home/invitedguest/edoardo-ghignone/pbl_f110_gym/configs/circle_waypoints.txt')
traj_track = splinify.Trajectory('/home/invitedguest/edoardo-ghignone/pbl_f110_gym/configs/circle_trajectory_MPCC.txt')

fig, ax = plt.subplots()
ax.set_xlim((-100,100))
ax.set_ylim((-100,100))

i=0
traj = np.array([traj_track.get_coordinate(i)])
inn = np.array([track.get_coordinate(i, line='int')])
mid = np.array([track.get_coordinate(i, line='mid')])
out = np.array([track.get_coordinate(i, line='out')])
trajline, = plt.plot(traj[0,0], traj[0,1], 'x')
innline, = plt.plot(inn[0,0], inn[0,1])
midline, = plt.plot(mid[0,0], mid[0,1])
outline, = plt.plot(out[0,0], out[0,1])

def update_plot(frame):
    global i, inn, mid, out, traj

    traj = np.append(traj, [traj_track.get_coordinate(i)], axis = 0)
    inn = np.append(inn, [track.get_coordinate(i, line='int')], axis = 0)
    mid = np.append(mid, [track.get_coordinate(i)], axis = 0)
    out = np.append(out, [track.get_coordinate(i, line='out')], axis = 0)
    trajline.set_data(traj[:, 0], traj[:, 1])
    innline.set_data(inn[:, 0], inn[:, 1])
    midline.set_data(mid[:, 0], mid[:, 1])
    outline.set_data(out[:, 0], out[:, 1])
    i+=0.1
    plt.draw()


my_animatino = FuncAnimation(fig, update_plot, interval=0, repeat=True)
plt.show()


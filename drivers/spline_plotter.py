import time
from matplotlib.animation import FuncAnimation
from splinify.splinify import SplineTrack, SplineTrackNew
import pathlib
import matplotlib.pyplot as plt
import numpy as np

cur_dir = str(pathlib.Path(__file__).parent.resolve())
maps_directory = '/src/maps/'
track_name = 'smile' # only SOCHI, circle, squarish are available for the time being
dir_path = cur_dir + maps_directory + track_name

#track = SplineTrack(dir_path + '_waypoints.txt', safety_margin=0)
track = SplineTrackNew(dir_path + '_waypoints_new.txt', safety_margin=1*0.31)

inn = np.empty((0, 2))
mid = np.empty((0, 2))
out = np.empty((0, 2))
traj = np.empty((0, 2))

fig = plt.figure(figsize=(10, 10))
plt.xlim(-15, 20)
plt.ylim(-30, 5)
inl = None
midl = None
outl = None
trajl = None

theta_traj = 0
tot_time = 0

i = 0

def update_plot(frame):
    global inn, mid, out, i, inl, midl, outl, traj, trajl, tot_time

    inn = np.append(inn, [track.get_coordinate(i, line='int')], axis=0)
    mid = np.append(mid, [track.get_coordinate(i, line='mid')], axis=0)
    out = np.append(out, [track.get_coordinate(i, line='out')], axis=0)

    theta_traj = track.trajectory.find_theta(track.get_coordinate(i, line='mid'), i+track.delta_traj)
    traj = np.append(traj, [track.trajectory.get_coordinate(theta_traj)], axis=0)
    
    if i==0:
        inl, = plt.plot(inn[:, 0], inn[:, 1], 'r')
        midl, = plt.plot(mid[:, 0], mid[:, 1], 'b')
        outl, = plt.plot(out[:, 0], out[:, 1], 'g')
        trajl, = plt.plot(traj[:, 0], traj[:, 1], 'purple')
    else:
        inl.set_data(inn[:, 0], inn[:, 1])
        midl.set_data(mid[:, 0], mid[:, 1])
        outl.set_data(out[:, 0], out[:, 1])
        trajl.set_data(traj[:, 0], traj[:, 1])
    print("Angle: {}°".format(360*track.get_angle(i)/(2*np.pi)))
    i += 1




my_animatino = FuncAnimation(fig, update_plot, interval=100, repeat=True)

plt.show()
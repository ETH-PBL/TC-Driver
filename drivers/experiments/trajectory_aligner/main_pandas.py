import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("./coords_mpc_tr.csv", index_col=0)    

start = 316
stop = -640
step = 3

red_x = df['x_0'][start:stop:step]
red_y = df['y_0'][start:stop:step]

dist_x = red_x + 5.47066488784
dist_y = red_y + 54.3682934902
dist = dist_x**2 + dist_y**2
idx = np.argmin(dist)

red_x = pd.concat((red_x.iloc[idx:], red_x.iloc[0:idx]))
red_y = pd.concat((red_y.iloc[idx:], red_y.iloc[0:idx]))

plt.plot(red_x, red_y, 'x')
plt.show()

with open("f_trajectory_MPCC.txt", 'w') as file:
    file.writelines("x, y\n")
    for x, y in zip(red_x, red_y):
        file.writelines("{}, {} \n".format(x, y))


import matplotlib.pyplot as plt
import numpy as np

def parseline(line: str):
    line = line.strip().split(',')
    line = [el.strip('"') for el in line]
    return line
    
x = np.empty(1819)

with open("f_position_x.csv", 'r') as file:
    file.readline() # discard first line

    for i, line in enumerate(file):
        data = parseline(line)
        x[i] = float(data[1])
    
y = np.empty(1819)

with open("f_position_y.csv", 'r') as file:
    file.readline() # discard first line

    for i, line in enumerate(file):
        data = parseline(line)
        y[i] = float(data[1])

start = 590
stop = -351
red_x = x[start:stop:10]
red_y = y[start:stop:10]
plt.plot(red_x, red_y, 'x')
plt.show()

with open("f_trajectory_MPCC.txt", 'w') as file:
    for x, y in zip(red_x, red_y):
        file.writelines("{}, {} \n".format(x, y))


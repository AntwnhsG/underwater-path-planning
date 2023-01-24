import numpy as np
import matplotlib.pyplot as plt
import csv

# A*
# path_x = [-103.0, -97.14, -97.04, -97.1, -91.23, -85.31, -79.54, -73.63, -67.76, -61.83,\
#           -55.91, -49.99, -44.08, -38.17, -32.23, -26.32, -20.4, -14.48, -8.55, -2.63, 3.29, 9.21, 15.13]
# path_y = [15.0, 20.87, 26.77, 32.67, 32.73, 32.65, 38.61, 38.72, 38.66, 38.68, 38.68, 38.68, \
#           38.68, 38.68, 38.69, 38.69, 38.69, 38.69, 38.69, 38.69, 38.69, 38.69, 38.7]

#PSO
# path_x = [-103, -70.384, -48.033, -30.127, -20.179, -8.177, 1.247, 7.610, 9.322, 9.077, 10.391]
# path_y = [15, 8.134, 8.507, 3.075, 6.799, 10.095, 13.991, 20.204, 22.540, 25.936, 33.632]

# MSP
# path_x = [-103, -103, -103, -97.2, -91.4, -85.6, -79.8, -74, -68.2, -62.4, -56.6, -50.8, -45, -39.2, -33.4, -27.6, -21.8, -10.2, -4.4, 1.4, 7.2, 13, 18.8] 
# path_y = [13, 18.8, 24.6, 30.4, 30.4, 30.4, 30.4, 30.4, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2, 36.2]

with open('path.csv') as file_obj:
    heading = next(file_obj)
    reader_obj = csv.reader(file_obj)
    path_x = []
    path_y = []
    pathx = [0]
    pathy = [10]
    j = 0
    x = 0
    y = 0
    for row in reader_obj:
        pathx.append(int(row[0]))
        pathy.append(int(row[1]))
for i in pathx:
    if i == 0:
        pathx.append(-103)
        x = -103
        j = i
    else:
        if i-1 == j:
            x = x + 5.8
            j = i
            path_x.append(x)
        elif i+1 == j:
            x = x -5.8
            path_x.append(x)
        else:
            path_x.append(x)

for i in pathy:
    if i == 10:
        path_y.append(13)
        y = 13
        j = i
    else:
        if i-1 == j:
            y = y + 5.8
            j = i
            path_y.append(y)
        else:
            path_y.append(y)

obs_x = [
            -79.8, -79.8, 
            -74, -74,

            -91.4, -91.4, -91.4, -91.4, 
            -85.6, -85.6, -85.6, -85.6,
            -79.8, -79.8, -79.8, -79.8,

            -74, -74, 
            -68.2, -68.2,

            -56.6, -56.6, -56.6, 
            -50.8, -50.8, -50.8, 

            -56.6, -56.6, 
            -50.8, -50.8, 

            -27.6, -27.6, 
            -21.8, -21.8,

            -21.8, -21.8, -21.8, 
            -16, -16, -16, 
            -10.2, -10.2, -10.2]

obs_y = [
            -27.6, -21.8, 
            -27.6, -21.8,

            7.2, 13, 18.8, 24.6,
            7.2, 13, 18.8, 24.6, 
            7.2, 13, 18.8, 24.6, 

            57.6, 63.4, 
            57.6, 63.4, 

            -10.2, -4.4, 1.4,
            -10.2, -4.4, 1.4,

            24.6, 30.4, 
            24.6, 30.4, 

            -16, -10.2, 
            -16, -10.2,

            18.8, 24.6, 30.4, 
            18.8, 24.6, 30.4, 
            18.8, 24.6, 30.4]

plt.plot(path_y, path_x, 'c', label='Path')
plt.plot(obs_y, obs_x, 'r8', label ='obs')
plt.title("Optimal Path Using PSO")
plt.xlabel("Y Axis")
plt.ylabel("X Axis")
plt.legend()
plt.savefig("PSO_Path.png")
plt.show()
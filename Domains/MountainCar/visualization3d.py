#!/usr/bin/python
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3 
import numpy as np
import csv
import math

positions_x = list()
with open('../../../ManifoldLearning-build/converged_position_data_x_3d_pca.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     for row in spamreader:
         for cell in row:
              positions_x.append(cell)

positions_y = list()
with open('../../../ManifoldLearning-build/converged_position_data_y_3d_pca.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     for row in spamreader:
         for cell in row:
              positions_y.append(cell)

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)
print len(positions_x)
for t in range(len(positions_x)):
    if t == 0:
        points, = ax.plot([float(positions_x[t])], [float(positions_y[t])], [math.sin(3*float(positions_x[t]))], marker='o', linestyle='None')
        ax.set_xlim(-1.5, 1) 
        ax.set_ylim(-1.5, 1)
        ax.set_zlim(-2, 2) 
    else:
        new_x = float(positions_x[t])
	new_y = float(positions_y[t])
        new_z = math.sin(3 * float(positions_x[t])) + math.sin(3 * float(positions_y[t]))
        points.set_data(new_x, new_y)
        points.set_3d_properties(new_z)
        #points.set_data(new_x, new_y, new_z)
    plt.pause(0.01)

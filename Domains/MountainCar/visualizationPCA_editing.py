#!/usr/bin/python
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3 
import numpy as np
import csv
import math

#data = list(list())
data_x = list()
data_y = list();
data_z = list();
with open('../../../ManifoldLearning-build/converged_state_data_normal_3d.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     for row in spamreader:
         data_temp = list()
         data_x.append(float(row[0]))
         data_y.append(float(row[1]))
         data_z.append(float(row[2]))

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)
#for i in range(0, len(data)):
ax.scatter(data_x, data_y, data_z, c='r', marker='o')
ax.quiver(0, 0, 0, .6811607692341, .6400383526871, .3554868683173, length=0.1)

plt.show()

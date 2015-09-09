#!/usr/bin/python
import matplotlib.pyplot as plt 
import numpy as np
import csv
import math

positions = list()
with open('../../../ManifoldLearning-build/converged_position_data.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     for row in spamreader:
         for cell in row:
              positions.append(cell)

fig, ax = plt.subplots()
for t in range(len(positions)):
    if t == 0:
        points, = ax.plot(float(positions[t]), math.sin(3*float(positions[t])), marker='o', linestyle='None')
        ax.set_xlim(-1.5, 1) 
        ax.set_ylim(-1.5, 1) 
    else:
        new_x = float(positions[t])
        new_y = math.sin(3 * float(positions[t]))
        points.set_data(new_x, new_y)
    plt.pause(0.01)

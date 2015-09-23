#!/usr/bin/python
import matplotlib.pyplot as plt 
import numpy as np
import csv
import math
import sys

num_links = int(sys.argv[1]) + 1
all_positions = list()
with open('data.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     for row in spamreader:
         positions = list()
         for cell in row:
              positions.append(cell)
              if len(positions) == 2:
                  continue
         all_positions.append(positions)

fig, ax = plt.subplots()

t = 0
pos_x = list()
pos_y = list()
it = 0
while 1:
    if t > len(all_positions):
        break
    for i in range(0, num_links):
        pos_x.append(float(all_positions[t][0]))
        pos_y.append(float(all_positions[t][1]))
        t = t + 1
    if it == 0:    
        points, = ax.plot(pos_x, pos_y, marker='o', linestyle='None')
        ax.set_xlim(-1, 1) 
        ax.set_ylim(-1, 1) 
    else:
        points.set_data(pos_x, pos_y)
    it = it + 1
    pos_x = []
    pos_y = []
    #if it % 10 == 0:
    plt.pause(0.01)

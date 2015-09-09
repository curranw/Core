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
with open('../../../ManifoldLearning-build/transformed.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     for row in spamreader:
         data_temp = list()
         data_x.append(float(row[0]))
         data_y.append(float(row[1]))

# Attaching 3D axis to the figure
#for i in range(0, len(data)):
plt.scatter(data_x, data_y, c='r', marker='o')
plt.show()

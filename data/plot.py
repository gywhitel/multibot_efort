#!/usr/bin/python3
import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np

# data = csv.reader(open('joint_angle_robot3.csv'))
data = pd.read_csv('joint_angle_robot3.csv')

# joint1, joint2, joint3, joint4, joint5, joint6 =[], [], [], [], [], []
# for row in data:
#     joint1.append(row[0])
#     joint2.append(row[1])
#     joint3.append(row[2])
#     joint4.append(row[3])
#     joint5.append(row[4])
#     joint6.append(row[5])
t = np.arange(0, len(data['J1']))
plt.plot(t, data['J1'])
plt.plot(t, data['J2'])
plt.plot(t, data['J3'])
plt.plot(t, data['J4'])
plt.plot(t, data['J5'])
plt.plot(t, data['J6'])
plt.show()
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 18:18:24 2017

@author: hyj
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Jun 22 21:43:55 2017

@author: hyj
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import os
filepath=os.path.abspath('..')+"/bin"  #表示当前所处的文件夹上一级文件夹的绝对路径

position = []
quaterntions = []
timestamp = []
tx_index = 5

with open(filepath + '/imu_pose.txt', 'r') as f:  # imu_circle   imu_spline

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数

        position.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])

position1 = []
quaterntions1 = []
timestamp1 = []
with open(filepath + '/imu_int_pose.txt', 'r') as f:  # imu_pose   imu_spline

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数

        # timestamp.append( numbers_float[0])
        # quaterntions1.append( [numbers_float[tx_index+6], numbers_float[tx_index+3],numbers_float[tx_index+4],numbers_float[tx_index+5]   ] )   # qw,qx,qy,qz
        position1.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])

position2 = []
quaterntions2 = []
timestamp2 = []
with open(filepath + '/imu_int_pose_noise.txt', 'r') as f:  # cam_pose_opt_o_0   cam_pose_opt_o_0

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数

        # timestamp.append( numbers_float[0])
        # quaterntions2.append( [numbers_float[tx_index+6], numbers_float[tx_index+3],numbers_float[tx_index+4],numbers_float[tx_index+5]   ] )   # qw,qx,qy,qz
        position2.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])

position3 = []
quaterntions3 = []
timestamp3 = []
with open(filepath + '/imu_int_pose_noise_midpoint.txt', 'r') as f:  # imu_int_pose trajectory with noise using midpoint integrater

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数

        # timestamp.append( numbers_float[0])
        # quaterntions2.append( [numbers_float[tx_index+6], numbers_float[tx_index+3],numbers_float[tx_index+4],numbers_float[tx_index+5]   ] )   # qw,qx,qy,qz
        position3.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])

position4 = []
quaterntions4 = []
timestamp4 = []
with open(filepath + '/imu_int_pose_midpoint.txt', 'r') as f:  # imu_int_pose trajectory without noise using midpoint integrater

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数

        # timestamp.append( numbers_float[0])
        # quaterntions2.append( [numbers_float[tx_index+6], numbers_float[tx_index+3],numbers_float[tx_index+4],numbers_float[tx_index+5]   ] )   # qw,qx,qy,qz
        position4.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])


    ### plot 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

xyz = zip(*position)
xyz1 = zip(*position1)
#xyz2 = zip(*position2)
#xyz3 = zip(*position3)
xyz4 = zip(*position4)
print
ax.plot(xyz[0], xyz[1], xyz[2], label='gt')
ax.plot(xyz1[0], xyz1[1], xyz1[2], label='imu_int')
#ax.plot(xyz2[0], xyz2[1], xyz2[2], label='noise')
#ax.plot(xyz3[0], xyz3[1], xyz3[2], label='noise_midpoint')
ax.plot(xyz4[0], xyz4[1], xyz4[2], label='imu_int_midpoint')
ax.legend()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 18:18:24 2017

@author: hyj
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from GeometryLib import drawCoordinateFrame, euler2Rbn, euler2Rnb
import transformations as tf

import os
# filepath=os.path.abspath('.')  #表示当前所处的文件夹的绝对路径
filepath = os.path.abspath('..')+"/bin"  # 表示当前所处的文件夹上一级文件夹的绝对路径

point_id = []
x = []
y = []
z = []

with open(filepath + '/all_points.txt', 'r') as f:
    data = f.readlines()  # txt中所有字符串读入data

    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数
        x.append(numbers_float[0])
        y.append(numbers_float[1])
        z.append(numbers_float[2])


position = []
quaterntions = []
timestamp = []
qw_index = 1
with open(filepath + '/cam_pose.txt', 'r') as f:  # imu_circle   imu_spline

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数

        #timestamp.append( numbers_float[0])
        quaterntions.append([numbers_float[qw_index], numbers_float[qw_index+1],
                             numbers_float[qw_index+2], numbers_float[qw_index+3]])   # qw,qx,qy,qz
        position.append([numbers_float[qw_index+4],
                         numbers_float[qw_index+5], numbers_float[qw_index+6]])


# plot 2d frame
fig_frame = plt.figure()
ax_frame = fig_frame.gca()


# plot 3d
fig = plt.figure()
plt.ion()
ax = fig.gca(projection='3d')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
rpy = []
t = []
for i in range(0, 400, 5):
    ax_frame.clear()
    ax.clear()
    # ax.scatter(x, y, z, c='g')

    x1 = []
    y1 = []
    z1 = []
    rpy.append(tf.euler_from_quaternion(quaterntions[i]))
    t.append(position[i])
    p = position[i]
    for j in range(len(rpy)):
        drawCoordinateFrame(ax, rpy[j], t[j])
        pass

    s = filepath + '/keyframe/all_points_' + str(i)+'.txt'
    with open(s, 'r') as f:
        data = f.readlines()  # txt中所有字符串读入data
        skip = False
        for line in data:
            if(skip):
                skip = False
                continue
            else:
                skip = False
                odom = line.split()  # 将单个数据分隔开存好
                numbers_float = list(map(float, odom))  # 转化为浮点数
                x1.append(numbers_float[0])
                y1.append(numbers_float[1])
                z1.append(numbers_float[2])

                ax.plot([numbers_float[0],   p[0]], [
                        numbers_float[1], p[1]], zs=[numbers_float[2], p[2]])
                ax_frame.plot(numbers_float[4], numbers_float[5], ".")

    s = filepath + '/house_model/house.txt'
    with open(s, 'r') as f:
        data = f.readlines()  # txt中所有字符串读入data
        for line in data:
            odom = line.split()  # 将单个数据分隔开存好
            numbers_float = list(map(float, odom))  # 转化为浮点数
            ax.plot([numbers_float[0], numbers_float[3]], [numbers_float[1],
                                                           numbers_float[4]], 'b', zs=[numbers_float[2], numbers_float[5]])

    ax.scatter(x1, y1, z1, c='r', marker='^')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-15, 20)
    ax.set_ylim(-15, 20)
    ax.set_zlim(0, 20)
    ax.legend()

    ax_frame.set_xlim(0, 640)
    ax_frame.set_ylim(0, 480)
    ax_frame.set_aspect('equal')

    plt.show()
    plt.pause(0.01)

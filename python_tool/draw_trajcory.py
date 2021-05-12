#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 18:18:24 2017

@author: hyj

example: python draw_trajcory imu
"""
import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def draw_trajecttory(argv):
    np.set_printoptions(suppress = True) #是否压缩由科学计数法表示的浮点数
    filepath=os.path.abspath('..')+"/bin"  #表示当前所处的文件夹上一级文件夹的绝对路径

    sensor = argv[0]

    position = []
    quaterntions = []
    timestamp = []
    tx_index = 5
    position = np.loadtxt(filepath + '/' + sensor + '_pose.txt', usecols=(tx_index, tx_index + 1, tx_index + 2))

    position1 = []
    quaterntions1 = []
    timestamp1 = []
    data = np.loadtxt(filepath + '/' + sensor + '_int_pose.txt')
    # timestamp1 = data[:,0]
    # quaterntions1 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
    position1 = data[:, [tx_index, tx_index + 1, tx_index + 2]]

    position2 = []
    quaterntions2 = []
    timestamp2 = []
    data = np.loadtxt(filepath + '/' + sensor + '_int_pose_noise.txt')
    # timestamp2 = data[:,0]
    # quaterntions2 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
    position2 = data[:, [tx_index, tx_index + 1, tx_index + 2]]

    position3 = []
    quaterntions3 = []
    timestamp3 = []
    data = np.loadtxt(filepath + '/' + sensor + '_int_pose_midpoint.txt')
    # timestamp3 = data[:,0]
    # quaterntions3 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
    position3 = data[:, [tx_index, tx_index + 1, tx_index + 2]]

    position4 = []
    quaterntions4 = []
    timestamp4 = []
    data = np.loadtxt(filepath + '/' + sensor + '_int_pose_noise_midpoint.txt')
    # timestamp4 = data[:,0]
    # quaterntions4 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
    position4 = data[:, [tx_index, tx_index + 1, tx_index + 2]]

    ### plot 3d
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.plot(position[:, 0], position[:, 1], position[:, 2], label='gt')
    # ax.plot(position1[:, 0], position1[:, 1], position1[:, 2], label=sensor + '_int')
    # ax.plot(position2[:, 0], position2[:, 1], position2[:, 2], label=sensor + '_int_noise')
    ax.plot(position3[:, 0], position3[:, 1], position3[:, 2], label=sensor + '_midint')
    ax.plot(position4[:, 0], position4[:, 1], position4[:, 2], label=sensor + '_midint_noise')
    ax.plot([position[0, 0]], [position[0, 1]], [position[0, 2]], 'r.', label='start')

    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


if __name__ == '__main__':
    draw_trajecttory(sys.argv[1:])
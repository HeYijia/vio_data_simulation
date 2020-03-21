# -*- coding: utf-8 -*-
"""
Created on Tue May  9 22:08:34 2017

@author: hyj
"""

import numpy as np

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
# ############## 3d tranform lib  ######################
#            几何变换的一些函数，欧拉角，　旋转矩阵等
##############################################


def Rx(phi):
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), np.sin(phi)],
                     [0, -np.sin(phi), np.cos(phi)]])


def Ry(theta):
    return np.array([[np.cos(theta), 0, -np.sin(theta)],
                     [0, 1, 0],
                     [np.sin(theta), 0, np.cos(theta)]])


def Rz(psi):
    return np.array([[np.cos(psi), np.sin(psi), 0],
                     [-np.sin(psi), np.cos(psi), 0],
                     [0, 0, 1]])


def euler2Rbn(eulerAngles):
    ''' navigation frame to body frame '''
    return Rx(eulerAngles[0]).dot(Ry(eulerAngles[1]).dot(Rz(eulerAngles[2])))


def euler2Rnb(eulerAngles):
    """ body frame to navigation frame ,
        euler2Rnb = euler2Rnb^t """
    return Rz(-eulerAngles[2]).dot(Ry(-eulerAngles[1]).dot(Rx(-eulerAngles[0])))

# Checks if a matrix is a valid rotation matrix.


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.atan2(R[2, 1], R[2, 2])
        y = np.atan2(-R[2, 0], sy)
        z = np.atan2(R[1, 0], R[0, 0])
    else:
        x = np.atan2(-R[1, 2], R[1, 1])
        y = np.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def Transfrome(eulerAngles, t, point):
    ''' eulerAngles = np.array([roll, pitch, yaw] ) , t = np.array( [tx, ty, tz]) ,  point = np.array( [px, py, pz])'''
    return euler2Rnb(eulerAngles).dot(point) + t


# ############## 3d draw lib  ######################
#            some function used to draw 3d arrows
##############################################


# draw 3d arrow
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def drawCoordinateFrame(ax, rpy, t):

    # define origin
    o = np.array([0, 0, 0])
    # define ox0y0z0 axes
    x0 = np.array([1, 0, 0])
    y0 = np.array([0, 1, 0])
    z0 = np.array([0, 0, 1])

    R = euler2Rnb(rpy)

    # transfrom b0 frame to the new bi frame
    o1 = R.dot(o) + t
    x1 = R.dot(x0) + t
    y1 = R.dot(y0) + t
    z1 = R.dot(z0) + t

    x = Arrow3D([o1[0], x1[0]],    [o1[1], x1[1]],     [
                o1[2], x1[2]],  mutation_scale=20, arrowstyle='-|>', color='r')
    y = Arrow3D([o1[0], y1[0]],    [o1[1], y1[1]],     [
                o1[2], y1[2]],  mutation_scale=20, arrowstyle='-|>', color='b')
    z = Arrow3D([o1[0], z1[0]],    [o1[1], z1[1]],     [
                o1[2], z1[2]],  mutation_scale=20, arrowstyle='-|>', color='g')

   # draw
    ax.add_artist(x)
    ax.add_artist(y)
    ax.add_artist(z)

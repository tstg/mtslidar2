import numpy as np


def rotate(degrees, axis):
    radian = np.deg2rad(degrees)
    cos_r = np.cos(radian)
    ax = axis / np.linalg.norm(axis)

    ax = np.array([ax])
    x = ax[0, 0]
    y = ax[0, 1]
    z = ax[0, 2]

    aa = np.matmul(ax.T, ax)
    a = np.array([[0, -z,  y],
                  [z,  0, -x],
                  [-y,  x, 0]])

    r = cos_r * np.eye(3) + (1 - cos_r) * aa + np.sin(radian) * a
    return r


def scale(sx, sy, sz):
    s = np.eye(4)
    s[0, 0] = sx
    s[1, 1] = sy
    s[2, 2] = sz
    return s


def translate(tx, ty, tz):
    t = np.eye(4)
    t[0, 3] = tx
    t[1, 3] = ty
    t[2, 3] = tz
    return t


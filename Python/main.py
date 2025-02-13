# This is a sample Python script.
import math
import sys

import numpy as np
import random
from scipy.spatial import distance

from matplotlib import pyplot as plt
import matplotlib.patches as patches

import Lattices
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def dist(_v1, _v2) -> float:
    return math.sqrt(math.pow(_v1[0]))


# def volume(_vertices) -> int:
#    return abs(np.linalg.det(vertices))


# return the number of samples  created from this group of grid-vectors
def samples(dim, _vertices, R) -> int:
    det = abs(np.linalg.det(_vertices))
    return pow(R, dim) / det


# for a list of vertices defining a parallelogram, check if a vertex ball-cover with
# radius (R) covers it.
def check_if_cover_2d(_vertices, R, res=10) -> bool:
    for i in range(0, res + 1):
        for j in range(0, res + 1):
            v_inner = np.array(_vertices[0]) * (i / float(res)) + np.array(_vertices[1]) * (j / float(res))
            covered = False
            for _v in _vertices:
                # if np.linalg.norm(_v, v_inner) <=  R:
                if distance.euclidean(_v, v_inner) <= R:
                    covered = True
                    break
            if not covered:
                # print("[{0}] point not covered: {1},{2}".format(R, v_inner[0], v_inner[1]))
                return False
    # print("[{0}] it is a cover".format(R))
    return True


def check_if_cover_3d(_vertices, R, res=10) -> bool:
    for i in range(0, res + 1):
        for j in range(0, res + 1):
            for w in range(0, res + 1):
                v_inner = (np.array(_vertices[0]) * (i / float(res)) + np.array(_vertices[1]) * (j / float(res)) +
                           np.array(_vertices[2]) * (w / float(res)))
                covered = False
                for _v in _vertices:
                    # if np.linalg.norm(_v, v_inner) <=  R:
                    if distance.euclidean(_v, v_inner) <= R:
                        covered = True
                        break
                if not covered:
                    # print("[{0}] point not covered: {1},{2}".format(R, v_inner[0], v_inner[1]))
                    return False
    # print("[{0}] it is a cover".format(R))
    return True


def unit_vector(dim, angle1, angle2=0):
    angle1_rad = math.radians(angle1)
    angle2_rad = math.radians(angle2)
    if dim == 2:
        return np.array([math.cos(angle1_rad), math.sin(angle1_rad)])
    elif dim == 3:
        return np.array([math.sin(angle1_rad) * math.cos(angle2_rad), math.sin(angle1_rad) * math.sin(angle2_rad), math.cos(angle1_rad)])


def test_2d_cover():
    min_R = 1
    step_angle = 1
    step_radius = 0.05
    vertices = []
    min_samples = sys.float_info.max
    angle1 = 0
    for angle2 in np.arange(angle1 + step_angle, 90 + step_angle, step_angle):
        # print("angles: {0},{1}".format(angle1, angle2))
        v1 = unit_vector(2, angle1)
        v2 = unit_vector(2, angle2)
        vertices = np.array([v1, v2, np.zeros(2), v1 + v2])
        vertices_base = np.array([v1, v2])
        # print(vertices)
        min_R = 1
        for R in np.arange(0, 1 + step_radius, step_radius):
            # print("trying for R={0}".format(R))
            if check_if_cover_2d(vertices, R) and min_R > R:
                min_R = R
                break
        samples_curr = samples(2, vertices_base, min_R)
        if samples_curr < min_samples:
            min_samples = samples_curr
            print(
                "The angles {0},{1} achieved a better sample multiplier of [{2}] with radius={3}".format(angle1, angle2,
                                                                                                         min_samples,
                                                                                                         min_R))


def test_3d_cover():
    min_R = 1
    vertices = []
    min_samples = sys.float_info.max
    c = 0

    step_angle = 5
    angle_start = 0
    angle_stop = 90
    angle1_a = angle2_a = 0
    angle2_b = 0
    step_radius = 0.05
    radius_start = 0.5
    radius_stop = 0.75
    resolution = 5
    for angle1_b in np.arange(step_angle, angle_stop + step_angle, step_angle):
        for angle1_c in np.arange(step_angle, angle_stop + step_angle, step_angle):
            for angle2_c in np.arange(step_angle, angle_stop + step_angle, step_angle):
                # print("angles: {0},{1}".format(angle1, angle2))
                # print (angle1_a, angle1_b, angle1_c, angle2_a, angle2_b, angle2_c)
                v1 = unit_vector(3, angle1_a, angle2_a)
                v2 = unit_vector(3, angle1_b, angle2_b)
                v3 = unit_vector(3, angle1_c, angle2_c)
                # print(v1, v2, v3)
                if np.count_nonzero(v1 - v2) == 0 or np.count_nonzero(v1 - v3) == 0 or np.count_nonzero(v2 - v3) == 0:
                    continue
                vertices = np.array([v1, v2, v3, v1 + v2, v3 + v1, v3 + v2, np.zeros(3), v1 + v2 + v3])
                vertices_base = np.array([v1, v2, v3])
                # print(vertices)
                min_R = 1
                for R in np.arange(0, 1 + step_radius, step_radius):
                    # print("trying for R={0}".format(R))
                    if check_if_cover_3d(vertices, R, resolution) and min_R > R:
                        if check_if_cover_3d(vertices, R, 10) and check_if_cover_3d(vertices, R, 50):
                            # a harder test for points that pass the first one
                            min_R = R
                            break
                samples_curr = samples(3, vertices_base, min_R)
                if samples_curr < min_samples:
                    min_samples = samples_curr
                    # print(min_samples, min_R)
                    print("v1:{0},{1} v2:{2},{3} v1:{4},{5}".
                          format(angle1_a, angle2_a, angle1_b, angle2_b, angle1_c, angle2_c))
                    print("achieved a better sample multiplier of [{0}] with radius={1}".
                          format(min_samples, min_R))


def plot_parallelogram(vertices):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    v_x = []
    v_y = []
    v_z = []
    v1 = vertices[0]
    v2 = vertices[1]
    v3 = vertices[2]
    for v in vertices:
        v_x.append(v[0])
        v_y.append(v[1])
        v_z.append(v[2])
    ax.set_box_aspect((np.ptp(v_x), np.ptp(v_y), np.ptp(v_z)))
    ax.scatter(v_x, v_y, v_z, marker='o')
    ax.quiver3D(0, 0, 0, v1[0], v1[1], v1[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], length=1,
                arrow_length_ratio=0.02)

    ax.quiver3D(0, 0, 0, v2[0], v2[1], v2[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v2[0], v2[1], v2[2], v1[0], v1[1], v1[2], length=1,
                arrow_length_ratio=0.02)

    ax.quiver3D(0, 0, 0, v3[0], v3[1], v3[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v1[0], v1[1], v1[2], v3[0], v3[1], v3[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v2[0], v2[1], v2[2], v3[0], v3[1], v3[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2], v3[0], v3[1], v3[2], length=1,
                arrow_length_ratio=0.02)

    ax.quiver3D(v3[0], v3[1], v3[2], v1[0], v1[1], v1[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v3[0] + v1[0], v3[1] + v1[1], v3[2] + v1[2], v2[0], v2[1], v2[2], length=1,
                arrow_length_ratio=0.02)

    ax.quiver3D(v3[0], v3[1], v3[2], v2[0], v2[1], v2[2], length=1,
                arrow_length_ratio=0.02)
    ax.quiver3D(v3[0] + v2[0], v3[1] + v2[1], v3[2] + v2[2], v1[0], v1[1], v1[2], length=1,
                arrow_length_ratio=0.02)


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # a = np.array([[11.8467,  11.8467 , 11.8467  ,11.8467 , 11.8467 ,  -9.126],
    #              [-11.8467,        0  ,      0   ,     0      ,  0  , 2.7207],
    #              [0, -11.8467, 0, 0, 0, 2.7207],
    #              [0, 0, -11.8467, 0, 0, 2.7207],
    #              [0, 0, 0, -11.8467, 0, 2.7207],
    #              [0, 0, 0, 0, -11.8467, 2.7207]])
    # b = np.array([5,-2,-5,1,0,0]).transpose()
    # # b = np.array([0,0,0,1,0,1,]).transpose()
    # c1 = a @ b
    # c = a @ b + np.array([20.831700,35.657200,7.181250,-23.466600,37.366900,34.179100,]).transpose()

    ## Show samples in ball
    # zn = Lattices.Zn(2, 0.5, 0.5)
    # zn.show_samples_in_ball(1)
    # dn = Lattices.DnStar(2, 0.5, 0.5)
    # dn.show_samples_in_ball(1)
    # an = Lattices.AnStar(2, 0.5, 0.5)
    # an.show_samples_in_ball(1)
    # plot_samples_upper_limit()
    an = Lattices.AnStar(3, 0.5, 0.5) # 3D
    an.show_samples_in_ball(0.6) # 3D
    zn = Lattices.Zn(3, 0.5, 0.5) # 3D
    zn.show_samples_in_ball(0.6) # 3D

    ## Show construction
    # an = Lattices.AnStar(2, 0.5, 0.5)
    # an.show_lattice_construction()
    # # 3D #
    # # test_3d_cover()
    # v1 = unit_vector(3, 90, 0)
    # v2 = unit_vector(3, 90, 60)
    # v3 = unit_vector(3, 30, 30)
    # va = v1+v3
    # vb = v2-v1
    # vertices = np.array([v1, v2, v3, v1 + v2, v3 + v1, v3 + v2, np.zeros(3), v1 + v2 + v3])
    # vertices_base = np.array([v1, v2, v3])
    # # print(check_if_cover_2d(vertices, math.sqrt(3/8), 1000))
    # print(samples(3, vertices_base, math.sqrt(3/8)))
    # plot_parallelogram(vertices)
    # # 2D #
    # # test_2d_cover()
    # # v1 = unit_vector(2, 0)
    # # v2 = unit_vector(2, 23.3)
    # # vertices2d = np.array([v1, v2, v1 + v2, np.zeros(2)])
    # # vertices2d_base = np.array([v1, v2])
    # # print(check_if_cover_2d(vertices2d, 0.6365, 100))
    # # print(samples(2, vertices2d_base, 0.6365))
    pass

# See PyCharm help at https://www.jetbrains.com/help/pycharm/

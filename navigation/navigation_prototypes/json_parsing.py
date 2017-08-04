import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_navigation_positions(data):
    """ Reads json file and returns a tuple of three numpy arrays
    corresponding to the x, y, and z values of the navigation data. """

    navigationDataList = []
    for matrix in data["navigationData"]["L"]:
        matrixData = matrix["L"]
        navigationDataList.append([float(item[u'N']) for item in matrixData])
    navigationDataList = np.asarray(navigationDataList)
    return (navigationDataList[:, 12],
        navigationDataList[:, 13],
        navigationDataList[:, 14])

def get_keypoint_positions(data):
    """ Reads json file and returns a tuple of three numpy arrays
    corresponding to the x, y, and z values of the keypoint positions."""

    keypointDataList = []
    for pos in data["keypointData"]["L"]:
        keypointDataList.append([float(item[u'N']) for item in pos[u'L']])
    keypointDataList = np.asarray(keypointDataList)
    return (keypointDataList[:, 0],
        keypointDataList[:, 1],
        keypointDataList[:, 2])

def get_path_positions(data):
    """ Reads json file and returns a tuple of three numpy arrays
    corresponding to the x, y, and z values of the path data. """

    pathDataList = []
    for matrix in data["PathData"]["L"]:
        matrixData = matrix["L"]
        pathDataList.append([float(item[u'N']) for item in matrixData])
    pathDataList = np.asarray(pathDataList)
    return (pathDataList[:, 12],
        pathDataList[:, 13],
        pathDataList[:, 14])

def data_map_birdseye(data):
    """ Given json data, plots path and navigation route
    in 2D, using only the horizontal (X/Z) values. """

    nx, ny, nz = get_navigation_positions(data)
    px, py, pz = get_path_positions(data)
    kx, ky, kz = get_keypoint_positions(data)
    plt.plot(nz, nx)
    plt.plot(pz, px)
    plt.scatter(kz, kx)
    plt.scatter(kz[-1], kx[-1], s=75) # This is the starting point
    plt.show()

def data_map_3d(data):
    """ Given json data, plots the path and navigation
    route in 3D and scales the graph to be more readable."""

    nx, ny, nz = get_navigation_positions(data)
    px, py, pz = get_path_positions(data)
    kx, ky, kz = get_keypoint_positions(data)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(nz, nx, ny)
    ax.plot(pz, px, py)
    ax.scatter(kz, kx, ky)
    x_scale, y_scale, z_scale = iso_scale(nx, ny, nz)
    ax.auto_scale_xyz(z_scale, x_scale, y_scale)
    plt.show()

def iso_scale(xs, ys, zs):
    """ Given three numpy arrays xs, ys, and zs, determines
    the ranges of each that would plot them to scale. """

    range_z = max(zs) - min(zs)
    range_x = max(xs) - min(xs)
    range_y = max(ys) - min(ys)
    max_range = max([range_z, range_x, range_y])
    max_dev = 0.5 * max_range
    avg_z = 0.5 * (max(zs) + min(zs))
    avg_y = 0.5 * (max(ys) + min(ys))
    avg_x = 0.5 * (max(xs) + min(xs))
    return ([avg_x - max_dev, avg_x + max_dev],
        [avg_y - max_dev, avg_y + max_dev],
        [avg_z - max_dev, avg_z + max_dev])

if __name__ == '__main__':
    with open('data.json') as data_file:
        data = json.load(data_file)
    data_map_birdseye(data)
    data_map_3d(data)

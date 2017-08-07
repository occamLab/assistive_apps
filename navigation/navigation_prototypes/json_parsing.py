import json
import sys
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

def get_navigation_times(data):
    """ Returns a list of all times corresponding to the
    navigation path positions. """

    navigationTimeList = []
    for time in data["navigationDataTime"]["L"]:
        navigationTimeList.append(float(time[u'N']))
    navigationTimeList = np.asarray(navigationTimeList)
    return navigationTimeList

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
    points, instructions = get_instructions(data)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # for i, instruction in enumerate(instructions):
    #     x, y, z = points[i-1]
    #     ax.text(z, x, y + 2, str(i), 'x')
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

def nearest_nav_point(data, time):
    """ Finds the index of the closest navigation point to the given time. """

    nx, ny, nz = get_navigation_positions(data)
    nt = get_navigation_times(data)
    dif = abs(nt - time)
    mdif_index, mdif = min(enumerate(dif), key=lambda f:f[1])
    return mdif_index

def get_navigation_point(data, index):
    """ Returns the point of the given index from the navigation path. """

    nx, ny, nz = get_navigation_positions(data)
    return (nx[index], ny[index], nz[index])

def get_instructions(data):
    """ Gets a list of indexes for points along the path instructions were
    given, and a list of all voice instructions for that path. """

    point_indexes = [nearest_nav_point(data, time) \
        for time in get_instruction_times(data)]
    points = [get_navigation_point(data, index) for index in point_indexes]
    instructions = get_voice_text(data)
    return points, instructions

def get_voice_text(data):
    """ Get a list of all voice instructions given for that run. """

    voiceDataList = []
    for string in data["speechData"]["L"]:
        voiceDataList.append(str(string[u'S']))
    voiceDataList = np.asarray(voiceDataList)
    return voiceDataList

def get_instruction_times(data):
    """ Get a list of times that voice instructions were given. """

    instructionTimeList = []
    for time in data["speechDataTime"]["L"]:
        instructionTimeList.append(float(time[u'N']))
    instrucitonTimeList = np.asarray(instructionTimeList)
    return instructionTimeList

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print("No data file selected. Use the following syntax:")
        print("python json_parsing.py data.json")
    else:
        try:
            with open(sys.argv[1]) as data_file:
                data = json.load(data_file)
                data_map_3d(data)
        except IOError:
            print("File '%s' not found.") % sys.argv[1]
        #data_map_birdseye(data)

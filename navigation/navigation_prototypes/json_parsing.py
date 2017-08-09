import json
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import urllib2
from mpl_toolkits.mplot3d import Axes3D

def get_json_data(userId):
    """ Reads json data from url determined from userId. """

    data_file = urllib2.urlopen("https://27bcct7nyg.execute-api.us-east-1.amazonaws.com/Test/pathid/%s" % userId)
    data = json.load(data_file)
    if data[u'Count'] == 0:
        print("No data for that user.")
        return None
    return(get_most_recent_path(data))

def get_most_recent_path(data):
    if len(sys.argv) < 2:
        pathid = data[u'Items'][-1][u'PathID'][u'S'][:-2]
    else:
        pathid = data[u'Items'][-int(sys.argv[2])][u'PathID'][u'S'][:-2]
    ind_list = []
    for ind, item in enumerate(data[u'Items']):
        if pathid in item[u'PathID'][u'S']:
            ind_list.append(ind)
    return data[u'Items'][ind_list[0]:ind_list[-1] + 1]


def get_navigation_positions(data):
    """ Reads json file and returns a tuple of three numpy arrays
    corresponding to the x, y, and z values of the navigation data. """

    navigationDataList = []
    for data_piece in data:
        for matrix in data_piece["navigationData"]["L"]:
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
    for data_piece in data:
        for time in data_piece["navigationDataTime"]["L"]:
            navigationTimeList.append(float(time[u'N']))
    navigationTimeList = np.asarray(navigationTimeList)
    return navigationTimeList

def get_keypoint_positions(data):
    """ Reads json file and returns a tuple of three numpy arrays
    corresponding to the x, y, and z values of the keypoint positions."""

    keypointDataList = []
    for data_piece in data:
        for pos in data_piece["keypointData"]["L"]:
            keypointDataList.append([float(item[u'N']) for item in pos[u'L']])
    keypointDataList = np.asarray(keypointDataList)
    return (keypointDataList[:, 0],
        keypointDataList[:, 1],
        keypointDataList[:, 2])

def get_path_positions(data):
    """ Reads json file and returns a tuple of three numpy arrays
    corresponding to the x, y, and z values of the path data. """

    pathDataList = []
    for data_piece in data:
        for matrix in data_piece["PathData"]["L"]:
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

    get_navigation_positions.data = data
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

def plot_keypoints(data):
    try:
        if animation_run_3d.fig:
            fig = animation_run_3d.fig
            ax = animation_run_3d.ax
        else:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
        kx, ky, kz = get_keypoint_positions(data)
        ax.scatter(kz, kx, ky)
    except IndexError:
        print("ERROR: No keypoint data found.")

def iso_scale(xs, ys, zs):
    """ Given three numpy arrays xs, ys, and zs, determines
    the ranges of each that would plot them to scale. """

    range_z = max(zs) - min(zs)
    range_x = max(xs) - min(xs)
    range_y = max(ys) - min(ys)
    max_range = max([range_z, range_x, range_y])
    max_range *= 1.2
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

    point_indexes = []
    for data_piece in data:
        point_indexes.append([nearest_nav_point(data_piece, time) \
            for time in get_instruction_times(data_piece)])
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

def nav_data_generator():
    data = nav_data_generator.data
    ts = get_navigation_times(data)
    xs = get_navigation_positions(data)[0]
    ys = get_navigation_positions(data)[1]
    zs = get_navigation_positions(data)[2]
    for idx, time in enumerate(ts):
        yield zs[idx], xs[idx]

def path_data_generator():
    data = path_data_generator.data
    xs = get_path_positions(data)[0]
    ys = get_path_positions(data)[1]
    zs = get_path_positions(data)[2]
    for idx, time in enumerate(xs):
        yield zs[idx], xs[idx]

def animation_prep(xyinput):
    ax = animation_prep.ax
    line = animation_prep.line
    # update the data
    t, y = xyinput
    animation_prep.xdata.append(t)
    animation_prep.ydata.append(y)
    xmin, xmax = ax.get_xlim()
    line.set_data(animation_prep.xdata, animation_prep.ydata)
    return line,

def animation_run_3d(data, gen_func):
    try:
        animation_run_3d.fig == True
    except:
        animation_run_3d.fig = plt.figure()
        animation_run_3d.ax = animation_run_3d.fig.add_subplot(111, projection='3d')
    ax = animation_run_3d.ax
    xs, ys, zs = gen_func(data)
    ts = range(0, len(xs))

    xr, yr, zr = iso_scale(xs, ys, zs)
    ax.set_xlim(zr)
    ax.set_ylim(xr)
    ax.set_zlim(yr)

    if gen_func == get_navigation_positions:
        color = 'r'
    elif gen_func == get_path_positions:
        color = 'b'
    else:
        color = 'g'

    # Begin plotting.
    wframe = None
    tstart = time.time()
    scatter = None
    plt.title(determine_phone(data) + ": " + determine_date(data))
    for i, t in enumerate(ts):
        if scatter:
            scatter.remove()
        # If a line collection is already remove it before drawing.
        if i > 0:
        # Plot the new wireframe and pause briefly before continuing.
            plot = ax.plot(zs[i-1:i+1], xs[i-1:i+1], ys[i-1:i+1], color=color)
            scatter = ax.scatter(zs[i], xs[i], ys[i], color=color)
        plt.pause(0.1)

def determine_phone(data):
    id_dict = {"09F2D016-33E8-4FCC-838A-220B8B151328":"Occam Lab iPhone",
        "899ABF35-B36B-4ACF-9D2F-23CF3A3E549B": "Chris's iPhone"}
    user_id = sys.argv[1]
    if user_id in id_dict:
        phone_id = id_dict[user_id]
    else:
        phone_id = "Unknown Device " + user_id[0:4]
    return phone_id

def determine_date(data):
    pathdate = data[0][u'PathDate'][u'S']
    if pathdate == "0":
        print("ERROR: No header found. Determining date from PathID.")
        pathdate = data[0][u'PathID'][u'S'][36:]
    try:
        monthdict = {"01": "January", "02": "February", "03": "March",
            "04": "April", "05": "May", "06": "June", "07": "July", "08": "August",
            "09": "September", "10": "October", "11": "November", "12": "December"}
        month = monthdict[pathdate[5:7]]
        day = str(pathdate[8:10])
        year = str(pathdate[0:4])
        time = str(pathdate[11:16])
        pathdate_text = "%s %s, %s - %s" % (month, day, year, time)
    except:
        print("ERROR: Date information not formatted correctly.")
        pathdate_text = pathdate
    return pathdate_text

def animation_run(data, gen_func):
    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)
    xs, ys, zs = get_navigation_positions(data)
    xscale, yscale, zscale = iso_scale(xs, ys, zs)
    ax.set_ylim(xscale)
    ax.set_xlim(zscale)
    ax.grid()
    animation_prep.xdata, animation_prep.ydata = [], []
    animation_prep.ax = ax
    animation_prep.line = line
    ani = animation.FuncAnimation(fig, animation_prep, gen_func, blit=True, interval=30,
        repeat=False)
    plt.show()
    animation_prep.xdata, animation_prep.ydata = [], []

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print("No data file selected. Use the following syntax:")
        print("python json_parsing.py data.json")
    else:
        try:
            data = get_json_data(sys.argv[1])
            nav_data_generator.data = data
            path_data_generator.data = data
            animation_run_3d(data, get_path_positions)
            plot_keypoints(data)
            animation_run_3d(data, get_navigation_positions)
            plt.show()
            #data_map_3d(data)
        except IOError:
            print("File '%s' not found.") % sys.argv[1]
        #data_map_birdseye(data)

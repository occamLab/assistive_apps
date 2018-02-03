#vertices=[(id, x, y, z), (id, x, y, z)]
#pathinfo=[float pathlength, [list of node ids]]

import numpy as np

def simplify_path(vertices, pathinfo):
    path_vertices = [vertices[idx] for idx in pathinfo[1]]
    points = vertices_to_points(path_vertices)

    node_points = douglas_ramer_peucher_wrapper(points)
    return node_points


def douglas_ramer_peucher_wrapper(points):
    """ Runs the recursive function, but also adds points at the beginning and
    end of list as nodes.

    Input: 3 by X numpy array of 3D points
    Output: 3 by Y numpy array of select points that are nodes in points """

    #TODO make function return the same type it takes

    point_nums = range(0, points.shape[1])
    start_point = np.reshape(points[:, 0], (-1, 1))
    mid_points = np.reshape(douglas_ramer_peucher(points), (3, -1))
    end_point = np.reshape(points[:, point_nums[-1]], (-1, 1))
    list_of_nodes = [start_point, mid_points, end_point]

    #   Remove all instance of null_matrix from the list of nodes to return.
    list_of_nodes = remove_nulls(list_of_nodes)

    return np.concatenate(list_of_nodes, 1)


def douglas_ramer_peucher(points):

    #   Find indexes of first and last points.
    point_nums = range(0, points.shape[1])
    start_point = points[:, 0]
    end_point = points[:, point_nums[-1]]

    #   Set threshold value --- distance points need to be away from
    #   center axis of path to create another node
    threshold = 1.5

    #   Calculate unit vectors to project onto
    total_vec = end_point - start_point
    total_vec_normalized = total_vec/np.linalg.norm(total_vec)
    normal_vec_flat = np.asarray([total_vec[1], -total_vec[0], 0])
    normal_vec_flat_normalized = normal_vec_flat/np.linalg.norm(normal_vec_flat)
    normal_vec_up = np.cross(total_vec_normalized, normal_vec_flat_normalized)

    #   Set values for ditance function
    distance_on_vectors.axis_1 = normal_vec_flat
    distance_on_vectors.axis_2 = normal_vec_up
    distance_on_vectors.points = points - np.asarray([start_point]).transpose()

    #   Find the index of the point with the most distance along unit vectors
    max_idx = max(point_nums, key=distance_on_vectors)

    #   Break recursion if path is fairly straight
    null_matrix = np.asarray([[None], [None], [None]])
    if distance_on_vectors(max_idx) < threshold:
        return null_matrix

    #   Recurse to find all other points before and after maximum
    pre_max_points = np.reshape(douglas_ramer_peucher(points[:, :max_idx+1]), (3, -1))
    max_point = np.reshape(points[:, max_idx], (3, -1))
    post_max_points = np.reshape(douglas_ramer_peucher(points[:, max_idx:]), (3, -1))
    list_of_nodes = [pre_max_points, max_point, post_max_points]

    #   Remove all instance of null_matrix from the list of nodes to return.
    list_of_nodes = remove_nulls(list_of_nodes)

    return np.concatenate(list_of_nodes, 1)


def numpy_vectors_are_equivalent(vec_1, vec_2):
    """ Returns true if each item in vec_1 is the same as the same index item in
    vec_2, and the vectors are the same shape. """

    if vec_1.shape != vec_2.shape:
        return False
    for idx, item in enumerate(vec_1):
        if item != vec_2[idx]:
            return False
    return True


def remove_nulls(list_of_vectors):

    null_matrix = np.asarray([[None], [None], [None]])

    #   Remove all instance of null_matrix from the list of nodes to return.
    progress = False
    while progress == False:
        progress = True
        for i, vector in enumerate(list_of_vectors):
            if numpy_vectors_are_equivalent(vector, null_matrix):
                list_of_vectors = list_of_vectors[:i]+list_of_vectors[i+1:]
                progress = False
                break

    #   Return list, sans nulls
    return list_of_vectors


def project_onto_unit(vector, unit):
    """ Not sure why this is a function. It's just using numpy.dot, really. """

    return np.dot(vector, unit)


def distance_on_vectors(vector_idx):
    """ Inputs: an index to a vector, referring to the column in a numpy array.
    Outputs: the vector's distance onto two arbitrary axes.

    The following values of this function must be set before use:
    axis_1: length 3 numpy array
    axis_2: length 3 numpy array
    points: size 3 by X numpy array """


    #   Project points onto axes and determine distance
    vec = distance_on_vectors.points[:, vector_idx]
    a = project_onto_unit(vec, distance_on_vectors.axis_1)
    b = project_onto_unit(vec, distance_on_vectors.axis_2)

    dist = (a**2 + b**2)**0.5
    return dist

def vertex_to_point(vertex):
    """ Input: vertex tuple in the form of (id, x, y, z)
    Output: vertical numpy array in the form of [x, y, z] """

    vertex_list = [[vertex[idx]] for idx in range(1, 4)]
    return np.asarray(vertex_list)


def vertices_to_points(vertices):
    """ Input: vertices list in the form of [(id, x, y, z), (id, x, y, z)]
    Output: point list as a 3 by X numpy array, with the rows being
        x, y, and z coordinates """

    points = [vertex_to_point(vertex) for vertex in vertices]
    return np.concatenate(points, axis=1)


if __name__ == '__main__':
    #print(douglas_ramer_peucher_wrapper(np.asarray([[3, 6, 9, -10, -29], [1, 2, 3, 10, 17], [1, 1, 1, 1, 1]])))
    pass

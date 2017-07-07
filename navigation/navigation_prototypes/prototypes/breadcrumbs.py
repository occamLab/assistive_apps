import numpy as np
import rospy
from copy import deepcopy

class Breadcrumbs():
    def __init__(self):
        self.path_width = 1.5
        self.crumb_threshold = 3
        self.backtrack = 0

    def calculate_keypoints(self, crumb_list):
        """ Takes in list of points in an x by 3 numpy
        array. Calculates keypoints where path turns """

        keypoints = crumb_list[0, :].reshape(1, 3)
        edible_crumbs = deepcopy(crumb_list)

        while len(edible_crumbs):
            last_keypoint = keypoints[-1, :]
            #print(last_keypoint)
            next_keypoint_index = self.get_next_keypoint(last_keypoint,
                                                        edible_crumbs)
            next_keypoint = edible_crumbs[next_keypoint_index, :]
            edible_crumbs = edible_crumbs[next_keypoint_index + 1:, :]
            keypoints = np.vstack((keypoints, next_keypoint))

        return keypoints

    def get_next_keypoint(self, last_keypoint, edible_crumbs):
        """ Calculates the index of the next keypoint in a path, given a list
        of points and the position of the last keypoint. """

        for index, crumb in enumerate(edible_crumbs):
            current_crumbs = deepcopy(edible_crumbs)
            current_crumbs = current_crumbs[:index + 1, :]

            point_vec = crumb - last_keypoint
            #print(point_vec)
            point_vec = point_vec.T

            #   Rotate point_vec 90 degrees to find normal vector
            norm_vec = np.matmul(np.asarray([[0, 1, 0],
                                            [-1, 0, 0],
                                            [0, 0, 1]]), point_vec)
            unit_norm_vec = norm_vec/np.linalg.norm(norm_vec)
            #print(unit_norm_vec)

            #   TODO make sure that calculating vectors in 3 dimensions
            #   doesn't reduce accuracy

            list_of_distances = np.matmul(current_crumbs - last_keypoint,
                                        unit_norm_vec)
            list_of_distances = np.absolute(list_of_distances)
            list_of_distances = np.sort(list_of_distances)[::-1]

            if self.has_turned(list_of_distances):
                print("TURNED")
                return index - self.backtrack
        return len(edible_crumbs) - 1

    def has_turned(self, list_of_distances):
        #print(list_of_distances)
        counter = 0
        for i, item in enumerate(list_of_distances):
            #print(item)
            if counter >= self.crumb_threshold:
                return True
            elif item > self.path_width:
                counter += 1
            elif i < self.crumb_threshold:
                return False
        return False

if __name__ == '__main__':
    trail = np.asarray([[0, 0, 0],
                        [0, 1, 0],
                        [0, 2, 0],
                        [0, 3, 0],
                        [0, 4, 0],
                        [0, 5, 0],
                        [1, 5, 0],
                        [2, 5, 0],
                        [3, 5, 0],
                        [4, 5, 0],
                        [5, 5, 0],
                        [5, 6, 0],
                        [5, 7, 0],
                        [5, 8, 0],
                        [5, 9, 0],
                        [5, 10, 0]])
    a = Breadcrumbs()
    a.path_width = 0.5
    a.crumb_threshold = 1
    a.backtrack = 1
    print(a.calculate_keypoints(trail))

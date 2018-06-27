#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from os import path
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix, translation_from_matrix
from mobility_games.utils.helper_functions import convert_pose_inverse_transform, convert_translation_rotation_to_pose
import string


class G2O_Viz:
    def __init__(self):
        self.vertices = {}
        self.old_edges = {}
        self.new_edges = {}
        self.transdifference = []
        self.rotdifference = []
        self.vertex_id_start = 587
        self.g2o_result_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data_withnewline_optimized.g2o'

    def GatherData(self):
        """
        Record old edges computed by tango and published to tf. (g2o only change the vertices information, not edge.)
        """
        self.vertices = {}
        self.old_edges = {}
        with open(self.g2o_result_path, 'rb') as g2o_result:
            for line in g2o_result:
                if line.startswith("VERTEX_SE3:QUAT "):
                    contents = line.split()
                    newline = []
                    for data in contents[1:]:
                        newline.append(float(data))
                    if newline[0] >= self.vertex_id_start:
                        self.vertices[int(newline[0])] = (tuple(newline[1:4]), tuple(newline[4:8]))
                        print("found vertex: " + str(newline[0]))
                elif line.startswith("EDGE_SE3:QUAT "):
                    contents = line.split()
                    newline = []
                    for data in contents[1:]:
                        newline.append(float(data))
                    if int(newline[0]) + 2 == int(newline[1]):
                        self.old_edges[int(newline[0])] = (tuple(newline[2:5]), tuple(newline[5:9]))
                        print("found edge: " + str(newline[0]))

    def CalculateNewEdges(self):
        """
        Compute the new edges from updated vertices in result.g2o
        """
        self.new_edges = {}
        ind = self.vertex_id_start
        i = 0
        # print(self.vertices.keys())
        while i < len(self.old_edges.keys()):
            pose = convert_translation_rotation_to_pose(self.vertices[ind][0], self.vertices[ind][1])
            # print(self.vertices[ind][1])
            # print pose
            (trans, rot) = convert_pose_inverse_transform(pose)
            (trans2, rot2) = self.vertices[ind + 2]
            # print(rot)

            T0_1 = quaternion_matrix(rot)
            T0_1[:-1, -1] = np.asarray(trans).T

            T2_0 = quaternion_matrix(rot2)
            T2_0[:-1, -1] = np.asarray(trans2)
            # print T2_0
            # print T0_1

            FinTransform = np.matmul(T0_1, T2_0)
            # print FinTransform

            rot_fin = tuple(quaternion_from_matrix(FinTransform))
            trans_fin = tuple(translation_from_matrix(FinTransform))
            self.new_edges[ind] = (trans_fin, rot_fin)
            # print("comparison: %s" % ind)
            # print(rot_fin)
            # print(self.old_edges[ind][1])
            print ("compute new edge: " + str(ind))
            ind += 2
            i += 1

    def CalculateDifference(self):
        ind = self.vertex_id_start
        i = 0
        # len(self.old_edges.keys())
        self.transdifference = []
        self.rotdifference = []
        while i < len(self.old_edges.keys()):
            transdiff = [self.new_edges[ind][0][j] - self.old_edges[ind][0][j] for j in range(3)]
            # print(transdiff)
            euler_rot0 = euler_from_quaternion(self.old_edges[ind][1])
            euler_rot1 = euler_from_quaternion(self.new_edges[ind][1])
            rotdiff = euler_rot1[2] - euler_rot0[2]

            self.transdifference.append(np.linalg.norm(np.asarray(transdiff)))
            self.rotdifference.append(rotdiff)
            ind += 2
            i += 1

    def run(self):
        self.GatherData()
        self.CalculateNewEdges()
        self.CalculateDifference()
        # print("final info: ")
        # print("translations: %s" % str(self.transdifference))
        # print("rotations: %s" % str(self.rotdifference))
        # Two subplots, the axes array is 1-d
        f, axarr = plt.subplots(2, sharex=True)
        # axarr[0].plot(x, y)
        # axarr[0].set_title('Sharing X axis')
        # axarr[1].scatter(x, y)
        axarr[0].plot(np.arange(len(self.old_edges.keys())), np.asarray(self.transdifference))
        axarr[0].set_ylabel('ChangeInDistance (m)')
        axarr[0].set_title('G2O Correction')
        axarr[0].grid(True)
        axarr[1].plot(np.arange(len(self.old_edges.keys())), np.asarray(self.rotdifference))
        axarr[1].set_xlabel('Time (.1 sec)')
        axarr[1].set_ylabel('ChangeInAngle(rads)')
        axarr[1].grid(True)
        plt.show()


if __name__ == "__main__":
    g2o_viz = G2O_Viz()
    g2o_viz.run()

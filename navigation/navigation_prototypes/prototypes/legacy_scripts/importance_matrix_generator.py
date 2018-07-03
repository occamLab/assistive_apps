#!/usr/bin/env python

#import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import matplotlib.patches as mpatches
#import math
import rospy
from os import system
#import re
import tf#.transformations import euler_from_quaternion as efq
#from tf#.transformations import quaternion_from_euler as qfe
from mobility_games.utils.helper_functions import convert_pose_inverse_transform, convert_translation_rotation_to_pose, invert_transform_2

class Importance_Generator:
    def __init__(self):
        #rospy.init_node('importance_matrix_generator')
        #self.vertices = {}
        #self.old_vertices = {}
        #self.old_edges = {}
        #self.new_edges = {}
        #self.old_AR = {}
        #self.new_AR = {}
        #self.transdifference = []
        #self.rotdifference = []
        self.g2o_result_path = '/home/juicyslew/catkin_ws/result_edited.g2o'
        self.g2o_data_path = '/home/juicyslew/catkin_ws/data_cp.g2o'
        self.g2o_edited_path = '/home/juicyslew/catkin_ws/data_edited.g2o'
        self.importance_inds = [10, 16, 21, 25, 28, 30]
        self.importance_val_AR = [100, 100, 100, 100, 100, 100]
        self.importance_val_pose = [1,1,1,1,1,1]
    def Create_Edited_Data(self):
        #self.vertices = {}
        #self.old_edges = {}
        #self.old_vertices = {}
        """with open(self.g2o_result_path, 'rb') as g2o_result:
            for line in g2o_result:
                if line.startswith("VERTEX_SE3:QUAT "):
                    line = line.strip()
                    line = line.split(' ')
                    line = [float(i) for i in line[1:]]
                    if line[0] >= 587:
                        self.vertices[int(line[0])] = (tuple(line[1:4]), tuple(line[4:8]))
                        #print("found vertex: " + str(line[0]))
                    else:
                        self.new_AR[int(line[0])] = (tuple(line[1:4]), tuple(line[4:8]))
                        print("found tag: " + str(line[0]))
                elif line.startswith("EDGE_SE3:QUAT "):
                    line = line.strip()
                    line = line.split(' ')
                    line = [float(i) for i in line[1:]]
                    if int(line[0]) + 1 == int(line[1]):
                        self.old_edges[int(line[0])] = (tuple(line[2:5]), tuple(line[5:9]))
                        print("found edge: " + str(line[0]))"""
        with open(self.g2o_data_path, 'rb') as g2o_data:
            with open(self.g2o_edited_path, 'wb') as edited_data:
                for line in g2o_data:
                    if line.startswith("EDGE_SE3:QUAT "):
                        line = line.strip()
                        line = line.split(' ')
                        if int(line[2]) < 587:
                            j = 0
                            for i in self.importance_inds:
                                line[i] = str(self.importance_val_AR[j])
                                j+=1
                        else:
                            j = 0
                            for i in self.importance_inds:
                                line[i] = str(self.importance_val_pose[j])
                                j+=1
                        line = (" ").join(line)
                        line += "\n"
                        print line
                        edited_data.write(line)
                    else:
                        edited_data.write(line)
                        #line = [float(i) for i in line[1:]]
                        """if int(line[0]) + 1 == int(line[1]):
                            self.old_edges[int(line[0])] = (tuple(line[2:5]), tuple(line[5:9]))
                            print("found edge: " + str(line[0]))"""
                        """line = line.strip()
                        line = line.split(' ')
                        line = [float(i) for i in line[1:]]
                    if line[0] >= 587:
                        self.old_vertices[int(line[0])] = (tuple(line[1:4]), tuple(line[4:8]))
                        #print("found vertex: " + str(line[0]))
                    else:
                        self.old_AR[int(line[0])] = (tuple(line[1:4]), tuple(line[4:8]))
                        print("found tag: " + str(line[0]))"""
    """def CalculateNewEdges(self):
        self.new_edges = {}
        ind = 587
        i = 0
        #print(self.vertices.keys())
        while i < len(self.old_edges.keys()):
            pose = convert_translation_rotation_to_pose(self.vertices[ind][0], self.vertices[ind][1])
            #print(self.vertices[ind][1])
            #print pose
            (trans, rot) = convert_pose_inverse_transform(pose)
            (trans2, rot2) = self.vertices[ind+1]
            #print(rot)

            T0_1 = tf.transformations.quaternion_matrix(rot)
            T0_1[:-1, -1] = np.asarray(trans).T

            T2_0 = tf.transformations.quaternion_matrix(rot2)
            T2_0[:-1, -1] = np.asarray(trans2)
            #print T2_0
            #print T0_1

            FinTransform = np.matmul(T0_1, T2_0)
            #print FinTransform

            rot_fin = tuple(tf.transformations.quaternion_from_matrix(FinTransform))
            trans_fin = tuple(tf.transformations.translation_from_matrix(FinTransform))
            self.new_edges[ind] = (trans_fin, rot_fin)
            #print("comparison: %s" % ind)
            #print(rot_fin)
            #print(self.old_edges[ind][1])

            ind += 1
            i += 1
    def Calculate_Old_Vertices(self):
        # TODO make function for finding the old vertex locations based on the old edges.
        ind = 587
        i = 0
        #print(self.vertices.keys())
        fixpoint = self.vertices[0]
        while i < len(self.old_edges.keys()):
            pose = convert_translation_rotation_to_pose(self.old_edges[ind], self.vertices[ind])
            #print(self.vertices[ind][1])
            #print pose
            (trans, rot) = convert_pose_inverse_transform(pose)
            (trans2, rot2) = self.vertices[ind+1]
            #print(rot)

            T0_1 = tf.transformations.quaternion_matrix(rot)
            T0_1[:-1, -1] = np.asarray(trans).T

            T2_0 = tf.transformations.quaternion_matrix(rot2)
            T2_0[:-1, -1] = np.asarray(trans2)
            #print T2_0
            #print T0_1

            FinTransform = np.matmul(T0_1, T2_0)
            #print FinTransform

            rot_fin = tuple(tf.transformations.quaternion_from_matrix(FinTransform))
            trans_fin = tuple(tf.transformations.translation_from_matrix(FinTransform))
            self.new_edges[ind] = (trans_fin, rot_fin)
            #print("comparison: %s" % ind)
            #print(rot_fin)
            #print(self.old_edges[ind][1])

            ind += 1
            i += 1
        pass"""

    """def CalculateDifference(self):
        ind = 587
        i = 0
        #len(self.old_edges.keys())
        self.transdifference = []
        self.rotdifference = []
        while i < len(self.old_edges.keys()):
            transdiff = [self.new_edges[ind][0][j] - self.old_edges[ind][0][j] for j in range(3)]
            #print(transdiff)
            euler_rot0 = tf.transformations.euler_from_quaternion(self.old_edges[ind][1])
            euler_rot1 = tf.transformations.euler_from_quaternion(self.new_edges[ind][1])
            rotdiff = euler_rot1[2] - euler_rot0[2]

            self.transdifference.append(np.linalg.norm(np.asarray(transdiff)))
            self.rotdifference.append(rotdiff)
            ind += 1
            i += 1"""
    def execute_g2o(self):
        system("g2o -o %s %s" % (self.g2o_result_path, self.g2o_edited_path))


    def run(self):
        self.Create_Edited_Data()
        self.execute_g2o()

if __name__ == "__main__":
    matGen = Importance_Generator()
    matGen.run()

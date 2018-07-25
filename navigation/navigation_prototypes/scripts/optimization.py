#!/usr/bin/env python

from pose_graph import PoseGraph, Vertex, Edge
import pickle
from os import path
import numpy as np
from rospkg import RosPack
import rospy
import geometry_msgs.msg
import tf
from tf.transformations import quaternion_matrix, translation_from_matrix, quaternion_from_matrix, \
    euler_from_quaternion, quaternion_from_euler
from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose)
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import seaborn as sns


class Optimization:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.raw_data_folder = path.join(self.package, 'data/raw_data')
        self.processed_data_folder = path.join(self.package, 'data/processed_data')
        self.optimized_data_folder = path.join(self.package, 'data/optimized_data')
        with open(path.join(self.raw_data_folder, filename), 'rb') as data:
            self.posegraph = pickle.load(data)  # pose graph that will be optimized
        with open(path.join(self.raw_data_folder, filename), 'rb') as data:
            data.seek(0)
            self.unoptimzied_posegraph = pickle.load(data)
        self.g2o_result_path = self.posegraph.g2o_result_path

        #### Interpretation of optimized data ####
        self.optimized_pose = None
        self.unoptimized_pose = None
        self.optimized_tag = None
        self.xmax = []
        self.xmin = []
        self.ymax = []
        self.ymin = []
        self.zmax = []
        self.zmin = []

    def g2o(self):
        self.posegraph.process_graph()
        with open(path.join(self.processed_data_folder, "data_processed.pkl"), 'wb') as f:
            pickle.dump(self.posegraph, f)
            print "PROCESSED POSE GRAPH SAVED"
        self.posegraph.optimize_pose()

    @staticmethod
    def is_integer(num):
        try:
            int(num)
            return True
        except ValueError:
            return False

    @staticmethod
    def update_transformation(dataset, id, translation, rotation):
        dataset[id].translation = translation
        dataset[id].rotation = rotation

    def update_vertices(self):
        with open(self.g2o_result_path, 'rb') as g2o_result:
            for line in g2o_result:
                if line.startswith("VERTEX_SE3:QUAT "):
                    line = line.strip().split()
                    translation = [float(data) for data in line[2:5]]
                    rotation = [float(data) for data in line[5:9]]
                    if Optimization.is_integer(line[1]) and int(line[1]) <= self.posegraph.num_tags:
                        id = int(line[1])
                        Optimization.update_transformation(self.posegraph.tag_vertices, id, translation, rotation)
                    elif Optimization.is_integer(line[1]) and int(line[1]) > self.posegraph.num_tags:
                        id = int(line[1])
                        Optimization.update_transformation(self.posegraph.odometry_vertices, id, translation, rotation)
                    else:
                        id = line[1]
                        Optimization.update_transformation(self.posegraph.waypoints_vertices, id, translation, rotation)

    def query_trans_rot_from_calibrated_vertices(self, id):
        """
        Retrun translation, rotation from the corresponding vertices dictionary given a vertex id
        """
        if Optimization.is_integer(id) and int(id) <= self.posegraph.num_tags:
            translation = self.posegraph.tag_vertices[id].translation
            rotation = self.posegraph.tag_vertices[id].rotation
        elif Optimization.is_integer(id) and int(id) > self.posegraph.num_tags:
            translation = self.posegraph.odometry_vertices[id].translation
            rotation = self.posegraph.odometry_vertices[id].rotation
        else:
            translation = self.posegraph.waypoints_vertices[id].translation
            rotation = self.posegraph.waypoints_vertices[id].rotation
        return translation, rotation

    def update_edges_vertices_from_calibration(self, edges):
        for id in edges.keys():
            start_id = edges[id].start.id
            trans_start, rot_start = self.query_trans_rot_from_calibrated_vertices(start_id)
            edges[id].start.translation = trans_start
            edges[id].start.rotation = rot_start
            end_id = edges[id].end.id
            trans_end, rot_end = self.query_trans_rot_from_calibrated_vertices(end_id)
            edges[id].end.translation = trans_end
            edges[id].end.rotation = rot_end

    def update_all_edges_vertices(self):
        for odom_start_id in self.posegraph.odometry_edges.keys():
            self.update_edges_vertices_from_calibration(self.posegraph.odometry_edges[odom_start_id])
        for tag_id in self.posegraph.odometry_tag_edges.keys():
            self.update_edges_vertices_from_calibration(self.posegraph.odometry_tag_edges[tag_id])
        for waypoint_id in self.posegraph.odometry_waypoints_edges.keys():
            self.update_edges_vertices_from_calibration(self.posegraph.odometry_waypoints_edges[waypoint_id])

    @staticmethod
    def compute_new_edges_math(edges):
        for id in edges.keys():
            # write start vertex as a transformation
            start_trans = edges[id].start.translation
            start_rot = edges[id].start.rotation
            start_pose = convert_translation_rotation_to_pose(start_trans, start_rot)
            start_trans_inv, start_rot_inv = convert_pose_inverse_transform(start_pose)
            start_transformation = quaternion_matrix(start_rot_inv)
            start_transformation[:-1, -1] = np.array(start_trans_inv).T
            # write end vertex as a transformation
            end_trans = edges[id].end.translation
            end_rot = edges[id].end.rotation
            end_transformation = quaternion_matrix(end_rot)
            end_transformation[:-1, -1] = np.array(end_trans).T
            # compute transformation between two vertices
            start_end_transformation = np.matmul(end_transformation, start_transformation)
            translation = translation_from_matrix(start_end_transformation)
            rotation = quaternion_from_matrix(start_end_transformation)
            edges[id].translation_computed = list(translation)
            edges[id].rotation_computed = list(rotation)

    def compute_all_new_edges_math(self):
        for odom_start_id in self.posegraph.odometry_edges.keys():
            Optimization.compute_new_edges_math(self.posegraph.odometry_edges[odom_start_id])
        for tag_id in self.posegraph.odometry_tag_edges.keys():
            Optimization.compute_new_edges_math(self.posegraph.odometry_tag_edges[tag_id])
        for waypoint_id in self.posegraph.odometry_waypoints_edges.keys():
            Optimization.compute_new_edges_math(self.posegraph.odometry_waypoints_edges[waypoint_id])

    @staticmethod
    def write_transform_stamped_msg(transformer, id, trans, rot, parent):
        frame = geometry_msgs.msg.TransformStamped()
        frame.header.frame_id = parent
        frame.child_frame_id = id
        frame.transform.translation.x = trans[0]
        frame.transform.translation.y = trans[1]
        frame.transform.translation.z = trans[2]
        frame.transform.rotation.x = rot[0]
        frame.transform.rotation.y = rot[1]
        frame.transform.rotation.z = rot[2]
        frame.transform.rotation.w = rot[3]
        transformer.setTransform(frame)

    @staticmethod
    def compute_new_edges_transformer(edges):
        for id in edges.keys():
            transform = tf.Transformer(True, rospy.Duration(10))
            start_trans = edges[id].start.translation
            start_rot = edges[id].start.rotation
            end_trans = edges[id].end.translation
            end_rot = edges[id].end.rotation
            Optimization.write_transform_stamped_msg(transform, "start", start_trans, start_rot, "map")
            Optimization.write_transform_stamped_msg(transform, "end", end_trans, end_rot, "map")
            translation, rotation = transform.lookupTransform("start", "end", rospy.Time(0))
            edges[id].translation_computed = list(translation)
            edges[id].rotation_computed = list(rotation)

    def compute_all_new_edges_transformer(self):
        for odom_start_id in self.posegraph.odometry_edges.keys():
            Optimization.compute_new_edges_transformer(self.posegraph.odometry_edges[odom_start_id])
        for tag_id in self.posegraph.odometry_tag_edges.keys():
            Optimization.compute_new_edges_transformer(self.posegraph.odometry_tag_edges[tag_id])
        for waypoint_id in self.posegraph.odometry_waypoints_edges.keys():
            Optimization.compute_new_edges_transformer(self.posegraph.odometry_waypoints_edges[waypoint_id])

    def update_edges_math(self):
        self.update_all_edges_vertices()
        self.compute_all_new_edges_math()

    def update_edges_transformer(self):
        self.update_all_edges_vertices()
        self.compute_all_new_edges_transformer()

    @staticmethod
    def compute_edges_transformation_difference(edges):
        for id in edges.keys():
            trans_old = edges[id].translation
            trans_new = edges[id].translation_computed
            edges[id].translation_diff = [trans_new[i] - trans_old[i] for i in range(3)]
            rot_old = edges[id].rotation
            rot_old_euler = euler_from_quaternion(rot_old)
            rot_new = edges[id].rotation_computed
            rot_new_euler = euler_from_quaternion(rot_new)
            rot_diff_euler = [rot_new_euler[i] - rot_old_euler[i] for i in range(3)]
            edges[id].rotation_diff = list(
                quaternion_from_euler(rot_diff_euler[0], rot_diff_euler[1], rot_diff_euler[2]))

    def compute_all_edges_transformation_difference(self):
        for odom_start_id in self.posegraph.odometry_edges.keys():
            Optimization.compute_edges_transformation_difference(self.posegraph.odometry_edges[odom_start_id])
        for tag_id in self.posegraph.odometry_tag_edges.keys():
            Optimization.compute_edges_transformation_difference(self.posegraph.odometry_tag_edges[tag_id])
        for waypoint_id in self.posegraph.odometry_waypoints_edges.keys():
            Optimization.compute_edges_transformation_difference(self.posegraph.odometry_waypoints_edges[waypoint_id])

    def parse_g2o_result_math(self):
        self.update_vertices()
        self.update_edges_math()
        self.compute_all_edges_transformation_difference()
        with open(path.join(self.optimized_data_folder, "data_optimized.pkl"), 'wb') as f:
            pickle.dump(self.posegraph, f)
            print "OPTIMIZED POSE GRAPH SAVED"

    def parse_g2o_result_transformer(self):
        self.update_vertices()
        self.update_edges_transformer()
        self.compute_all_edges_transformation_difference()
        with open(path.join(self.optimized_data_folder, "data_optimized.pkl"), 'wb') as f:
            pickle.dump(self.posegraph, f)
            print "OPTIMIZED POSE GRAPH SAVED"

    @staticmethod
    def MultiplyTransform(tr1, tr2):
        T = quaternion_matrix(tr1[1])
        T[:-1, -1] = np.squeeze(np.asarray(tr1[0]))

        T2 = quaternion_matrix(tr2[1])
        T2[:-1, -1] = np.squeeze(np.asarray(tr2[0]))

        Tres = np.matmul(T, T2)
        trans = translation_from_matrix(Tres)
        rot = quaternion_from_matrix(Tres)

        return trans, rot

    @staticmethod
    def process_vertices_for_plot(vertices):
        optimized_vertices = []
        for key in sorted(vertices):
            if vertices[key].translation != [0, 0, 0]:  # remove dummy vertices
                optimized_vertices.append(vertices[key].translation)
        optimized_vertices = np.array(optimized_vertices)
        return optimized_vertices

    def plot_extreme_vertices_for_pose(self, ax):
        all_translations = []
        for id in self.posegraph.odometry_vertices.keys():
            translation = self.posegraph.odometry_vertices[id].translation
            all_translations.append((id, translation[0], translation[1], translation[2]))
        trans_sort_x = sorted(all_translations, key=lambda vertex: vertex[1])
        trans_sort_y = sorted(all_translations, key=lambda vertex: vertex[2])
        trans_sort_z = sorted(all_translations, key=lambda vertex: vertex[3])
        self.xmax = trans_sort_x[-1]
        self.xmin = trans_sort_x[0]
        self.ymax = trans_sort_y[-1]
        self.ymin = trans_sort_y[0]
        self.zmax = trans_sort_z[-1]
        self.zmin = trans_sort_z[0]
        for extreme_pt in [self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax]:
            print "id:", extreme_pt[0]
            plt.plot((extreme_pt[1],), (extreme_pt[2],), (extreme_pt[3],), 'rx')
            ax.text(extreme_pt[1], extreme_pt[2], extreme_pt[3], str(extreme_pt[0]))

    @staticmethod
    def plot_landmarks(ax, landmarks):
        for row in range(np.shape(landmarks)[0]):
            landmark = landmarks[row, :]
            plt.plot((landmark[0],), (landmark[1],), (landmark[2],), 'go')
            ax.text(landmark[0], landmark[1], landmark[2], str(row))

    def plot_g2o_trajectory(self):
        self.optimized_pose = Optimization.process_vertices_for_plot(self.posegraph.odometry_vertices)
        self.unoptimized_pose = Optimization.process_vertices_for_plot(self.unoptimzied_posegraph.odometry_vertices)
        self.optimized_tag = Optimization.process_vertices_for_plot(self.posegraph.tag_vertices)
        fig = plt.figure()
        ax = p3.Axes3D(fig)

        # plot optimized pose vertices
        optimized_pose, = plt.plot(self.optimized_pose[:, 0], self.optimized_pose[:, 1], self.optimized_pose[:, 2], 'b',
                                   label='corrected trajectory')

        # plot unoptimized pose vertices
        unoptimized_pose, = plt.plot(self.unoptimized_pose[:, 0], self.unoptimized_pose[:, 1],
                                     self.unoptimized_pose[:, 2], 'm--', label='uncorrected trajectory')

        # plot extreme pose vertices
        self.plot_extreme_vertices_for_pose(ax)

        # plot tag vertices
        Optimization.plot_landmarks(ax, self.optimized_tag)

        plt.legend(handles=[optimized_pose, unoptimized_pose])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('G2O Trajectory Plot')
        plt.grid(True)
        plt.show()

    def run(self):
        self.g2o()
        self.parse_g2o_result_transformer()
        # self.parse_g2o_result_math()
        self.plot_g2o_trajectory()


if __name__ == "__main__":
    process = Optimization("data_collected.pkl")
    process.run()

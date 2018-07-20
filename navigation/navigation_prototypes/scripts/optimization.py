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


class Optimization:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.raw_data_folder = path.join(self.package, 'data/raw_data')
        self.processed_data_folder = path.join(self.package, 'data/processed_data')
        self.optimized_data_folder = path.join(self.package, 'data/optimized_data')
        with open(path.join(self.raw_data_folder, filename), 'rb') as data:
            self.posegraph = pickle.load(data)
        self.g2o_result_path = self.posegraph.g2o_result_path

    def g2o(self):
        self.posegraph.process_graph()
        with open(path.join(self.processed_data_folder, "data_processed.pkl"), 'wb') as file:
            pickle.dump(self.posegraph, file)
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
                    rotation = [float(data) for data in line[5:8]]
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
            end_id = edges[id].start.id
            trans_end, rot_end = self.query_trans_rot_from_calibrated_vertices(end_id)
            edges[id].end.translation = trans_end
            edges[id].end.rotation = rot_end

    def update_all_edges_vertices(self):
        self.update_edges_vertices_from_calibration(self.posegraph.odometry_edges)
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
        Optimization.compute_new_edges_math(self.posegraph.odometry_edges)
        for tag_id in self.posegraph.odometry_tag_edges.keys():
            Optimization.compute_new_edges_math(self.posegraph.odometry_tag_edges[tag_id])
        for waypoint_id in self.posegraph.odometry_waypoints_edges.keys():
            Optimization.compute_new_edges_math(self.posegraph.odometry_waypoints_edges[waypoint_id])

    @staticmethod
    def write_transform_stamped_msg(id, trans, rot, parent):
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
        return frame

    @staticmethod
    def compute_new_edges_transformer(edges):
        for id in edges.keys():
            transform = tf.Transformer(True, rospy.Duration(10))
            start_trans = edges[id].start.translation
            start_rot = edges[id].start.rotation
            end_trans = edges[id].end.translation
            end_rot = edges[id].end.rotation
            start = Optimization.write_transform_stamped_msg("start", start_trans, start_rot, "map")
            end = Optimization.write_transform_stamped_msg("end", end_trans, end_rot, "map")
            transform.setTransform(start)
            transform.setTransform(end)
            translation, rotation = transform.lookupTransform(start, end, rospy.Time(0))
            edges[id].translation_computed = list(translation)
            edges[id].rotation_computed = list(rotation)

    def compute_all_new_edges_transformer(self):
        Optimization.compute_new_edges_transformer(self.posegraph.odometry_edges)
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
        Optimization.compute_edges_transformation_difference(self.posegraph.odometry_edges)
        for tag_id in self.posegraph.odometry_tag_edges.keys():
            Optimization.compute_edges_transformation_difference(self.posegraph.odometry_tag_edges[tag_id])
        for waypoint_id in self.posegraph.odometry_waypoints_edges.keys():
            Optimization.compute_edges_transformation_difference(self.posegraph.odometry_waypoints_edges[waypoint_id])

    def parse_g2o_result_math(self):
        self.update_vertices()
        self.update_edges_math()
        self.compute_all_edges_transformation_difference()
        with open(path.join(self.optimized_data_folder, "data_optimized.pkl"), 'wb') as file:
            pickle.dump(self.posegraph, file)

    def parse_g2o_result_transformer(self):
        self.update_vertices()
        self.update_edges_transformer()
        self.compute_all_edges_transformation_difference()
        with open(path.join(self.optimized_data_folder, "data_optimized.pkl"), 'wb') as file:
            pickle.dump(self.posegraph, file)

    def plot_g2o_result(self):
        pass

    def run(self):
        self.g2o()
        self.parse_g2o_result_transformer()
        # self.parse_g2o_result_math()


if __name__ == "__main__":
    process = Optimization("data_collected.pkl")
    process.run()

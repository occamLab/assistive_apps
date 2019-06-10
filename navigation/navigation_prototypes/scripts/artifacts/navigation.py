#!/usr/bin/env python

"""
First pass prototype of a navigation class for the Invisible Map. This loads a
Pose Graph after the data collection and optimization process is complete.

Informed by previous work:
    - invisible_map_updated.py (assistive_apps, summer2018 branch)
    - pathfinder2.py (assistive_apps, Multiple_AR branch)
    - semantic_waypoints.py (mobility_games, summer2018 branch)

**Development Process and Tests:**

[x] Load a calibrated pose graph successfully
    - Test: be able to read and manipulate a graph as expected
    - Viz Test: show some visualization of the graph
[ ] Successfully localize current position within a loaded pose graph
    - Test: stand in known position and have the system also recognize this position correctly within the graph
    - Viz Test: show a correct visualization of the user's current position in the graph
[ ] Generate a hardcoded simple path for a user to navigate along
    - Test: Given the localized position and a hardcoded goal along the loaded graph, generate a path
    - Viz Test: show a visualization of the current location, goal, and path generated
[ ] Create simple interface to navigate user
    - Test: Ask user for a goal location, then generate a path to that location, and give directions for them to get there
    - Viz Test: (same visualization as previous step, this is only creating a basic UI)
[ ] Generate more complex paths (using junctions and search algorithms) for a user to navigate along
    - Test: Ask user for a goal location not along the graph's predetermined paths, generate that path, and give directions
    - Viz Test: visualize junctions within the loaded graph, and keep same current location/goal/path from before
[ ] Test! A whole lot!
"""

import rospy
from rospkg import RosPack
from os import path
import pickle
from pose_graph import PoseGraph, Vertex, Edge
import numpy as np
# import geometry_msgs.msg
# import tf
# from tf.transformations import quaternion_matrix, translation_from_matrix, quaternion_from_matrix, \
#     euler_from_quaternion, quaternion_from_euler
# from helper_functions import (convert_pose_inverse_transform,
#                               convert_translation_rotation_to_pose)
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

class Navigation(object):
    """
    Navigation class for invisible map: initializing and running this will
    start the navigation sequence, given an already calibrated and optimized map.
    """
    def __init__(self, filepath='raw_data/data_collected.pkl'):
        """
        Initializes variables and loads pose graph from filepath argument.
        Args:
            filepath: string specifying a full filename and its path (from within the data folder).
        Note:
            /data is (currently) in the gitignore, so make sure to set that up
            and include a data file before attempting to run this.
        """
        #ROS Initialization
        # rospy.init_node('navigation')

        #Load data graph.
        self.ros_package_path = RosPack().get_path('navigation_prototypes')  # Directory for this ros package
        with open(path.join(self.ros_package_path,'data/',filepath), 'rb') as data:
            self.posegraph = pickle.load(data)

        #### Interpretation of optimized data - from Optimization.py ####
        self.optimized_pose = None
        self.optimized_tag = None
        self.optimized_waypoint = None
        self.index_to_tag_conversion = {} #not sure this is ever modified?
        self.index_to_waypoint_conversion = {} #not sure this is ever modified?

    def plot_g2o_trajectory(self):
        """pulled from optimization.py. Modified slightly to display only the
        optimized path (not the original path)."""
        self.optimized_pose = Navigation.process_vertices_for_plot(self.posegraph.odometry_vertices)
        self.optimized_tag = Navigation.process_vertices_for_plot(self.posegraph.tag_vertices)
        self.optimized_waypoint = Navigation.process_vertices_for_plot(self.posegraph.waypoints_vertices)
        fig = plt.figure()
        ax = p3.Axes3D(fig)

        # plot optimized pose vertices
        pose, = plt.plot(self.optimized_pose[:, 0], self.optimized_pose[:, 1], self.optimized_pose[:, 2],
                                   'bo',
                                   label='corrected trajectory')

        # plot extreme pose vertices
        self.plot_extreme_vertices_for_pose(ax)

        # plot tag vertices
        Navigation.map_index_to_landmark_id(self.posegraph.tag_vertices, self.index_to_tag_conversion)
        Navigation.plot_landmarks(ax, self.optimized_tag, self.index_to_tag_conversion)

        # plot waypoint vertices
        Navigation.map_index_to_landmark_id(self.posegraph.waypoints_vertices, self.index_to_waypoint_conversion)
        Navigation.plot_landmarks(ax, self.optimized_waypoint, self.index_to_waypoint_conversion)

        plt.legend(handles=[pose])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('G2O Trajectory Plot')
        plt.grid(True)
        plt.show()

    @staticmethod
    def map_index_to_landmark_id(landmarks, stored_loc):
        """pulled directly from optimization.py"""
        sorted_landmarks_id = sorted(landmarks.keys())
        for landmark_id in sorted(landmarks):
            id_index = sorted_landmarks_id.index(landmark_id)
            stored_loc[id_index] = landmark_id

    @staticmethod
    def process_vertices_for_plot(vertices):
        """pulled directly from optimization.py"""
        optimized_vertices = []
        for key in sorted(vertices):
            if vertices[key].translation != [0, 0, 0]:  # remove dummy vertices
                optimized_vertices.append(vertices[key].translation)
        optimized_vertices = np.array(optimized_vertices)
        return optimized_vertices

    def plot_extreme_vertices_for_pose(self, ax):
        """pulled directly from optimization.py"""
        all_translations = []
        for id in self.posegraph.odometry_vertices.keys():
            if not self.posegraph.odometry_vertices[id].fix_status:  # not including dummy vertices
                translation = self.posegraph.odometry_vertices[id].translation
                all_translations.append((id, translation[0], translation[1], translation[2]))
        trans_sort_x = sorted(all_translations, key=lambda vertex: vertex[1])
        trans_sort_y = sorted(all_translations, key=lambda vertex: vertex[2])
        trans_sort_z = sorted(all_translations, key=lambda vertex: vertex[3])
        xmin, xmax = trans_sort_x[0], trans_sort_x[-1]
        ymin, ymax = trans_sort_y[0], trans_sort_y[-1]
        zmin, zmax = trans_sort_z[0], trans_sort_z[-1]
        for extreme_pt in [xmin, xmax, ymin, ymax, zmin, zmax]:
            print "id:", extreme_pt[0]
            plt.plot((extreme_pt[1],), (extreme_pt[2],), (extreme_pt[3],), 'rx')
            ax.text(extreme_pt[1], extreme_pt[2], extreme_pt[3], str(extreme_pt[0]))

    @staticmethod
    def plot_landmarks(ax, landmarks, landmarks_index_to_id):
        """pulled directly from optimization.py"""
        for row in range(np.shape(landmarks)[0]):
            landmark = landmarks[row, :]
            plt.plot((landmark[0],), (landmark[1],), (landmark[2],), 'go')
            ax.text(landmark[0], landmark[1], landmark[2], str(landmarks_index_to_id[row]))

    def run(self):
        """
        potential end to end process:
            - load data
            - meld junctions and create better paths
            - start navigation process with user
                - allow input of goal
                - somehow localize within graph
                - start navigation
                - potentially announce things as they pass by
                - finish navigation when
        """
        ### Loading Graph ###
        # print 'number of tags in loaded graph:',self.posegraph.num_tags
        # print 'origin tag:',self.posegraph.origin_tag
        # print 'waypoints:',self.posegraph.waypoints

        ### Investigating one waypoint vertex ###
        # robolab_wp = self.posegraph.waypoints['robolab']
        # # print 'robolab id:', robolab_wp.id
        # print 'robolab waypoint type:',type(robolab_wp)
        # print 'robolab translation:',robolab_wp.translation, type(robolab_wp.translation)
        # #translation is list of (x,y,z)
        # #rotation is a list of (x,y,z,w)
        # print 'robolab rotation:',robolab_wp.rotation, type(robolab_wp.rotation)
        # print 'robolab type:',robolab_wp.type
        # print 'robolab fix status:',robolab_wp.fix_status

        ### First attempt at pulling over Optimization's visualization ###
        self.plot_g2o_trajectory()

if __name__ == '__main__':
    navigation = Navigation()
    navigation.run()

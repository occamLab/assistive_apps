#!/usr/bin/env python

import rospy
from rospkg import RosPack
import tf
from std_msgs.msg import Int32, Bool, Header
from geometry_msgs.msg import PoseStamped

import pickle
from os import path
from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose)
from pose_graph import PoseGraph


class Frames:

    def __init__(self):
        rospy.init_node('AR_frames')
        self.package = RosPack().get_path('navigation_prototypes')  # Directory for this ros package
        self.data_folder = path.join(self.package, 'prototypes/raw_data')

        #### Transform Parameters ####
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.map_frame = "old_odom"
        self.odom_to_map = None

        ### pose graph ###
        self.first_tag = None  # first tag seen during data collection
        self.pose_graph = None
        self.translations = {}
        self.rotations = {}

        rospy.Subscriber('/first_tag', Int32, self.first_tag_callback)
        rospy.Subscriber('/map', Bool, self.map_frame_callback)

    def first_tag_callback(self, msg):
        self.first_tag = msg

    def map_frame_callback(self, msg):
        if msg:  # load pose graph
            with open(path.join(self.data_folder, "data_collected.pkl"), 'rb') as f:  # read pose graph file
                self.pose_graph = pickle.load(f)  # load pickle
                print("Posegraph LOADED")

    def update_odom_AR_transform(self):
        try:
            tag_frame = "tag_" + str(self.first_tag)
            self.listener.waitForTransform("odom", tag_frame, rospy.Time(0), rospy.Duration(0.5))
            if self.listener.canTransform("odom", tag_frame, rospy.Time(0)):
                trans, rot = self.listener.lookupTransform("odom", tag_frame, rospy.Time(0))
                self.translations[self.first_tag] = trans
                self.rotations[self.first_tag] = rot

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException) as e:
            print "ODOM_AR TRANSFORM EXCEPTION:", e

    def broadcast_odom_AR_transform(self):
        """ Will broadcast the transform between parent node
        odom with AR as the direct child """

        try:
            self.broadcaster.sendTransform(self.translations[self.first_tag], self.rotations[self.first_tag],
                                           rospy.Time.now(), "AR",  # child
                                           "odom")  # parent
        except (KeyError) as e:
            print "BROADCAST ERROR:", e

    def
    def compute_map_to_odom_transform(self, map_trans, map_rot):
        map_pose = PoseStamped(pose=convert_translation_rotation_to_pose(map_trans, map_rot),
                                    header=Header(stamp=rospy.Time.now(), frame_id="AR")) # frame_id: frame the pose is in
        self.listener.waitForTransform("AR", "odom", rospy.Time(0), rospy.Duration(1))
        self.odom_to_map = self.listener.transformPose("odom", map_pose)
        translation, rotation = convert_pose_inverse_transform(self.odom_to_map.pose) # put odom in new odom
        return translation, rotation

    def update_map_odom_transform(self):
        translation, rotation = self.compute_map_to_odom_transform(map_trans, map_rot)
        self.translations[self.map_frame] = translation
        self.rotations[self.map_frame] = rotation

    def broadcast_map_odom_transform(self):
        try:
            self.broadcaster.sendTransform(self.translations[self.map_frame], )

    @staticmethod
    def parse_tag_id(tag_x):
        return tag_x.split('_')[1]

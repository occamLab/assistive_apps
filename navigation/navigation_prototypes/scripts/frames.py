#!/usr/bin/env python

import rospy
from rospkg import RosPack
import tf
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from navigation_prototypes.srv import CheckMapFrame
from navigation_prototypes.srv import TagSeen

import pickle
from os import path
from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose)
from optimization import Optimization
from pose_graph import PoseGraph


class Frames:

    def __init__(self):
        rospy.init_node('frames')
        self.package = RosPack().get_path('navigation_prototypes')  # Directory for this ros package
        self.data_folder = path.join(self.package, 'data/raw_data')

        #### Transform Parameters ####
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.map_frame = "map"
        self.map_frame_published = False
        self.AR_broadcasted = False

        ### pose graph ###
        self.tag_in_frame = None  # tag seen during data collection
        self.tag_for_transform = None

        self.pose_graph = None
        self.translations = {}
        self.rotations = {}

        #### ros service ####
        rospy.Service('map_frame_published', CheckMapFrame, self.map_frame_published_service)

    def update_tag_client(self):
        # print "REQUESTING FIRST TAG"
        rospy.wait_for_service('tag_seen')
        try:
            get_tag = rospy.ServiceProxy('tag_seen', TagSeen)
            response = get_tag()
            self.tag_in_frame = int(response.tag)
            if self.pose_graph and self.tag_in_frame in self.pose_graph.tag_vertices.keys():
                self.tag_for_transform = self.tag_in_frame
            elif self.pose_graph and not self.map_frame_published:
                print "CURRENT TAG NOT RECORDED BEFORE. MAP FRAME NOT COMPUTED"
            # print "tag_in_frame frame:", self.tag_in_frame
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def update_map_frame_client(self):
        # print "REQUESTING MAP FRAME INFO"
        rospy.wait_for_service('check_map_frame')
        try:
            map_frame_exist = rospy.ServiceProxy('check_map_frame', CheckMapFrame)
            response = map_frame_exist()
            if response.exist:
                with open(path.join(self.data_folder, "data_collected.pkl"), 'rb') as f:  # read pose graph file
                    self.pose_graph = pickle.load(f)  # load pickle
                    print("Posegraph LOADED FOR COORDINATE FRAMES")
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def map_frame_published_service(self, req):
        return self.map_frame_published

    def update_odom_AR_transform(self):
        try:
            tag_frame = "tag_" + str(self.tag_in_frame)
            self.listener.waitForTransform("odom", tag_frame, rospy.Time(0), rospy.Duration(0.5))
            if self.listener.canTransform("odom", tag_frame, rospy.Time(0)):
                trans, rot = self.listener.lookupTransform("odom", tag_frame, rospy.Time(0))
                self.translations[self.tag_in_frame] = trans
                self.rotations[self.tag_in_frame] = rot

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf2_ros.TransformException) as e:
            if e is tf.ConnectivityException:
                print "ODOM_AR TRANSFORM EXCEPTION:", e

    def broadcast_odom_AR_transform(self,tag):
        """ Will broadcast the transform between parent node
        odom with AR as the direct child """

        try:
            self.broadcaster.sendTransform(self.translations[tag], self.rotations[tag],
                                           rospy.Time.now(), "AR_%d" % tag,  # child
                                           "odom")  # parent
            self.AR_broadcasted = True
            #print "AR odom broadcasted"
        except (KeyError) as e:
            print "BROADCAST ERROR: TAG TRANSFORMED NOT CACHED. PLEASE RESCAN."

    def compute_map_to_odom_transform_transformer(self, AR_in_map_trans, AR_in_map_rot, tag):
        transform = tf.Transformer(True, rospy.Duration(10))
        self.listener.waitForTransform("AR_%d" % tag, "odom", rospy.Time(0), rospy.Duration(0.5))
        odom_in_AR_trans, odom_in_AR_rot = self.listener.lookupTransform("AR_%d" % tag, "odom", rospy.Time(0))
        Optimization.write_transform_stamped_msg(transform, "odom", odom_in_AR_trans, odom_in_AR_rot, "AR")
        Optimization.write_transform_stamped_msg(transform, "AR", AR_in_map_trans, AR_in_map_rot, "map")
        odom_in_map_trans, odom_in_map_rot = transform.lookupTransform("map", "odom", rospy.Time(0))
        print "ODOM IN MAP TRANSFORM:", list(odom_in_map_trans) + list(odom_in_map_rot)
        # AR_in_odom_trans, AR_in_odom_rot = transform.lookupTransform("odom", "AR", rospy.Time(0))
        # print "DEBUG: AR IN odom TRANSFORM:", list(AR_in_odom_trans) + list(AR_in_odom_rot)
        # map_in_AR_trans, map_in_AR_rot = transform.lookupTransform("AR", "map", rospy.Time(0))
        # print "DEBUG: MAP IN AR TRANSFORM:", list(map_in_AR_trans) + list(map_in_AR_rot)
        return odom_in_map_trans, odom_in_map_rot

    def compute_map_to_odom_transform_math(self, map_trans, map_rot, tag):
        map_pose = PoseStamped(pose=convert_translation_rotation_to_pose(map_trans, map_rot),
                               header=Header(stamp=rospy.Time(0),
                                             frame_id="AR_%d" % tag))  # frame_id: frame the pose is in
        self.listener.waitForTransform("AR_%d" % tag, "odom", map_pose.header.stamp, rospy.Duration(0.5))
        odom_to_map = self.listener.transformPose("odom", map_pose)
        translation, rotation = convert_pose_inverse_transform(odom_to_map.pose)  # put odom in new odom
        # odom_trans, odom_rot = self.listener.lookupTransform("AR_%d" % tag, "odom", rospy.Time(0))
        # print "odom in AR transform:", odom_trans + odom_rot
        # ar_trans, ar_rot = self.listener.lookupTransform("odom", "AR_%d" % tag, rospy.Time(0)) # debug
        # print "AR in odom transform:", ar_trans + ar_rot # debug
        # odom_in_AR_pose = convert_translation_rotation_to_pose(odom_trans, odom_rot)
        # AR_in_odom_comp_trans, AR_in_odom_comp_rot = convert_pose_inverse_transform(odom_in_AR_pose)
        # print "AR in odom computed transform:", AR_in_odom_comp_trans, AR_in_odom_comp_rot
        # print "odom in map translation:", translation
        # print "odom in map rotation:", rotation
        return translation, rotation

    def update_map_odom_transform_pose_graph_math(self):
        AR_in_map_translation = self.pose_graph.tag_vertices[self.tag_for_transform].translation
        AR_in_map_rotation = self.pose_graph.tag_vertices[self.tag_for_transform].rotation
        AR_in_map_pose = convert_translation_rotation_to_pose(AR_in_map_translation, AR_in_map_rotation)
        map_translation, map_rotation = convert_pose_inverse_transform(AR_in_map_pose)
        translation, rotation = self.compute_map_to_odom_transform_math(map_translation, map_rotation,self.tag_for_transform)
        self.translations[self.map_frame] = translation
        self.rotations[self.map_frame] = rotation
        # print "AR_IN_MAP_POSE:", AR_in_map_pose
        # print "map translation in AR:", map_translation
        # print "map rotation in AR:", map_rotation

    def update_map_odom_transform_pose_graph_transformer(self):
        AR_in_map_translation = self.pose_graph.tag_vertices[self.tag_for_transform].translation
        AR_in_map_rotation = self.pose_graph.tag_vertices[self.tag_for_transform].rotation
        AR_in_map_pose = convert_translation_rotation_to_pose(AR_in_map_translation, AR_in_map_rotation)
        map_translation, map_rotation = convert_pose_inverse_transform(AR_in_map_pose)
        translation, rotation = self.compute_map_to_odom_transform_transformer(map_translation, map_rotation,self.tag_for_transform)
        self.translations[self.map_frame] = translation
        self.rotations[self.map_frame] = rotation

    def broadcast_map_odom_transform(self):
        try:
            self.broadcaster.sendTransform(self.translations[self.map_frame], self.rotations[self.map_frame],
                                           rospy.Time.now() + rospy.Duration(0.2), "odom",  # child
                                           self.map_frame)  # parent
            self.map_frame_published = True
        except(KeyError) as e:
            print "BROADCAST ERROR", e

    def odom_AR_transform(self):
        self.update_tag_client()
        if self.tag_in_frame != -1:
            self.update_odom_AR_transform()
            self.broadcast_odom_AR_transform(self.tag_in_frame)
        if self.tag_for_transform is not None:
            self.broadcast_odom_AR_transform(self.tag_for_transform)

    def map_odom_transform(self):
        if self.pose_graph and self.tag_for_transform is not None and self.AR_broadcasted:
            self.update_map_odom_transform_pose_graph_transformer()
            # self.update_map_odom_transform_pose_graph_math()
            self.broadcast_map_odom_transform()
        elif not self.pose_graph:
            self.update_map_frame_client()

    def run(self):
        """ The main run loop """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.odom_AR_transform()
            self.map_odom_transform()

            r.sleep()


if __name__ == "__main__":
    AR_Frame = Frames()
    AR_Frame.run()

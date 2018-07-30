#!/usr/bin/env python

import rospy
from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from apriltags_ros.msg import AprilTagDetectionArray
from navigation_prototypes.srv import CheckMapFrame
from navigation_prototypes.srv import TagSeen
from navigation_prototypes.srv import phone
import tf
from os import path

import pickle
from pose_graph import PoseGraph, Vertex, Edge
import pyttsx


class WaypointDetectionNavigation(object):
    def __int__(self):
        self.posegraph = None
        self.origin_frame_presence = False
        self.tag_in_frame = None

        #### ROS SUBSCRIBERS ####
        self.get_phone_type_client()
        self.determine_subscribed_topics_from_phone_type()

        #### ROS Service ####
        rospy.Service('tag_seen', TagSeen, self.tag_seen_service)
        rospy.Service('check_map_frame', CheckMapFrame, self.check_map_frame_service)

    def determine_subscribed_topics_from_phone_type(self):
        print "PHONE: ", self.phone
        if self.phone == "iPhone":
            ''' For use with the iPhone: '''
            rospy.Subscriber('/ios_pose', PoseStamped, self.process_pose)
            rospy.Subscriber('/april_tags_ios',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                             AprilTagDetectionArray,
                             self.tag_callback)
        elif self.phone == "tango":
            ''' For use with the tango: '''
            rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)  # Subscriber for the tango pose.
            rospy.Subscriber('/fisheye_undistorted/tag_detections',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                             AprilTagDetectionArray,
                             self.tag_callback)

    def get_phone_type_client(self):
        print "CALLING THE SERVICE"
        rospy.wait_for_service('/phone_type')
        try:
            get_phone_type = rospy.ServiceProxy('/phone_type', phone)
            self.phone = get_phone_type().phoneType
        except rospy.ServiceException, e:
            print "Service call failed: %s"

    def tag_seen_service(self, req):
        """
        Ros service responsible for offering the id of current april tags detected.
        :param req: None
        :return: if of current april tags detected
        """
        if self.tag_in_frame is None:
            service_resp = -1
        else:
            service_resp = self.tag_in_frame
            # print "RETURNING TAG SERVICE:", service_resp
        return service_resp

    def check_map_frame_service(self, req):
        return True

    def map_frame_published_client(self):
        # print "REQUESTING FIRST TAG"
        rospy.wait_for_service('map_frame_published')
        try:
            map_frame_published = rospy.ServiceProxy('map_frame_published', CheckMapFrame)
            response = map_frame_published()
            if response.exist:
                self.origin_frame_presence = True
                print "MAP FRAME BROADCASTED, START DATA COLLECTION"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def tag_callback(self, msg):
        """
        Process tag
        """
        # Processes the april tags currently in Tango's view.
        self.tags_detected = None  # Initialize tags_detected
        if msg.detections:  # if a tag is detected
            self.tags_detected = msg.detections  # save the detected tags
            # get first tag (assume only 1 tag for the most part, if not then this code will just choose one.)
            curr_tag = msg.detections[0]
            self.tag_in_frame = curr_tag.id

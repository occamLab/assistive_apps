#!/usr/bin/env python

import rospy
from rospkg import RosPack
from keyboard.msg import Key
from geometry_msgs.msg import PoseStamped
from apriltags_ros.msg import AprilTagDetectionArray
from navigation_prototypes.srv import CheckMapFrame
from navigation_prototypes.srv import TagSeen
from navigation_prototypes.srv import phone
from helper_functions import convert_translation_rotation_to_pose
import tf
from os import path
from math import sqrt

import pickle
from pose_graph import PoseGraph, Vertex, Edge
import pyttsx


class NavWaypoints(object):
    def __init__(self, filename):
        rospy.init_node('nav_waypoints')
        self.engine = pyttsx.init()  # Speech engine
        self.has_spoken = False  # Boolean for if the speech engine has spoken
        self.listener = tf.TransformListener()

        #### POSE GRAPH PARAMETERS ####
        self.posegraph = None
        self.package = RosPack().get_path('navigation_prototypes')
        self.optimized_data_folder = path.join(self.package, 'data/optimized_data')
        self.filename = filename

        #### NAVIGATION PARAMETERS ####
        self.origin_frame_presence = False
        self.tag_in_frame = None
        self.curr_pose = None
        self.proximity_to_destination = 2
        self.terminate_session = False

        #### ROS SUBSCRIBERS ####
        self.get_phone_type_client()
        self.determine_subscribed_topics_from_phone_type()
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)  # Subscriber for the keyboard information.

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
        :return: id of current april tags detected
        """
        if self.tag_in_frame is None:
            service_resp = -1
        else:
            service_resp = self.tag_in_frame
            # print "RETURNING TAG SERVICE:", service_resp
        return service_resp

    def check_map_frame_service(self, req):
        """
        The origin frame this module run on is always the map frame
        :param req: None
        :return: Boolean to indicate whether the origin frame is the map frame or not
        """
        return True

    def map_frame_published_client(self):
        # print "REQUESTING FIRST TAG"
        rospy.wait_for_service('map_frame_published')
        try:
            map_frame_published = rospy.ServiceProxy('map_frame_published', CheckMapFrame)
            response = map_frame_published()
            if response.exist:
                self.origin_frame_presence = True
                print "MAP FRAME BROADCASTED, START NAVIGATION"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def process_pose(self, msg):
        if self.origin_frame_presence:
            self.listener.waitForTransform("map", "odom", msg.header.stamp, rospy.Duration(0.5))
            self.curr_pose = self.listener.transformPose("map", msg)

    @staticmethod
    def distance_formula(a, b):
        return sqrt((a.position.x - b.position.x) ** 2 + (a.position.y - b.position.y) ** 2 + (
                a.position.z - b.position.z) ** 2)

    def query_nearby_waypoints(self, curr_pose):
        for waypoint in self.posegraph.waypoints_vertices.keys():
            waypoint_pose = convert_translation_rotation_to_pose(
                self.posegraph.waypoints_vertices[waypoint].translation,
                self.posegraph.waypoints_vertices[waypoint].rotation)
            distance = NavWaypoints.distance_formula(curr_pose, waypoint_pose)
            # if the distance is less than the previously set waypoint radius
            if distance < self.proximity_to_destination:
                mesg = "Found %s" % waypoint + "distance to point: %f" % distance
                print mesg  # print the waypoint was found
                self.engine.say(mesg)  # have engine read out waypoint

    def tag_callback(self, msg):
        """
        Process tag
        """
        if msg.detections:  # if a tag is detected
            # get first tag (assume only 1 tag for the most part, if not then this code will just choose one.)
            curr_tag = msg.detections[0]
            self.tag_in_frame = curr_tag.id

    def key_pressed(self, msg):
        """
        This is the callback function for the ROS keyboard.  The msg.code here returns the ord's of what's pressed.
        """

        if msg.code == ord('e'):  # when user presses t
            """
            Terminate Navigation Session
            """
            self.terminate_session = True  # Navigation Session Terminate

    def load_pose_graph(self):
        with open(path.join(self.optimized_data_folder, self.filename), 'rb') as data:
            self.posegraph = pickle.load(data)  # pose graph that will be optimized
            print "POSE GRAPH LOADED. Ready to go."

    def start_speech_engine(self):
        """
        Initialize the speech engine.
        """
        if not self.has_spoken:
            self.engine.say("Starting up.")
            self.engine.runAndWait()
            self.engine.say("Welcome to Waypoints Detection Navigation")
            self.engine.runAndWait()
            self.has_spoken = True

    def run(self):
        r = rospy.Rate(10)  # Attempts to run loop at 10 times per second
        print "Starting Up."
        self.start_speech_engine()
        self.load_pose_graph()
        while not rospy.is_shutdown():
            if self.posegraph and self.origin_frame_presence:  # if calibration_AR and recording
                self.query_nearby_waypoints(self.curr_pose)

            if self.terminate_session:
                break
            else:
                r.sleep()

        print "Waypoint Navigation Finished."


if __name__ == "__main__":
    navigation = NavWaypoints("data_optimized.pkl")
    navigation.run()

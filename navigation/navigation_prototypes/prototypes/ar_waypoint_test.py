#!/usr/bin/env python

from os import system, path
import math
import time

try:
    import cPickle as pickle
except:
    import pickle

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf
from std_msgs.msg import Header, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
import pyttsx
from rospkg import RosPack
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header, ColorRGBA
from keyboard.msg import Key
from random import random
from os import system, path
from copy import deepcopy
import numpy as np
from scipy import linalg, matrix
import scipy


def null(a, rtol=1e-5):
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol * s[0]).sum()
    return rank, v[rank:].T.copy()


def compute_basis_vector(rotation):
    # for step in np.arange(0, 3*np.pi, .05):
    q2 = quaternion_from_euler(0, 0,
                               .05)  # for generating a rotation matrix to rotate a small amount around the z axis
    qsecondrotation = quaternion_multiply(q2, rotation)
    change = (qsecondrotation[0:3] - rotation[0:3])
    change = change / np.linalg.norm(
        change)  # Determine which direction is the yaw direction and then make sure that direction is diminished in the information matrix
    v = change / np.linalg.norm(change)
    _, u = null(v[np.newaxis])
    basis = np.hstack((v[np.newaxis].T, u))
    # place high information content on pitch and roll and low on changes in yaw
    I = basis.dot(np.diag([.001, 1000, 1000])).dot(basis.T)
    return I


class ArWaypointTest(object):
    def __init__(self):
        #### ROS Variables ####
        self.engine = pyttsx.init()  # Speech engine
        top = RosPack().get_path('mobility_games')  # Directory for this ros package
        self.sound_folder = path.join(top, 'auditory/sound_files')  # Location of audio files

        rospy.init_node('ar_waypoint_test')  # Initialization of another node.
        self.listener = tf.TransformListener()  # The transform listener
        self.broadcaster = tf.TransformBroadcaster()  # The transform broadcaster
        self.waypoint_viz_pub = rospy.Publisher('/waypoint_visualizer',
                                                # Create a ros puplisher for the marker visualization
                                                MarkerArray,
                                                queue_size=10)
        """self.place_pub = rospy.Publisher('/locations',
                                         String,
                                         queue_size=10)"""
        # srv = Server(SemanticWaypointsConfig, self.config_callback)

        #### Editing Variables ####
        self.proximity_to_destination = 1.8  # Size in meters of the radius of the waypoints.
        self.search_dist = 12  # Distance that the waypoint searches for when user attempts to search.
        self.record_interval_normal = rospy.Duration(.1)  # Interval of time between recording normally
        self.record_interval_tag_seen = rospy.Duration(.1)  # Interval of time between recording when tag is seen
        self.g2o_data_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data.g2o'  # path to the data compiled into the g2o file
        self.g2o_data_copy_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data_cp.g2o'  # copy of the unedit data
        self.g2o_result_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/result.g2o'  # result path of g2o

        #### Tracker Variables ####
        self.calibration_mode = True  # Flag to keep track of game state -- Calibration/Run Modes
        self.calibration_AR = False  # Flag to keep track of game state -- AR Calibration/Other Modes. (this supercedes other modes in priority if turned on)
        self.waypoints = {}  # Dictionary of waypoints
        self.waypoints_detected = {}  # Dictionary of currently detected waypoints??
        self.origin_tag = None  # tag_id of main tag.
        self.supplement_tags = {}  # Dictionary of tag_id's connected to their poses.
        self.tag_seen = False  # boolean describing if the first tag has been seen or not.
        self.AR_Find_Try = False  # Boolean for whether person is finding an AR tag or not.
        self.x = None  # x position of Tango. Start at None because no data have been received yet.
        self.y = None  # y position of Tango
        self.z = None  # z position of Tango
        self.has_spoken = False  # Boolean for if the speech engine has spoken
        self.markerlist = []  # Empty list, to contain waypoints.
        self.waypoint_id = 0  # Waypoint_id's
        self.last_record_time = 0  # Last time of a pose record
        self.g2o_result = None  # Result of g2o is the actual information from the file.e
        self.vertex_id = 586  # Start id is the id after all the tag ids
        self.recording = False  # Boolean for recording
        self.tags_detected = None  # list of detected tags.
        self.nowtime = None  # Timing for the rose pose and
        self.offset = 10 ** -3  # offset for importance matrix eigenvalues to importance matrix positive semidefinte
        self.error_count = 0  # count of pose failure
        # self.offset = 0
        ## Testing Materials
        self.origin_msg = None  # Main Tag Msg with all the pose and id info.
        self.lastpose = None  # Last pose of the phone
        self.distance_traveled = [0, 0, 0, 0]  # Counter for total traveled distance
        self.testing_tag_id = 2  # Tag Id for Testing (debugging)
        self.record_interval = self.record_interval_normal  # Interval of time between pose recording for SLAM algorithm
        self.g2o_data = None  # Have variable to be prepared for file reading and writing
        open(self.g2o_data_path, 'wb+').close()  # Overwrite current g2o data file
        self.tagtimes = {}  # Dictionary of Last recorded tag times
        self.tagrecorded = {}
        for i in range(587):
            self.tagtimes[i] = rospy.Time(1)
            self.tagrecorded[i] = False
        self.pose_failure = False  # Flag for whether record time has failed to update (most likely because of pose not updating)
        self.testfile = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/naive.txt'
        self.test_data = open(self.testfile, 'wb+')

        #### ROS SUBSCRIBERS ####
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)  # Subscriber for the tango pose.
        rospy.Subscriber('/fisheye_undistorted/tag_detections',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                         AprilTagDetectionArray,
                         self.tag_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)  # Subscriber for the keyboard information.

    def process_pose(self, msg):
        """
        This function processes the pose, and saves it in the AR frame which can be useful for various features.
        """
        if self.origin_tag >= 0:
            if (self.lastpose):  # if we didn't just start up and have seen the origin tag
                movement = [msg.pose.position.x - self.lastpose.pose.position.x,
                            # calculate the difference in the poses.
                            msg.pose.position.y - self.lastpose.pose.position.y,
                            msg.pose.position.z - self.lastpose.pose.position.z]
                for i in range(3):  # get the total distance that was traveled over that period
                    self.distance_traveled[i] += abs(movement[i])  # record it into the distance traveled.
            try:
                msg.header.stamp = rospy.Time(0)  # set the header stamp to now.
                newitem = self.listener.transformPose('AR', msg)  # transform the pose into the AR frame
                self.x = newitem.pose.position.x
                self.y = newitem.pose.position.y
                self.z = newitem.pose.position.z  # record the information from that transformed phone pose.

                if not self.calibration_mode and not self.AR_Find_Try:  # if not calibrating. (aka if running)
                    for waypoint in self.waypoints:  # iterate through the waypoints
                        # get the distance to each one
                        disttopoint = math.sqrt(
                            (self.waypoints[waypoint][0] - self.x) ** 2 + (self.waypoints[waypoint][1] - self.y) ** 2
                            + (self.waypoints[waypoint][2] - self.z) ** 2)
                        # if the distance is less than the previously set waypoint radius
                        if disttopoint < self.proximity_to_destination:
                            if not self.waypoints_detected[waypoint]:  # and the waypoint is not detected
                                mesg = "Found %s" % waypoint
                                print mesg  # print the waypoint was found
                                print "distance to point: " + str(disttopoint)  # print the distance to the waypoint
                                self.engine.say(mesg)  # have engine read out waypoint
                                self.waypoints_detected[waypoint] = True  # make this waypoint detected.
                                break
                            else:
                                # even if the tag has already been found, print out the distanc to the point
                                print "distance to point: " + str(disttopoint)
                        else:  # if not inside waypoint
                            self.waypoints_detected[waypoint] = False  # set its detcted status to false
            except Exception as inst:  # Exception
                print "Exception is", inst  # print excetion
        self.lastpose = msg  # Lastpose is set to the this pose msg

    def tag_callback(self, msg):
        """
        Callback Function for AR tags
        """
        # Processes the april tags currently in Tango's view.
        self.tags_detected = None  # Initialize tags_detected
        self.record_interval = self.record_interval_normal  # Initialize the record_interval to normal intervals
        if msg.detections:  # if a tag is detected
            self.record_interval = self.record_interval_tag_seen  # record at the tag_seen record_interval
            self.tags_detected = msg.detections  # save the detected tags
            curr_tag = msg.detections[
                0]  # get first tag (assume only 1 tag for the most part, if not then this code will just choose one.)
            curr_tag_pose = curr_tag.pose  # save the tag pose.
            headdiff = curr_tag_pose.header.stamp - rospy.Time.now()  # difference between last time tag seen and current time
            if headdiff > rospy.Duration(1):
                print("THIS IS BAD.")
            # print "header difference", headdiff
            # curr_tag_pose.header.stamp = rospy.Time(0) # set the tag_pose stamp to now.
            self.listener.waitForTransform(curr_tag_pose.header.frame_id, "odom", curr_tag_pose.header.stamp,
                                           rospy.Duration(.5))
            if self.listener.canTransform(curr_tag_pose.header.frame_id, "odom", curr_tag_pose.header.stamp):
                curr_tag_transformed_pose = self.listener.transformPose('odom',
                                                                        curr_tag_pose)  # Transform the pose from the camera frame to the odom frame.
                tag_id = curr_tag.id  # save tag_id
                # print "tag_id", tag_id
                if not tag_id in self.tagtimes.keys():
                    self.tagtimes[tag_id] = rospy.Time(1)

                # if in Ar calibrate mode and user presses the update button:
                if (self.calibration_AR and self.AR_Find_Try):
                    tagfound = False
                    print "AR_CALIBRATION: executing a find try", tag_id
                    # Only prompt user to input tag name once
                    if not self.tag_seen:  # if we haven't seen a tag
                        print "AR_CALIBRATION: Origin Tag Found: " + str(tag_id)  #
                        tagfound = self.RecordTag(curr_tag,
                                                  True)  # Record Tag Vertex (the True boolean is to denote that this is the origin tag and should create a FIX line in g2o)
                        if tagfound:
                            self.origin_msg = curr_tag_transformed_pose  # make this tag the origin tag
                            self.origin_tag = tag_id
                            self.tag_seen = True  # set that you have seen the first tag
                            self.distance_traveled = [0, 0, 0, 0]  # set distance to 0
                        # newfound = True #unnecessary here.
                    if not (tag_id == self.origin_tag):  # if the tag_id isn't that of the origin tag.
                        if not (
                                tag_id in self.supplement_tags.keys()):  # if the tag is not in the supplementary tag list
                            tagfound = self.RecordTag(curr_tag, False)  # Record Tag Vertex (for g2o)
                            if tagfound:
                                print "AR_CALIBRATION: Supplementary Tag Found: " + str(tag_id)
                                self.supplement_tags[tag_id] = curr_tag_transformed_pose  # set new supplemental AR Tag
                                print(self.supplement_tags.keys())
                        else:  # If this is a previously found tag,
                            tagfound = True
                            print "AR_CALIBRATION: Found Old Tag: " + str(tag_id)

                    if tag_id == self.origin_tag and self.tag_seen:  # If the origin tag (first tag) is seen again.
                        tagfound = True
                        print "AR_CALIBRATION: Origin Tag Refound!"
                        self.origin_msg = curr_tag_transformed_pose  # Reset the origin tag

                    if not tagfound:
                        print("No tags found.")

                if self.tag_seen and tag_id == self.testing_tag_id:  # If we have seen the origin and we are now looking at the testing tag:
                    try:
                        print("tried to record test tag")
                        self.listener.waitForTransform("AR", "tag_" + str(tag_id), curr_tag.pose.header.stamp,
                                                       rospy.Duration(.5))  # Wait for tag transform
                        (trans, rot) = self.listener.lookupTransform("AR", "tag_" + str(tag_id),
                                                                     curr_tag.pose.header.stamp)  # Lookup tag transform
                        self.test_data.write("TAG %f %f %f %f %f %f %f\n" % tuple(trans + rot))  # Write into a G2O file
                    except (tf.ExtrapolationException,
                            tf.LookupException,
                            tf.ConnectivityException,
                            tf.Exception,
                            ValueError) as e:
                        print "TestTag Exception: " + str(e)
                """self.distance_traveled[3] = math.sqrt(math.pow(self.distance_traveled[0], 2) #calculate how far we have traveled since we saw the origin.
                                                      + math.pow(self.distance_traveled[1], 2)
                                                      + math.pow(self.distance_traveled[2], 2))
                pose_now = curr_tag_transformed_pose.pose.position #Get the pose of the testing AR tag
                pose_old = self.origin_msg.pose.position #Get pose of supposed origin tag location
                drift_dist = [pose_now.x-pose_old.x, pose_now.y-pose_old.y, pose_now.z-pose_old.z, 0] #calculate the drift [x, y, z, 0]

                ""[curr_tag_pose.pose.position.x - self.origin_msg.pose.pose.position.x,
                curr_tag_pose.pose.position.y - self.origin_msg.pose.pose.position.y,
                curr_tag.pose.pose.position.z - self.origin_msg.pose.pose.position.z,0]""

                drift_dist[3] = math.sqrt(math.pow(drift_dist[0], 2) + math.pow(drift_dist[1], 2) + math.pow(drift_dist[2], 2)) #make the 4th entry of the drift the total drift (magnitude)
                print("Distance Traveled: " + str(self.distance_traveled))
                print("Distance Drifted: " + str(drift_dist));"""

            else:
                print "TRANSFORM FAILURE: from tag to odom in tag callback"
        self.AR_Find_Try = False  # Finish trying to find a tag.

    def start_record(self):
        if self.calibration_AR and self.recording == False:  # If we aren't recording and AR Calibration is on.
            i = 0
            while (i < 10):
                try:
                    self.nowtime = rospy.Time.now()  # Set Nowtime to Now
                    self.listener.waitForTransform("odom", "real_device", self.nowtime,
                                                   rospy.Duration(.5))  # Wait for...
                    if self.listener.canTransform("odom", "real_device", self.nowtime):
                        (trans, rot) = self.listener.lookupTransform("odom", "real_device",
                                                                     self.nowtime)  # and lookup the transform for phone's current position.
                        self.g2o_data.write("VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n" % tuple(
                            [self.vertex_id + 1] + trans + rot))  # Write the vertex into g2o file.

                        ## Writing Orientation Dummy ## to dampen certain parameters of the movement
                        I = compute_basis_vector(rot)
                        indeces = np.triu_indices(
                            3)  # return the indices of an upper triangle of a 3x3 array
                        self.g2o_data.write(
                            "VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n" % tuple([self.vertex_id + 2] + [0, 0, 0] + rot))
                        # write edge for moving from the last pose to the current pose
                        self.g2o_data.write("EDGE_SE3:QUAT %i %i %f %f %f %f %f %f %f " % (
                                (self.vertex_id + 1, self.vertex_id + 2) + (0, 0, 0) + (0, 0, 0, 1)))
                        self.g2o_data.write("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 %f %f %f %f %f %f\n" % (
                            tuple(I[indeces])))  # no translation importance
                        self.g2o_data.write("FIX %i\n" % (self.vertex_id + 2))

                        print('recording started')
                        self.last_record_time = self.nowtime  # set last record time to nowtime as well.
                        self.vertex_id += 2  # add to the vertex id
                        self.recording = True  # set recording true
                        self.tags_detected = None  # set tags detected to none so as to not see tags during the first record.
                        i = 10
                except (tf.ExtrapolationException,
                        tf.LookupException,
                        tf.ConnectivityException,
                        tf.Exception,
                        ValueError) as e:
                    print "recordStart Exception: " + str(e)
                    i += 1

    def key_pressed(self, msg):
        """
        This is the callback function for the ROS keyboard.  The msg.code here returns the ord's of what's pressed.
        """
        # Switch to run mode
        if msg.code == ord('r'):  # when user presses R
            """
            Detect AR tag
            """
            print self.AR_Find_Try
            self.AR_Find_Try = True  # Try to find and record an AR tag next time possible.

        if msg.code == ord('y'):  # When user presses Y
            """
            Toggle AR_Calibration (and start and stop recording to file.)
            """
            self.calibration_AR = not self.calibration_AR  # Toggle calibration_AR
            if self.calibration_AR:  # If AR Calibration just turned on,
                print('switched to AR Calibration Mode')  # print message
                self.engine.say('switched to AR Calibration Mode')
                self.g2o_data = open(self.g2o_data_path, 'ab')
                if path.getsize(self.g2o_data_path) > 0:
                    self.g2o_data.write('\n')
                self.start_record()

            else:  # if AR Calibration just turned off.
                try:
                    self.g2o_data.close()  # Try closing the g2o file
                    self.recording = False  # Turn of the recording
                    print('recording stopped')
                except:
                    print("file close exception")
                if self.calibration_mode:  # If Calibration is on.
                    print('switched to Calibration Mode')
                else:  # If Calibration is off
                    print('switched to Run Mode')

        if msg.code == ord(']'):  # when user presses ]
            """
            Record and Execute G2O
            """
            if self.recording:
                print("Please finish AR_calibration before executing g2o")
            else:
                system("g2o -o %s %s" % (self.g2o_result_path, self.g2o_data_path))  # Run G2o
                system("cp %s %s" % (self.g2o_data_path, self.g2o_data_copy_path))  # Copy Original Data

        """if msg.code == ord('p'):
            ""
            Place Publisher
            ""
            if self.recording:
                print("Where are you?")
                s = raw_input("")
                self.place_pub.publish(s)
            else:
                print("You must be recording to publish your place.")"""
        if msg.code == ord('-'):
            """
            Start Recording for G2O
            """
            if self.recording:
                self.nowtime = rospy.Time.now()

        if msg.code == ord(' '):
            """
            Read out the nearby waypoints.
            """
            nearways = []  # set list of nearby waypoints to an empty list
            for waypoint in self.waypoints:  # for each waypoint
                disttopoint = math.sqrt(
                    (self.waypoints[waypoint][0] - self.x) ** 2 + (self.waypoints[waypoint][1] - self.y) ** 2 + 4 * (
                            self.waypoints[waypoint][2] - self.z) ** 2)  # determine distance
                if (disttopoint < self.search_dist):  # if distance is less than search distance
                    nearways.append(waypoint)  # add it to the list
            if len(nearways) > 0:  # if there are nearby waypoints
                self.engine.say('Here are some nearby waypoints ')  # say here are some nearby waypoints
                for way in nearways:
                    self.engine.say(way)  # say each waypoint
            else:
                self.engine.say('No nearby waypoints')  # if no nearby points say nothing.

        if msg.code == ord('.'):
            """
            Deleting Waypoints
            """
            if self.calibration_mode and not self.calibration_AR:  # only allow deleting if in calibration mode
                while True:
                    tmpwaydict = {}  # create dictionary of waypoints
                    ways = 0  # create counter
                    for waypoint in self.waypoints:  # for each waypoint
                        # if self.waypoints_detected[waypoint]:
                        ways += 1  # add to ways
                        tmpwaydict[ways] = waypoint  # put waypoint in dictionary at key ways
                    print(
                        "Enter the number of the waypoint to delete (enter 0 to not delete):")  # ask person to enter a number in the dictionary
                    waylist = tmpwaydict.items()  # get items in the dictionary
                    for i in range(len(waylist)):  # Print all of the keys and waypoint names
                        entry = waylist[i]
                        print(str(entry[0]) + ': ' + str(entry[1]))
                    s = raw_input("")  # get input from user
                    try:  # try to set the input to an integer
                        s = int(s)
                    except (ValueError, TypeError) as inst:
                        print ("That's not a number")
                    deletedpoint = tmpwaydict.get(s, -1)  # get deleted point
                    if (s == 0):
                        print("Deletion Menu Closed.")
                        break  # if user puts 0 then break
                    elif (deletedpoint == -1):
                        print(str(s) + ' is an invalid number.')
                    else:
                        self.waypoints.pop(deletedpoint, None)
                        self.waypoints_detected.pop(deletedpoint, None)
                        print('deleted waypoint: ' + str(deletedpoint))

        if msg.code == ord('a'):
            """
            Toggle Calibration Mode
            """
            self.calibration_mode = not self.calibration_mode  # toggle calibration mode
            if self.calibration_mode:  # print which mode one is now in.
                print('switched to Calibration Mode')
            else:
                print('switched to Run Mode')

        if msg.code == ord('b'):
            """
            Place Waypoint
            """
            if self.calibration_mode and not self.calibration_AR:
                waypoint_location = [self.x, self.y, self.z]  # Get current position
                waypoint_name = raw_input("Name the Waypoint: ")  # name the waypoint.
                scale = 2 * self.proximity_to_destination  # Scale
                newpoint = Marker(header=Header(frame_id="AR", stamp=rospy.Time(0)),  # Create marker for the point.
                                  type=Marker.SPHERE,
                                  pose=Pose(position=Point(x=waypoint_location[0], y=waypoint_location[1],
                                                           z=waypoint_location[2])),
                                  scale=Vector3(x=scale, y=scale, z=scale),
                                  color=ColorRGBA(r=random(), g=random(), b=random(), a=1.0),
                                  id=self.waypoint_id,
                                  ns=waypoint_name)
                self.markerlist.append(newpoint)  # append the marker to the markerlist
                self.waypoints[waypoint_name] = waypoint_location  # add the new waypoint to the waypoints dictionary
                self.waypoints_detected[waypoint_name] = False  # add a false to the waypoint detected dictionary
                self.waypoint_id += 1  # add to the waypoint id (this is just for setting markers as different ids.)
                print self.waypoints

        if msg.code == ord('s'):
            """
            Save Everything.
            """
            print("WAYPOINTS SAVED")
            # TODO(rlouie):
            with open(path.expanduser(
                    '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/saved_calibration.pkl',
                      'wb') as f:  # write to waypoint file
                pickle.dump(self.waypoints, f)  # DUMP WAYPOINTS INTO THE PICKLE LOCATION

        if msg.code == ord('l'):
            """
            Load Everything.
            """
            print("WAYPOINTS LOADED")
            with open(path.expanduser(
                    '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/saved_calibration.pkl',
                      'rb') as f:  # read waypoint file
                self.waypoints = pickle.load(f)  # load pickle
                self.markerlist = []  # start a markerlist
                self.waypoint_id = 0  # set waypoint id to 0
                scale = 2 * self.proximity_to_destination  # set scale
                for item in self.waypoints.items():  # for each waypoint
                    """olditem1_1 = item[1][1]
                    item[1][1] = item[1][2]
                    item[1][2] = olditem1_1"""
                    self.markerlist.append(Marker(header=Header(frame_id="AR", stamp=rospy.Time(0)),
                                                  # add markers for waypoints to the markerlist
                                                  type=Marker.SPHERE,
                                                  pose=Pose(position=Point(x=item[1][0], y=item[1][1], z=item[1][2])),
                                                  scale=Vector3(x=scale, y=scale, z=scale),
                                                  color=ColorRGBA(r=random(), g=random(), b=random(), a=1.0),
                                                  id=self.waypoint_id,
                                                  ns=item[0]))
                    self.waypoint_id += 1  # add to waypoint id
                print(self.waypoints)

    def start_speech_engine(self):
        """
        Initialize the speech engine.
        This janky speech engine requires this janky startup.
        """
        if not self.has_spoken:  # this doesn't make sense, but it works.
            self.engine.say("Starting up.")
            a = self.engine.runAndWait()
            self.engine.say("Hello.")
            a = self.engine.runAndWait()
            self.has_spoken = True

    def RecordTag(self, tag, is_origin_tag):
        """
        This function records tags to the g2o file.
        """
        if self.tagrecorded[tag.id]:
            print("tag_%i already found." % tag.id)
        else:
            print("Attempted to find tag_%i" % tag.id)
            try:
                self.listener.waitForTransform("odom", "tag_" + str(tag.id), tag.pose.header.stamp,
                                               rospy.Duration(.5))  # Wait for tag transform
                if self.listener.canTransform("odom", "tag_" + str(tag.id), tag.pose.header.stamp):
                    (trans, rot) = self.listener.lookupTransform("odom", "tag_" + str(tag.id),
                                                                 tag.pose.header.stamp)  # Lookup tag transform
                    print trans, rot
                    self.g2o_data.write("VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n" % tuple(
                        [tag.id] + trans + rot))  # Write into a G2O file
                    if is_origin_tag:  # if the tag is the origin tag
                        self.g2o_data.write("FIX %i\n" % tag.id)  # write it in as the fix point in g2o
                    print("recorded tag: " + str(tag.id))
                    self.tagrecorded[tag.id] = True
                    return True
                else:
                    return False
            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.Exception,
                    ValueError) as e:
                print "recordTag Exception: " + str(e)
                return False

    def RecordTime(self, ofst):
        try:
            # print "AR_CALIBRATION: recordtime"
            self.nowtime = rospy.Time.now()  # set nowtime to now
            self.listener.waitForTransform("odom", "real_device", self.nowtime, rospy.Duration(.5))
            if self.listener.canTransform("odom", "real_device", self.nowtime):
                (trans, rot) = self.listener.lookupTransform("odom", "real_device", self.nowtime)  # lookup current pose
                self.listener.waitForTransformFull("real_device", self.last_record_time, "real_device", self.nowtime,
                                                   "odom", rospy.Duration(.5))
                (trans2, rot2) = self.listener.lookupTransformFull("real_device", self.last_record_time, "real_device",
                                                                   self.nowtime,
                                                                   "odom")  # lookup relative pose from last recorded tango pose to this tango pose
                tagtrans = {}  # dictionary of tag translations
                tagrot = {}  # dictionary of tag rotations
                if self.tags_detected and self.tag_seen:  # if there were tags detected
                    # print('1')
                    # print "AR_CALIBRATION: tags_detected are:", self.tags_detected
                    for tag in self.tags_detected:  # for each tag
                        tag_stamp = tag.pose.header.stamp
                        # time between now and last time the tag being recorded
                        if (tag_stamp - self.tagtimes[tag.id]) > rospy.Duration(.025):
                            if tag.id == self.origin_tag:  # if it is the origin tag
                                self.listener.waitForTransformFull("real_device", self.nowtime,
                                                                   "tag_" + str(tag.id), tag_stamp, "odom",
                                                                   rospy.Duration(.5))
                                if self.listener.canTransform("AR", "odom", self.last_record_time):
                                    (trans3, rot3) = self.listener.lookupTransformFull("real_device", self.nowtime,
                                                                                       "tag_" + str(tag.id), tag_stamp,
                                                                                       "odom")  # lookup transform from phone to tag
                                    tagtrans[tag.id] = trans3  # record findings
                                    tagrot[tag.id] = rot3
                                    self.tagtimes[tag.id] = tag.pose.header.stamp
                                    # print(self.tagtimes)
                                else:
                                    print "TRANSFORM FAILURE: from AR to odom and from real_device to tag"
                            else:
                                self.listener.waitForTransformFull("real_device", self.nowtime,
                                                                   "tag_" + str(tag.id), tag_stamp, "odom",
                                                                   rospy.Duration(.5))
                                if self.listener.canTransform("AR_" + str(tag.id), "odom", self.last_record_time):
                                    (trans3, rot3) = self.listener.lookupTransformFull("real_device", self.nowtime,
                                                                                       "tag_" + str(tag.id), tag_stamp,
                                                                                       "odom")  # lookup transform from phone to tag
                                    tagtrans[tag.id] = trans3  # record findings
                                    tagrot[tag.id] = rot3
                                    self.tagtimes[tag.id] = tag.pose.header.stamp
                                    # print(self.tagtimes)
                                else:
                                    print "TRANSFORM FAILURE: from AR to odom and from real_device to tag"
                I = compute_basis_vector(rot)
                indeces = np.triu_indices(3)  # return the indices of an upper triangle of a 3x3 array

                ## Writing Pose Information ##
                self.g2o_data.write("VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n" % tuple(
                    [self.vertex_id + 1] + trans + rot))  # write vertex for the new pose
                self.g2o_data.write("EDGE_SE3:QUAT %i %i %f %f %f %f %f %f %f " % tuple([self.vertex_id - 1,
                                                                                         self.vertex_id + 1] + trans2 + rot2))  # write edge for moving from the last pose to the current pose
                if (self.pose_failure):
                    self.g2o_data.write("%f 0 0 0 0 0 %f 0 0 0 0 %f 0 0 0 %f 0 0 %f 0 %f\n" % (
                        0, 0, 0, 0, 0, 0))  # write edge importance matrix
                else:
                    self.g2o_data.write("%f 0 0 0 0 0 %f 0 0 0 0 %f 0 0 0 %f 0 0 %f 0 %f\n" % (
                        1, 1, 1, 1, 1, 1))  # write edge importance matrix

                ## Writing Orientation Dummy ## to dampen certain parameters of the movement
                self.g2o_data.write(
                    "VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n" % tuple([self.vertex_id + 2] + [0, 0, 0] + rot))
                self.g2o_data.write("EDGE_SE3:QUAT %i %i %f %f %f %f %f %f %f " % (
                        (self.vertex_id + 1, self.vertex_id + 2) + (0, 0, 0) + (
                    0, 0, 0, 1)))  # write edge for moving from the last pose to the current pose

                importance = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 %f %f %f %f %f %f\n" % (tuple(I[indeces]))
                importance_floats = [float(x) for x in importance.split()]
                for ind in np.cumsum([0] + range(6, 1, -1))[3:6]:
                    importance_floats[ind] += self.offset
                self.g2o_data.write(' '.join([str(x) for x in importance_floats]) + '\n')
                # self.g2o_data.write("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 %f %f %f %f %f %f\n" % (tuple(I[indeces])))  # no translation importance

                tri = np.zeros((6, 6))
                tri[np.triu_indices(6, 0)] = importance_floats
                tri_updated = tri + np.tril(tri.T, -1)
                value = np.linalg.eigvals(tri_updated)
                if min(value) < self.offset and min(value) != 0:
                    print "found an unexpectedly low Eigenvalue", min(value)
                self.g2o_data.write("FIX %i\n" % (self.vertex_id + 2))

                ## Writing Tag Information ##
                # print ("tags try to record are:", tagtrans.keys())
                for tag_id in tagtrans.keys():  # for each tag found in this recording
                    print ("AR_CALIBRATION: try to record tag")
                    if self.tagrecorded[tag_id]:
                        print("AR_CALIBRATION: recorded old id: %s" % tag_id)
                        self.g2o_data.write("EDGE_SE3:QUAT %i %i %f %f %f %f %f %f %f " % tuple(
                            [self.vertex_id + 1, tag_id] + tagtrans[tag_id] + tagrot[
                                tag_id]))  # write edge from the phone to the tag
                        self.g2o_data.write("%f 0 0 0 0 0 %f 0 0 0 0 %f 0 0 0 %f 0 0 %f 0 %f\n" % (
                            100, 100, 100, 100, 100, 100))  # write edge importance matrix

                # print('record pose')
                self.vertex_id += 2  # increment vertex id
                self.last_record_time = self.nowtime  # set previous record time to current record time.
                self.pose_failure = False
            else:
                print "TRANSFORM FAILURE: odom and real_device at nowtime in self.RecordTime()"
                self.error_count += 1
                print "pose_failure_count:", self.error_count

            ## Writing Testing Data ##
            self.listener.waitForTransform("AR", "real_device", self.nowtime, rospy.Duration(.5))
            if self.listener.canTransform("AR", "odom", self.last_record_time):
                (trans4, rot4) = self.listener.lookupTransform("AR", "real_device",
                                                               self.nowtime)  # lookup The AR to Real Device Transform
                self.test_data.write("PATH %f %f %f %f %f %f %f\n" % tuple(trans4 + rot4))  # write to test file.

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            # print "AR_CALIBRATION: recordTime Exception: " + str(e)
            self.pose_failure = True
            self.error_count += 1
            print "pose_failure_count:", self.error_count

    def run(self):
        r = rospy.Rate(10)  # Attempts to run loop at 10 times per second
        self.last_record_time = rospy.Time.now()  # set last record time to prevent crashing from it being none
        print "Starting Up."
        self.start_speech_engine()
        print "Ready to go."
        while not rospy.is_shutdown():
            if self.calibration_AR and self.recording:  # if calibration_AR and recording
                toffset = rospy.Time.now() - self.last_record_time  # find the time offset between last recorded time and now
                if toffset > self.record_interval:  # if that offset is greater than the record interval
                    self.RecordTime(toffset)  # Record to g2o
            for marker in self.markerlist:  # for marker
                marker.header.stamp = rospy.Time.now()  # set the stamp to now.
            self.waypoint_viz_pub.publish(self.markerlist)  # publish the marker list.
            r.sleep()


if __name__ == '__main__':  # run code
    waypoint_test = ArWaypointTest()
    waypoint_test.run()

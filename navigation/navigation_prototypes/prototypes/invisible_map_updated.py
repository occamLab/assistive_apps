#!/usr/bin/env python

import rospy
from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from apriltags_ros.msg import AprilTagDetectionArray
from keyboard.msg import Key
import tf
from os import path
import pyttsx

try:
    import cPickle as pickle
except:
    import pickle

from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose)
from pose_graph_optimization import PoseGraph

class DataCollection(object):

    def __init__(self, num_tags=587):
        rospy.init_node('data_collection')  # Initialization of another node.
        self.engine = pyttsx.init()  # Speech engine
        self.has_spoken = False  # Boolean for if the speech engine has spoken
        self.package = RosPack().get_path('navigation_prototypes')  # Directory for this ros package
        self.data_folder = path.join(self.package, 'pototypes/raw_data')

        #### Transformation Parameters ####
        self.listener = tf.TransformListener()  # The transform listener
        self.broadcaster = tf.TransformBroadcaster()  # The transform broadcaster

        #### Data Collection Parameters ####
        self.record_interval_normal = rospy.Duration(.1)  # Interval of time between recording normally
        self.record_interval_tag_seen = rospy.Duration(.1)  # Interval of time between recording when tag is seen
        self.record_interval = self.record_interval_normal
        self.tag_record_duration = rospy.Duration(0.025)
        self.transform_wait_time = rospy.Duration(1)
        self.nowtime = None
        self.last_record_time = None
        self.pose_failure = False
        self.curr_pose = None  # current pose of the tango
        self.last_pose = None
        self.pose_failure_count = 0
        self.test_data_count = 0

        #### Tag Specific Parameters ####
        self.num_tags = num_tags
        self.pose_vertex_id = self.num_tags + 1  # starting vertex id for tango odometry data written to pose graph
        self.tags_detected = None  # List of tags seen at the moment
        self.tagtimes = {}
        self.tag_seen = False  # whether the first tag has been detected
        self.test_tag = 2

        #### Data Collection Mode ####
        self.AR_calibration = False
        self.AR_Find_Try = False
        self.Waypoint_Find_Try = False
        self.recording = False

        #### Pose Graph ####
        self.pose_graph = PoseGraph(self.test_tag, self.num_tags)

        #### ROS SUBSCRIBERS ####
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)  # Subscriber for the tango pose.
        rospy.Subscriber('/fisheye_undistorted/tag_detections',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                         AprilTagDetectionArray,
                         self.tag_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)  # Subscriber for the keyboard information.

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

    def process_pose(self, msg):
        """
        Process received tango pose and process pose for waypoints recognition
        """
        self.compute_distance_traveled(msg)

    def compute_distance_traveled(self, msg):
        if self.last_pose and self.tag_seen:  # if we didn't just start up and have seen the origin tag
            new_pose = msg.pose.position
            delta = [new_pose.x - self.last_pose.x, new_pose.y - self.last_pose.y, new_pose.z - self.last_pose.z]
            for i in range(3):
                self.pose_graph.distance_traveled[i] += delta[i]
            self.last_pose = new_pose

    def key_pressed(self, msg):
        """
        This is the callback function for the ROS keyboard.  The msg.code here returns the ord's of what's pressed.
        """
        # Switch to run mode
        if msg.code == ord('t'):  # when user presses R
            """
            Detect AR tag
            """
            self.AR_Find_Try = True  # Try to find and record an AR tag next time possible.

        if msg.code == ord('w'):  # when user presses R
            """
            Detect waypoint
            """
            self.Waypoint_Find_Try = True  # Try to find and record an AR tag next time possible.

        if msg.code == ord('r'):  # When user presses Y
            """
            Turn on AR_Calibration (start and stop data collection.)
            """
            self.calibration_AR = not self.calibration_AR  # Toggle calibration_AR
            if self.calibration_AR:  # If AR Calibration just turned on,
                print('switched to AR Calibration Mode')  # print message
                self.engine.say('switched to AR Calibration Mode')
                self.start_record()
            else:
                print("AR Calibration Mode turned off")
                self.engine.say('turned off AR Calibration Mode')

        if msg.code == ord('u'):
            """
            Start Recording
            """
            if self.recording:
                self.nowtime = rospy.Time.now()

        if msg.code == ord('s'):
            """
            Save Everything.
            """
            with open(path.join(self.package, "data_collected_%d.pkl"), 'wb') as f:  # write to waypoint file
                pickle.dump(self.pose_graph, f)  # DUMP WAYPOINTS INTO THE PICKLE LOCATIONhey
                print("DATA SAVED")

        if msg.code == ord('l'):
            """
            Load Everything.
            """
            with open(path.join(self.package, "data_collected_%d.pkl"), 'rb') as f:  # read pose graph file
                self.pose_graph = pickle.load(f)  # load pickle
                if self.pose_graph.origin_tag is not None:
                    self.tag_seen = True
                self.pose_vertex_id = max(self.pose_graph.odometry_vertices.keys()) + 1
                print("Posegraph LOADED")

    def tag_callback(self, msg):
        """
        Process tag
        """
        # Processes the april tags currently in Tango's view.
        self.tags_detected = None  # Initialize tags_detected
        self.record_interval = self.record_interval_normal  # Initialize the record_interval to normal intervals
        if msg.detections:  # if a tag is detected
            self.record_interval = self.record_interval_tag_seen  # record at the tag_seen record_interval
            self.tags_detected = msg.detections  # save the detected tags
            # get first tag (assume only 1 tag for the most part, if not then this code will just choose one.)
            curr_tag = msg.detections[0]

            # difference between last time tag seen and current time
            time_diff = curr_tag.pose.header.stamp - rospy.Time.now()
            if time_diff > rospy.Duration(1):
                print("THIS IS BAD.")
            if self.gather_transformation(curr_tag.pose.header.frame_id, curr_tag.pose.header.stamp, "odom",
                                          curr_tag.pose.header.stamp,
                                          self.transform_wait_time):
                # Transform the pose from the camera frame to the odom frame.
                curr_tag_transformed_pose = self.listener.transformPose('odom', curr_tag.pose)

                # check if the tag is in the dictionary of tag recording time or not
                if curr_tag.id not in self.tagtimes.keys():
                    self.tagtimes[curr_tag.id] = curr_tag.pose.header.stamp

                # if in Ar calibrate mode and user presses the update button:
                if self.AR_calibration and self.AR_Find_Try:
                    self.record_tag_in_frame(curr_tag, curr_tag_transformed_pose)
            else:
                print "TRANSFORM FAILURE: from tag to odom in tag callback"
        self.AR_Find_Try = False  # Finish trying to find a tag.

    def record_tag_in_frame(self, tag, transformed_pose):
        print "AR_CALIBRATION: executing a find try", tag.id
        # Only prompt user to input tag name once
        if not self.record_tag_vertex(tag, transformed_pose, self.transform_wait_time):
            print("AR_CALIBRATION: No tags recorded.")

        if tag.id == self.test_tag and self.tag_seen:
            self.record_test_data_tag(tag)

    def gather_transformation(self, frame1, time1, frame2, time2, wait_time):
        """
        Check availability of transformation for given frames
        """
        self.listener.waitForTransformFull(frame1, time1, frame2, time2, "odom", wait_time)
        return self.listener.canTransformFull(frame1, time1, frame2, time2, "odom")

    def record_curr_pose_and_damping(self, wait_time):
        """
        Record vertex for current position to pose graph
        Record vertex, edge and importance matrix to pose graph for reducing damping
        """
        if self.gather_transformation("odom", self.nowtime, "real_device", self.nowtime, wait_time):
            (trans, rot) = self.listener.lookupTransformFull("odom", self.nowtime, "real_device", self.nowtime, "odom")
            # add vertex to pose graph for current position
            self.curr_pose = self.pose_graph.add_odometry_vertices(self.pose_vertex_id, trans, rot, False)
            print("RECORDED VERTEX: current pose")
            # add vertex, edge, importance for reducing damping
            self.pose_graph.add_damping(self.curr_pose)
            print("RECORDED VERTEX & EDGE: damping")
            return True
        else:
            self.pose_failure = True
            return False

    def record_tag_vertex(self, tag, transformed_pose, wait_time):
        """
        Record vertex for new tag seen to pose graph
        :param tag: Apriltag object
        :param transformed_pose:
        :param wait_time: Time to wait for receiving transformation
        """
        if self.gather_transformation("odom", tag.pose.header.stamp, "tag_" + str(tag.id), tag.pose.header.stamp,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull("odom", tag.pose.header.stamp, "tag_" + str(tag.id),
                                                             tag.pose.header.stamp, "odom")
            self.pose_graph.add_tag_vertices(tag.id, trans, rot, transformed_pose)
            self.tag_seen = True
            print("RECORDED VERTEX: tag " + str(tag.id))
            return True
        else:
            return False

    def record_waypoint_vertex(self, waypoint_id):
        self.pose_graph.add_waypoint_vertices(waypoint_id, self.curr_pose)
        print ("RECORD VERTEX: waypoint" + waypoint_id)

    def record_pose_to_pose_edge(self, wait_time):
        """
        Record edge between past odometry and current odometry to pose graph
        """
        if self.gather_transformation("real_device", self.last_record_time, "real_device", self.nowtime, wait_time):
            (trans, rot) = self.listener.lookupTransformFull("real_device", self.last_record_time, "real_device",
                                                             self.nowtime, "odom")
            if not self.pose_failure:  # set edge importance to 1
                self.pose_graph.add_pose_to_pose(self.curr_pose, trans, rot, 1)
            else:  # set edge importance to 0
                self.pose_graph.add_pose_to_pose(self.curr_pose, trans, rot, 0)
            print "RECORDED EDGE: Pose to pose transformation"
            return True
        else:
            self.pose_failure = True
            return False

    def record_pose_to_tag_edge(self, tag, wait_time):
        """
        Record edge between current odometry and one of the tag detected to pose graph
        """
        tag_stamp = tag.pose.header.stamp
        if self.listener.canTransform("AR", "odom", self.last_record_time) and self.gather_transformation(
                "real_device", self.nowtime, "tag_" + str(tag.id), tag_stamp, wait_time):
            (trans, rot) = self.listener.lookupTransformFull("real_device", self.nowtime, "tag_" + str(tag.id),
                                                             tag_stamp, "odom")
            if self.pose_graph.add_pose_to_tag(self.curr_pose, tag.id, trans, rot):
                print "RECORDED EDGE: Pose to tag transformation"
                # update last record time for this tag
                self.tagtimes[tag.id] = tag.pose.header.stamp
            else:
                print "RECORD FAILURE: Tag vertex does not exist"
        else:
            print "RECORD FAILURE: Pose to tag transformation"

    def record_all_pose_to_tag_edge(self):
        """
        Record all edges between current odometry to all tags detected to pose graph
        """
        if self.tags_detected and self.tag_seen:
            for tag in self.tags_detected:
                # check time duration since last time an edge is drawn from odom to tag is above certain threshold
                if (tag.pose.header.stamp - self.tagtimes[tag.id]) > self.tag_record_duration:
                    self.record_pose_to_tag_edge(tag, self.transform_wait_time)

    def record_pose_to_waypoint_edge(self, waypoint_id):
        if self.pose_graph.add_pose_to_waypoint(self.curr_pose, waypoint_id):
            print "RECORDED EDGE: Pose to waypoint"
        else:
            print "RECORD FAILURE: WAYPOINT vertex does not exist"

    def record_waypoint_vertices_edges(self):
        if self.Waypoint_Find_Try:
            waypoint_id = raw_input("What do you want to name this waypoint with?")
            self.record_waypoint_vertex(waypoint_id)
            self.record_waypoint_vertex(waypoint_id)
        self.Waypoint_Find_Try = False

    def record_test_data_tag(self, tag):
        try:
            print("tried to record test tag")
            if self.gather_transformation("AR", tag.pose.header.stamp, "tag_" + str(tag.id), tag.pose.header.stamp,
                                          self.transform_wait_time):
                (trans, rot) = self.listener.lookupTransform("AR", "tag_" + str(tag.id), tag.pose.header.stamp)
                self.pose_graph.add_test_data_tag(tag.ID, trans, rot)
                print "RECORDED: test tag transformation"
            else:
                print "RECORD FAILURE: test tag transformation"

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            print "TestTag Exception: " + str(e)

    def record_test_data_path(self):
        """
        Record test data for comparing g2o optimized path with unoptimized path from phone odometry
        """
        if self.gather_transformation("AR", self.nowtime, "real_device", self.nowtime, self.transform_wait_time):
            (trans, rot) = self.listener.lookupTransformFull("AR", self.nowtime, "real_device", self.nowtime, "odom")
            self.pose_graph.add_test_data_path(self.test_data_count, trans, rot)
            self.test_data_count += 1
            print "RECORD: path transformation"
        else:
            print "RECORD FAILURE: path from AR to real device"

    def compute_transform_odom_old_to_new(self, odom_old):
        odom_old_trans = odom_old[0:3]
        odom_old_rot = odom_old[3:7]
        odom_old_pose = PoseStamped(pose=convert_translation_rotation_to_pose(odom_old_trans, odom_old_rot),
                                    header=Header(stamp=self.nowtime, frame_id="AR"))
        self.listener.waitForTransform("AR", "odom", self.nowtime, rospy.Duration(1.0))
        self.odom_old_to_new = self.listener.transformPose("odom", odom_old_pose)
        translation = [self.odom_old_to_new.pose.position.x, self.odom_old_to_new.pose.position.y,
                       self.odom_old_to_new.pose.position.z]
        rotation = [self.odom_old_to_new.pose.orientation.x, self.odom_old_to_new.pose.orientation.y,
                    self.odom_old_to_new.pose.orientation.z, self.odom_old_to_new.pose.orientation.w]
        return translation, rotation

    def start_record(self):
        """
        Start recording for AR Calibration
        """
        if self.AR_calibration and self.recording == False:  # If we aren't recording and AR Calibration is on.
            try:
                self.nowtime = rospy.Time.now()  # Set Nowtime to Now
                if self.record_curr_pose_and_damping(rospy.Duration(2)):
                    print('RECORDING START: first odometry recorded')
                    self.last_record_time = self.nowtime  # set last record time to nowtime as well.
                    self.pose_vertex_id += 2  # add to the vertex id
                    self.recording = True  # set recording true
                    self.tags_detected = None  # set tags detected to none so as to not see tags during the first record.

                else:
                    self.start_record()  # if fail to get transformation, try again

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.Exception,
                    ValueError) as e:
                print "START RECORD Exception: " + str(e)

    def record(self):
        """
        Once first odometry vertex is recorded to pose graph, continue recording to pose graph.
        """
        try:
            self.nowtime = rospy.Time.now()
            # Record vertex of current pose, edge of damping correction, and edge of past and present pose
            if self.record_curr_pose_and_damping(self.transform_wait_time) and self.record_pose_to_pose_edge(
                    self.transform_wait_time):
                self.record_all_pose_to_tag_edge()  # Record edge of current pose to all tags detected
                self.record_waypoint_vertices_edges()  # Record waypoints

                self.pose_vertex_id += 2  # increment vertex id
                self.last_record_time = self.nowtime  # set previous record time to current record time.
                self.pose_failure = False
            else:
                self.pose_failure_count += 1
                print "Pose failure count:", self.pose_failure_count

            # Write Testing Data: path
            self.record_test_data_path()

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            print "AR_CALIBRATION: recordTime Exception: " + str(e)
            self.pose_failure = True
            self.pose_failure_count += 1
            print "Pose failure count:", self.pose_failure_count

    def run(self):
        r = rospy.Rate(10)  # Attempts to run loop at 10 times per second
        self.last_record_time = rospy.Time.now()  # set last record time to prevent crashing from it being none
        print "Starting Up."
        self.start_speech_engine()
        print "Ready to go."
        while not rospy.is_shutdown():
            if self.AR_calibration and self.recording:  # if calibration_AR and recording
                # find the time offset between last recorded time and now
                t_offset = rospy.Time.now() - self.last_record_time
                if t_offset > self.record_interval:
                    self.record()  # Record to pose graph
            r.sleep()

if __name__ == '__main__':
    map_collection = DataCollection()
    map_collection.run()
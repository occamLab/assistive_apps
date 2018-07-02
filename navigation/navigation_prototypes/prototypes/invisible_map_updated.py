#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from apriltags_ros.msg import AprilTagDetectionArray
from keyboard.msg import Key
from os import system, path
from collections import OrderedDict
from tf.transformations import quaternion_from_euler, quaternion_multiply
import numpy as np
import math as math

from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose)


class DataCollection(object):

    def __init__(self, num_tags=587):
        rospy.init_node('ar_waypoint_test')  # Initialization of another node.

        #### Transformation Parameters ####
        self.listener = tf.TransformListener()  # The transform listener
        self.broadcaster = tf.TransformBroadcaster()
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
        self.pose_failure_count = 0
        self.test_data_count = 0

        #### Tag Specific Parameters ####
        self.num_tags = num_tags
        self.pose_vertex_id = self.num_tags + 1  # starting vertex id for tango odometry data written to pose graph
        self.tags_detected = None  # List of tags seen at the moment
        self.tagtimes = {}
        self.tag_seen = False
        self.test_tag = 2

        #### Data Collection Mode ####
        self.AR_calibration = False
        self.AR_Find_Try = False
        self.recording = False

        #### Pose Graph ####
        self.pose_graph = PoseGraph(self.test_tag, self.num_tags)
        if self.pose_graph.origin_tag is not None:
            self.tag_seen = True

        #### ROS SUBSCRIBERS ####
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)  # Subscriber for the tango pose.
        rospy.Subscriber('/fisheye_undistorted/tag_detections',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                         AprilTagDetectionArray,
                         self.tag_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)  # Subscriber for the keyboard information.

    @staticmethod
    def distance_formula(x1, y1, z1, x2, y2, z2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    def process_pose(self, msg):
        """
        Process received tango pose and process pose for waypoints recognition
        """
        self.update_curr_position(msg)
        # Need to look up nearby waypoints
        # Need to compute distance travelled

    def update_curr_position(self, msg):
        try:
            msg.header.stamp = rospy.Time(0)  # set the header stamp to now.
            position = self.listener.transformPose('AR', msg)  # transform the pose into the AR frame
            self.x = position.pose.position.x
            self.y = position.pose.position.y
            self.z = position.pose.position.z  # record the information from that transformed phone pose.
        except Exception as inst:
            print "Exception is", inst

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

        pass

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

    def broadcast_other_AR_frames(self):
        


    def start_record_AR(self):
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
                    self.start_record_AR()  # if fail to get transformation, try again

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.Exception,
                    ValueError) as e:
                print "START RECORD Exception: " + str(e)

    def record_AR(self):
        """
        Once first odometry vertex is recorded to pose graph, continue recording to pose graph.
        """
        try:
            self.nowtime = rospy.Time.now()
            # Record vertex of current pose, edge of damping correction, and edge of past and present pose
            if self.record_curr_pose_and_damping(self.transform_wait_time) and self.record_pose_to_pose_edge(
                    self.transform_wait_time):
                # Record edge of current pose to all tags detected
                self.record_all_pose_to_tag_edge()
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
        # self.start_speech_engine()
        print "Ready to go."
        while not rospy.is_shutdown():
            if self.AR_calibration and self.recording:  # if calibration_AR and recording
                # find the time offset between last recorded time and now
                t_offset = rospy.Time.now() - self.last_record_time
                if t_offset > self.record_interval:
                    self.record_AR()  # Record to pose graph
            r.sleep()


class Vertex(object):
    def __init__(self, ID, trans, rot, type, fix_status=False):
        self.ID = ID
        self.type = type
        self.translation = trans
        self.rotation = rot
        self.fix_status = fix_status

    def write_to_g2o(self, datatype="VERTEX_SE3:QUAT"):
        """
        Write to g2o for recorded vertices
        """
        content = datatype + "%i %f %f %f %f %f %f %f\n" % tuple([self.ID] + self.translation + self.rotation)
        if self.fix_status:
            return content + "FIX %i\n" % self.ID
        else:
            return content


class Edge(object):
    def __init__(self, v_start, v_end, trans, rot, damping_status=False):
        self.start = v_start
        self.end = v_end
        self.translation = trans
        self.rotation = rot
        self.damping_status = damping_status

        #### importance ####
        self.importance_matrix = None
        self.eigenvalue_offset = 10 ** -3
        self.odometry_importance = 1
        self.tag_importance = 100
        self.waypoint_importance = 100
        self.yaw_importance = 0.001
        self.pitch_importance = 1000
        self.roll_importance = 1000
        self.eigenvalue_PSD = False

    @staticmethod
    def null(matrix, rtol=1e-5):
        u, s, v = np.linalg.svd(matrix)
        rank = (s > rtol * s[0]).sum()
        return rank, v[rank:].T.copy()

    def compute_basis_vector(self):
        # Generate a rotation matrix to rotate a small amount around the z axis
        q2 = quaternion_from_euler(0, 0, .05)
        # Rotate current pose by 0.05 degrees in yaw
        qsecondrotation = quaternion_multiply(q2, self.start.rotaton)
        # Get difference in rotated pose with current pose.
        change = (qsecondrotation[0:3] - self.start.rotation[0:3])
        # Determine which direction is the yaw direction and then make sure that direction is diminished in the information matrix
        change = change / np.linalg.norm(change)
        v = change / np.linalg.norm(change)
        _, u = Edge.null(v[np.newaxis])
        basis = np.hstack((v[np.newaxis].T, u))
        # place high information content on pitch and roll and low on changes in yaw
        I = basis.dot(np.diag([self.yaw_importance, self.pitch_importance, self.roll_importance])).dot(basis.T)
        return I

    @staticmethod
    def convert_uppertri_to_matrix(uppertri, size):
        """
        Convert a matrix in uppertriangular form to full matrix form.
        """
        tri = np.zeros((size, size))
        tri[np.triu_indices(size, 0)] = uppertri
        tri_updated = tri + np.tril(tri.T, -1)
        return tri_updated

    def compute_importance_matrix(self):
        if self.damping_status:  # if the edge is for damping correction
            I = self.compute_basis_vector()
            indeces = np.triu_indices(3)  # get indices of upper triangular entry of a 3x3 matrix
            importance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + I[indeces]
            for ind in np.cumsum([0] + range(6, 1, -1))[3:6]:  # increase eigenvalue of rotation importance
                importance[ind] += self.eigenvalue_offset
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        elif self.end.type == "tag":  # if the edge is between current position and a tag detected
            w_t = self.tag_importance
            importance = [w_t, 0, 0, 0, 0, 0, w_t, 0, 0, 0, 0, w_t, 0, 0, 0, w_t, 0, 0, w_t, 0, w_t]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)

        elif self.end.type == "waypoint":  # if the edge is between current position and a waypoint
            w_w = self.waypoint_importance
            importance = [w_w, 0, 0, 0, 0, 0, w_w, 0, 0, 0, 0, w_w, 0, 0, 0, w_w, 0, 0, w_w, 0, w_w]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        else:  # if the edge is between past pose to current pose
            w_o = self.odometry_importance
            importance = [w_o, 0, 0, 0, 0, 0, w_o, 0, 0, 0, 0, w_o, 0, 0, 0, w_o, 0, 0, w_o, 0, w_o]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)

    def check_importance_matrix_PSD(self):
        value = np.linalg.eigvals(self.importance_matrix)
        if min(value) < self.eigenvalue_offset and min(value) != 0:
            print "Found an unexpectedly low Eigenvalue", min(value)
            return False
        else:
            return True

    def write_to_g2o(self, datatype="EDGE_SE3:QUAT"):
        """
        Write to g2o for recorded edges
        """
        return datatype + "%i %i %f %f %f %f %f %f %f" % tuple(
            [self.start.ID, self.end.ID] + self.translation + self.rotation)


class WayPoint(object):
    def __init__(self, id, pose, x, y, z, time_stamp):
        self.id = id
        self.pose = pose
        self.x = x
        self.y = y
        self.z = z
        self.time_stamp = time_stamp


class PoseGraph(object):
    def __init__(self, test_tag_id, num_tags=587):
        #### tag and waypoints recording specific parameters ####
        self.num_tags = num_tags
        self.origin_tag = None  # First tag seen
        self.origin_tag_pose = None
        self.supplement_tags = {}
        self.waypoints = {}

        #### vertices, edges ####
        self.odometry_vertices = OrderedDict()
        self.odometry_edges = OrderedDict()
        self.tag_vertices = OrderedDict()
        self.odometry_tag_edges = OrderedDict()
        self.waypoints_vertices = OrderedDict()
        self.odometry_waypoints_edges = OrderedDict()

        #### g2o Recording ####
        self.g2o_data = None  # Have variable to be prepared for file reading and writing
        self.g2o_data_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data.g2o'  # path to the data compiled into the g2o file
        self.g2o_data_copy_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data_cp.g2o'  # copy of the unedit data
        self.g2o_result_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/result.g2o'  #
        open(self.g2o_data_path, 'wb+').close()  # Overwrite current g2o data file

        #### test data ####
        self.test_tag_id = test_tag_id
        self.testfile = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/naive.txt'
        self.g2o_test_data = open(self.testfile, 'wb+')
        self.test_data_tag = {}
        self.test_data_path = {}

    def add_odometry_vertices(self, ID, trans, rot, fix_status):
        self.odometry_vertices[ID] = Vertex(ID, trans, rot, "odometry", fix_status)
        return self.odometry_vertices[ID]

    def add_odometry_edges(self, v_start, v_end, trans, rot, damping_status):
        self.odometry_edges[v_start.ID] = Edge(v_start, v_end, trans, rot, damping_status)
        return self.odometry_edges[v_start.ID]

    def add_tag_vertices(self, ID, trans, rot, transformed_pose):
        if self.origin_tag is None:
            self.origin_tag = ID
            self.origin_tag_pose = transformed_pose  # make this tag the origin tag
            self.tag_vertices[ID] = Vertex(ID, trans, rot, "tag", True)
            print "AR_CALIBRATION: Origin Tag Found: " + str(ID)
        elif not (ID == self.origin_tag or ID in self.supplement_tags.keys()):
            self.supplement_tags[ID] = transformed_pose  # set new supplemental AR Tag
            self.tag_vertices[ID] = Vertex(ID, trans, rot, "tag", False)
            print "AR_CALIBRATION: Supplementary Tag Found: " + str(ID)
            print(self.supplement_tags.keys())
        elif ID == self.origin_tag:
            self.origin_tag_pose = transformed_pose  # Reset the origin tag
            print "AR_CALIBRATION: Origin Tag Refound: " + str(ID)
        else:
            print "AR_CALIBRATION: Found Old Tag: " + str(ID)

    def add_odometry_tag_edges(self, v_odom, v_tag, trans, rot):
        if v_tag.ID not in self.odometry_tag_edges.keys():
            self.odometry_tag_edges[v_tag.ID] = {}
        self.odometry_tag_edges[v_tag.ID][v_odom] = Edge(v_odom, v_tag, trans, rot)
        return self.odometry_tag_edges[v_tag.ID][v_odom]

    def add_waypoint(self, ID, curr_pose, x, y, z, time_stamp):
        if ID not in self.waypoints.keys():
            self.waypoints[ID] = WayPoint(ID, curr_pose, x, y, z, time_stamp)
            print "AR_CALIBRATION: Waypoint Found:" + ID
        else:
            print "AR_CALIBRATION: Found Old waypoint" + ID  # Need to fix

    def add_waypoint_vertices(self, ID, curr_pose):
        if ID not in self.waypoints_vertices.keys():
            self.waypoints_vertices[ID] = Vertex(ID, curr_pose.trans, curr_pose.rot, "waypoint")
            print "AR_CALIBRATION: Waypoint Found: " + str(ID)
            return self.waypoints_vertices[ID]
        else:
            print "AR_CALIBRATION: Found Old Waypoint: " + str(ID)

    def add_odometry_waypoint_edges(self, v_odom, v_waypoints):
        if v_waypoints not in self.odometry_waypoints_edges.keys():
            self.odometry_waypoints_edges[v_waypoints.ID] = {}
        self.odometry_waypoints_edges[v_waypoints.ID][v_odom] = Edge(v_odom, v_waypoints, [0, 0, 0], [0, 0, 0, 1])
        return self.odometry_waypoints_edges[v_waypoints.ID]

    def add_damping(self, curr_pose):
        """
        Add a vertex and edge for correcting damping
        :param curr_pose: Vertex object for current pose
        """
        damping_vertex = self.add_odometry_vertices(curr_pose.ID + 1, [0, 0, 0], curr_pose.rotation, True)
        damping_edge = self.add_odometry_edges(curr_pose, damping_vertex, [0, 0, 0], [0, 0, 0, 1], True)
        # compute importance matrix
        damping_edge.compute_importance_matrix()
        if damping_edge.check_importance_matrix_PSD():
            damping_edge.eigenvalue_PSD = True

    def add_pose_to_pose(self, curr_pose, trans, rot, importance):
        """
        Add an edge between vertices of current pose and last pose
        :param curr_pose: Vertex object of current pose
        :param trans: translation
        :param rot: rotation
        :param importance: Importance of this new edge
        """
        pose_edge = self.add_odometry_edges(self.odometry_vertices[curr_pose.ID - 1], curr_pose, trans, rot, False)
        pose_edge.odometry_importance = importance
        # compute importance matrix
        pose_edge.compute_importance_matrix()
        if pose_edge.check_importance_matrix_PSD():
            pose_edge.eigenvalue_PSD = True

    def add_pose_to_tag(self, curr_pose, tag_id, trans, rot):
        """
        Add an edge between vertices of current pose and current tag detected
        :param curr_pose: Vertex object of current pose
        :param tag_id: ID current tag detected
        :param trans: translation
        :param rot: rotation
        """
        if tag_id in self.tag_vertices.keys():
            tag = self.tag_vertices[tag_id]
            pose_tag_edge = self.add_odometry_tag_edges(curr_pose, tag, trans, rot)
            # compute importance matrix
            pose_tag_edge.compute_importance_matrix()
            if pose_tag_edge.check_importance_matrix_PSD():
                pose_tag_edge.eigenvalue_PSD = True
            return True
        else:
            return False

    def add_pose_to_waypoint(self, curr_pose, waypoint_id):
        if waypoint_id in self.waypoints_vertices.keys():
            waypoint = self.waypoints_vertices[waypoint_id]
            pose_waypoint_edge = self.add_odometry_waypoint_edges(curr_pose, waypoint)
            pose_waypoint_edge.compute_importance_matrix()
            if pose_waypoint_edge.check_importance_matrix_PSD():
                pose_waypoint_edge.eigenvalue_PSD = True
            return True
        else:
            return False

    def add_test_data_tag(self, ID, trans, rot):
        self.test_data_tag[ID] = trans + rot

    def add_test_data_path(self, ID, trans, rot):
        self.test_data_path[ID] = trans + rot

    def write_g2o_data(self):
        """
        Write to g2o data file
        """
        pass

    def optimize_pose(self):
        """
        Run g2o
        """
        pass

    def write_path_to_test_data(self):
        """
        Write path to test data file
        """
        for path in self.test_data_path.values():
            self.g2o_test_data.write("PATH %f %f %f %f %f %f %f\n" % tuple(path))

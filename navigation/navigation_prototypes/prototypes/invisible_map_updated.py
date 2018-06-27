#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PoseStamped
from apriltags_ros.msg import AprilTagDetectionArray
from keyboard.msg import Key
from os import system, path
from collections import OrderedDict
from tf.transformations import quaternion_from_euler, quaternion_multiply
import numpy as np


class DataCollection(object):

    def __init__(self, num_tags=587):
        rospy.init_node('ar_waypoint_test')  # Initialization of another node.

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
        self.curr_pose = None
        self.pose_failure_count = 0
        self.test_data_count = 0

        #### Tag Specific Parameters ####
        self.num_tags = num_tags
        self.pose_vertex_id = self.num_tags + 1
        self.tags_detected = None
        self.tag_seen = False
        self.origin_tag = None
        self.tagtimes = {}

        #### Data Collection Mode ####
        self.AR_calibration = False
        self.recording = False

        #### Pose Graph ####
        self.pose_graph = PoseGraph(self.num_tags)

        #### ROS SUBSCRIBERS ####
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)  # Subscriber for the tango pose.
        rospy.Subscriber('/fisheye_undistorted/tag_detections',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                         AprilTagDetectionArray,
                         self.tag_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)  # Subscriber for the keyboard information.

    def process_pose(self, msg):
        """
        Process received tango pose and process pose for waypoints recognition
        """
        pass

    def key_pressed(self, msg):
        """
        Process user command
        """
        pass

    def tag_callback(self, msg):
        """
        Process tag
        """
        

        pass

    def gather_transformation(self, frame1, time1, frame2, time2, wait_time):
        """
        Check availability of transformation for given frames
        """
        self.listener.waitForTransformFull(frame1, time1, frame2, time2, "odom", wait_time)
        if self.listener.canTransformFull(frame1, time1, frame2, time2, "odom"):
            return True
        else:
            return False

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

    def record_tag_vertex(self, tag, wait_time, origin_tag=False):
        """
        Record vertex for new tag seen to pose graph
        :param tag: Apriltag object
        :param wait_time: Time to wait for receiving transformation
        :param origin_tag: Boolean to indicate whether the tag is the first one seen
        """
        if self.gather_transformation("odom", tag.pose.header.stamp, "tag_" + str(tag.id), tag.pose.header.stamp,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull("odom", tag.pose.header.stamp, "tag_" + str(tag.id),
                                                             tag.pose.header.stamp, "odom")
            if not origin_tag:  # if not the first tag seen
                self.pose_graph.add_tag_vertices(tag.id, trans, rot)
            else:  # if first tag, fix this tag vertex in pose graph
                self.pose_graph.add_tag_vertices(tag.id, trans, rot, True)
            print("RECORDED VERTEX: tag " + str(tag.id))
            return True
        else:
            return False

    def record_pose_to_pose_edge(self, wait_time):
        """
        Record edge between past odometry and current odometry to pose graph
        """
        if self.gather_transformation("real_device", self.last_record_time, "real_device", self.nowtime, wait_time):
            (trans, rot) = self.listener.lookupTransformFull("real_device", self.last_record_time, "real_device",
                                                             self.nowtime, "odom")
            if not self.pose_failure: # set edge importance to 1
                self.pose_graph.add_pose_to_pose(self.curr_pose, trans, rot, 1)
            else: # set edge importance to 0
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
        if tag.id == self.origin_tag:
            AR_frame = "AR"
        else:
            AR_frame = "AR_" + str(tag.id)

        if self.listener.canTransform(AR_frame, "odom", self.last_record_time) and self.gather_transformation(
                "real_device", self.nowtime, "tag_" + str(tag.id), tag_stamp, wait_time):
            (trans, rot) = self.listener.lookupTransformFull("real_device", self.nowtime, "tag_" + str(tag.id),
                                                             tag_stamp, "odom")
            self.pose_graph.add_pose_to_tag(self.curr_pose, self.pose_graph.tag_vertices[tag.id], trans, rot)
            print "RECORDED EDGE: Pose to tag transformation"
            # update last record time for this tag
            self.tagtimes[tag.id] = tag.pose.header.stamp
        else:
            print "FAILURE: Pose to tag transformation"

    def record_all_pose_to_tag_edge(self):
        """
        Record all edges between current odometry to all tags detected to pose graph
        """
        if self.tags_detected and self.tag_seen:
            for tag in self.tags_detected:
                # check time duration since last time an edge is drawn from odom to tag is above certain threshold
                if (tag.pose.header.stamp - self.tagtimes[tag.id]) > self.tag_record_duration:
                    self.record_pose_to_tag_edge(tag, self.transform_wait_time)

    def record_test_data(self):
        """
        Record test data for comparing g2o optimized path with unoptimized path from phone odometry
        """
        if self.gather_transformation("AR", self.nowtime, "real_device", self.nowtime, self.transform_wait_time):
            (trans, rot) = self.listener.lookupTransformFull("AR", self.nowtime, "real_device", self.nowtime, "odom")
            self.pose_graph.add_test_data_path(self.test_data_count, trans, rot)
            self.test_data_count += 1

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
                    self.start_record_AR() # if fail to get transformation, try again

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

            # Write Testing Data
            self.record_test_data()

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
        if self.damping_status: # if the edge is for damping correction
            I = self.compute_basis_vector()
            indeces = np.triu_indices(3) # get indices of upper triangular entry of a 3x3 matrix
            importance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + I[indeces]
            for ind in np.cumsum([0] + range(6, 1, -1))[3:6]: # increase eigenvalue of rotation importance
                importance[ind] += self.eigenvalue_offset
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        elif self.end.type == "tag": # if the edge is between current position and a tag detected
            w_t = self.tag_importance
            importance = [w_t, 0, 0, 0, 0, 0, w_t, 0, 0, 0, 0, w_t, 0, 0, 0, w_t, 0, 0, w_t, 0, w_t]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        else: # if the edge is between past pose to current pose
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


class PoseGraph(object):
    def __init__(self, num_tags=587):
        self.origin_tag = None
        self.num_tags = num_tags

        #### vertices, edges ####
        self.odometry_vertices = OrderedDict()
        self.odometry_edges = OrderedDict()
        self.tag_vertices = OrderedDict()
        self.odometry_tag_edges = OrderedDict()

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
        self.testfile = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/naive.txt'
        self.g2o_test_data = open(self.testfile, 'wb+')
        self.test_data_tag = {}
        self.test_data_path = {}

    def add_odometry_vertices(self, ID, trans, rot, fix_status):
        self.odometry_vertices[ID] = Vertex(ID, trans, rot, "odometry", fix_status)
        return self.odometry_vertices[ID]

    def add_odometry_edge(self, v_start, v_end, trans, rot, damping_status):
        self.odometry_edges[v_start.ID] = Edge(v_start, v_end, trans, rot, damping_status)
        return self.odometry_edges[v_start.ID]

    def add_tag_vertices(self, ID, trans, rot, fix_status=False):
        self.tag_vertices[ID] = Vertex(ID, trans, rot, "tag", fix_status)
        return self.tag_vertices[ID]

    def add_odometry_tag_edges(self, v_odom, v_tag, trans, rot):
        if v_tag.ID not in self.odometry_tag_edges.keys():
            self.odometry_tag_edges[v_tag.ID] = {}
        self.odometry_tag_edges[v_tag.ID][v_odom] = Edge(v_odom, v_tag, trans, rot)
        return self.odometry_tag_edges[v_tag.ID][v_odom]

    def add_damping(self, curr_pose):
        """
        Add a vertex and edge for correcting damping
        :param curr_pose: Vertex object for current pose
        """
        damping_vertex = self.add_odometry_vertices(curr_pose.ID + 1, [0, 0, 0], curr_pose.rotation, True)
        damping_edge = self.add_odometry_edge(curr_pose, damping_vertex, [0, 0, 0], [0, 0, 0, 1], True)
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
        pose_edge = self.add_odometry_edge(self.odometry_vertices[curr_pose.ID - 1], curr_pose, trans, rot, False)
        pose_edge.odometry_importance = importance
        # compute importance matrix
        pose_edge.compute_importance_matrix()
        if pose_edge.check_importance_matrix_PSD():
            pose_edge.eigenvalue_PSD = True

    def add_pose_to_tag(self, curr_pose, tag, trans, rot):
        """
        Add an edge between vertices of current pose and current tag detected
        :param curr_pose: Vertex object of current pose
        :param tag: Vertex object of current tag detected
        :param trans: translation
        :param rot: rotation
        """
        pose_tag_edge = self.add_odometry_tag_edges(curr_pose, tag, trans, rot)
        # compute importance matrix
        pose_tag_edge.compute_importance_matrix()
        if pose_tag_edge.check_importance_matrix_PSD():
            pose_tag_edge.eigenvalue_PSD = True

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

#!/usr/bin/env python

"""
Data Collection Module for recording data streamed from a phone to construct a pose graph of phone and landmarks positions
at multiple time steps.

by Sherrie Shen & Daniel Connolly, 2018

Last Modified August, 2018

This script will:
- Communicate with Frames class defined in frames.py for broadcasting necessary frames in the tf tree.
- Record data streamed from a phone to construct a pose graph as defined in pose_graph.py
- Record data to a new data file
- Record positions of phone at different time steps as vertices in pose graph.
- Record positions of each april tag seen as vertices in pose graph.
- Record positions of each waypoints as vertices in pose graph.
- Record transformations between consecutive phone positions at two consecutive time stamp as edges.
- Record transformations between phone position and a tag position each time a tag is seen as edges.
- Record transformations between phone position and a waypoint position each time a waypoint is seen as edges.
- Store the data as a pickle file in data/raw_data folder. The default file is named as data_collection.pkl and a copy
of the file is user named.


To use:
- Open Terminal, run the code below:

roslaunch navigation_prototypes data_collection.py
Press "a" in the Ros keyboard input to start collecting data.
Type:
- 1 in the terminal window for starting a new data collection
- 2 in the terminal window for continue recording to a previous data collection

"""

import rospy
from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from apriltags_ros.msg import AprilTagDetectionArray
from keyboard.msg import Key
from navigation_prototypes.srv import CheckMapFrame
from navigation_prototypes.srv import TagSeen
from navigation_prototypes.srv import phone
import tf
from os import path, system
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs import Header, ColorRGBA

import pickle
from pose_graph import PoseGraph, Vertex, Edge


class DataCollection(object):
    """
        The DataCollection class, the main class of this script, calls methods defined in pose graph to construct the
        pose graph from streamed data.
        """

    def __init__(self, filename, num_tags=587):
        rospy.init_node('data_collection')  # Initialization of another node.
        # Directory of ros package for this script
        self.package = RosPack().get_path('navigation_prototypes')
        # Folder where the result will be stored.
        self.data_folder = path.join(self.package, 'data/raw_data')
        # Name of the file to start/continue recording and store the recorded data.
        self.filename = filename
        # set filename as a parameter such that the Frames class would load the same file
        rospy.set_param('posegraph_filename', path.join(
            self.data_folder, self.filename))

        """ Transformation Parameters """
        self.listener = tf.TransformListener()  # The transform listener
        self.broadcaster = tf.TransformBroadcaster()  # The transform broadcaster
        self.odom_frame = "odom"
        self.map_frame = "map"
        # Default origin frame is the odom frame assuming new data collection
        self.origin_frame = self.odom_frame
        self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)

        """Data Collection Parameters"""
        self.record_interval = rospy.Duration(
            .1)  # Interval of time between recording
        self.tag_record_duration = rospy.Duration(
            0.025)  # Interval of time between recording the transformation between phone postion and tag position
        # Time duration to wait for an available transform.
        self.transform_wait_time = rospy.Duration(0.5)
        # Time duration threshold to signify a pose failure
        self.pose_failure_wait_time = rospy.Duration(1)
        self.curr_record_time = None
        self.last_record_time = None
        self.pose_failure = False
        self.curr_pose = None  # Current pose of the tango
        self.last_pose = None
        self.pose_failure_count = 0

        """ Tag Specific Parameters"""
        self.num_tags = num_tags
        self.pose_vertex_id = self.num_tags + 1  # Starting vertex id for odometry data
        self.tags_detected = None  # List of tags seen at the moment
        self.tagtimes = {}  #
        for i in range(self.num_tags):
            self.tagtimes[i] = rospy.Time(1)
        self.tag_in_frame = None

        """ Data Collection Mode """
        self.AR_calibration = False  # Boolean for starting data collection
        # Boolean to check whether the origin frame is available on tf tree
        self.origin_frame_presence = False
        self.AR_Find_Try = False  # Boolean for recording a tag vertex
        self.Waypoint_Find_Try = False  # Boolean for recording a waypoint vertex
        self.recording = False  # Boolean for recording
        self.data_saved = False  # Boolean for indicating data saved

        """ Pose Graph """
        self.pose_graph = PoseGraph(self.num_tags)

        """ ROS Service & Subscribers """
        self.phone = None  # Type of phone used
        self.get_phone_type_client()  # Determine the type of used for data collection
        self.determine_subscribed_topics_from_phone_type()
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)
        # Current tag in frame
        rospy.Service('tag_seen', TagSeen, self.tag_seen_service)
        rospy.Service('check_map_frame', CheckMapFrame,
                      self.check_map_frame_service)  # Name of origin frame

    def determine_subscribed_topics_from_phone_type(self):
        """
        Determine Ros topic to subscribe to for streaming data from phone and April Tag Detection
        :return: None
        """
        print "PHONE: ", self.phone
        if self.phone == "iPhone":
            ''' For use with the iPhone: '''
            rospy.Subscriber('/ios_pose', PoseStamped, self.process_pose)
            rospy.Subscriber('/april_tags_ios',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                             AprilTagDetectionArray,
                             self.tag_callback)
        elif self.phone == "tango":
            ''' For use with the tango: '''
            rospy.Subscriber('/tango_pose', PoseStamped,
                             self.process_pose)  # Subscriber for the tango pose.
            rospy.Subscriber('/fisheye_undistorted/tag_detections',  # SUBSCRIBER FOR THE TAG_DETECTION TRANSFORMS.
                             AprilTagDetectionArray,
                             self.tag_callback)

    def get_phone_type_client(self):
        """
        Determine the type of phone used for Data Collection
        :return: None
        """
        print "CALLING THE SERVICE FOR PHONE TYPE"
        rospy.wait_for_service('/phone_type')
        try:
            get_phone_type = rospy.ServiceProxy('/phone_type', phone)
            self.phone = get_phone_type().phoneType
        except rospy.ServiceException, e:
            print "Service call failed: %s"

    def tag_seen_service(self, req):
        """
        Ros service responsible for offering the id of current detected april tag.
        :param req: None
        :return: id of current detected tag
        """
        if self.tag_in_frame is None:
            service_resp = -1
        else:
            service_resp = self.tag_in_frame
        return service_resp

    def check_map_frame_service(self, req):
        """
        Ros service responsible for checking if the origin frame is the map frame or odom frame.
        :param req: None
        :return: True if origin frame is map frame. False if origin frame is odom frame.
        """
        return self.origin_frame == self.map_frame

    def map_frame_published_client(self):
        """
        Check presence of map frame when continue recording from a previous dataset.
        :return: None
        """
        rospy.wait_for_service('map_frame_published')
        try:
            map_frame_published = rospy.ServiceProxy(
                'map_frame_published', CheckMapFrame)
            response = map_frame_published()
            if response.exist:
                self.origin_frame_presence = True
                print "MAP FRAME BROADCASTED, START DATA COLLECTION"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def process_pose(self, msg):
        """
        Process received tango pose.
        :param msg: Pose stamped message
        """
        self.compute_distance_traveled(msg)

    def compute_distance_traveled(self, msg):
        """
        Compute 3D distance of phone movement.
        :param msg: Pose stamped message
        :return: None
        """
        if self.AR_calibration:
            if self.last_pose:
                new_pose = msg.pose.position
                delta = [new_pose.x - self.last_pose.x, new_pose.y -
                         self.last_pose.y, new_pose.z - self.last_pose.z]
                for i in range(3):
                    self.pose_graph.distance_traveled[i] += delta[i]
                self.last_pose = new_pose
            else:
                self.last_pose = msg.pose.position

    def key_pressed(self, msg):
        """
        This is the callback function for ROS keyboard input.
        :param msg: Keyboard Events
        :return: None
        """

        if msg.code == ord('t'):  # when user presses t
            """
            Detect AR tag
            """
            self.AR_Find_Try = True  # Try to find and record an AR tag next time possible.

        if msg.code == ord('w'):  # when user presses w
            """
            Detect waypoint
            """
            self.Waypoint_Find_Try = True  # Try to find and record an AR tag next time possible.

        if msg.code == ord('a'):  # When user presses a
            """
            Toggle AR_Calibration (start and stop data collection.) for data collection.
            """
            self.AR_calibration = not self.AR_calibration  # Toggle calibration_AR
            if self.AR_calibration:  # If AR Calibration just turned on,
                print('Switched to AR Calibration Mode: New data collection')
                system('espeak "START DATA COLLECTION"')
                self.determine_data_collection_mode()
            else:
                print("AR Calibration Mode turned off")
                self.recording = False
                system('espeak "FINISH DATA COLLECTION"')

        if msg.code == ord('s'):
            """
            Pickle pose graph.
            """
            if not self.recording:
                self.tag_in_frame = None
                with open(path.join(self.data_folder, "data_collected.pkl"), 'wb') as data:
                    pickle.dump(self.pose_graph, data)
                filename = raw_input("How would you like to name this data?")
                with open(path.join(self.data_folder, "%s.pkl" % filename), 'wb') as f:
                    pickle.dump(self.pose_graph, f)
                    print("DATA SAVED")
                    self.data_saved = True
            else:
                print("Please finish AR_calibration before saving pose graph!")

    def tag_callback(self, msg):
        """
        This is the callback function for tag detection
        :param msg: Tag stamped message
        :return: None
        """
        # Processes the april tags currently in Tango's view.
        self.tags_detected = None  # Initialize tags_detected
        if msg.detections:
            self.tags_detected = msg.detections  # save the detected tags
            # get first tag (assume only 1 tag for the most part, if not then this code will just choose one.)
            curr_tag = msg.detections[0]
            self.tag_in_frame = curr_tag.id
            if self.origin_frame_presence:
                self.process_current_tag(curr_tag)
        self.AR_Find_Try = False  # Finish recording a tag vertex.

    def process_current_tag(self, curr_tag):
        """
        :param curr_tag: 
        :return:
        """
        if self.gather_transformation(curr_tag.pose.header.frame_id, curr_tag.pose.header.stamp, self.origin_frame,
                                      curr_tag.pose.header.stamp,
                                      self.transform_wait_time):
            # Transform the pose from the camera frame to the origin frame.
            curr_tag_transformed_pose = self.listener.transformPose(
                self.origin_frame, curr_tag.pose)

            # check if the tag is in the dictionary of tag recording time or not
            if curr_tag.id not in self.tagtimes.keys():
                self.tagtimes[curr_tag.id] = curr_tag.pose.header.stamp

            # if in Ar calibrate mode and user presses the update button:
            if self.AR_calibration and self.AR_Find_Try:
                self.record_tag_in_frame(curr_tag, curr_tag_transformed_pose)
        else:
            print "TRANSFORM FAILURE: from tag to odom in tag callback"

    def record_tag_in_frame(self, tag, transformed_pose):
        """

        :param tag:
        :param transformed_pose:
        :return:
        """
        print "AR_CALIBRATION: executing a find try", tag.id
        # Only prompt user to input tag name once
        if not self.record_tag_vertex(tag, transformed_pose, self.transform_wait_time):
            print("AR_CALIBRATION: No tags recorded.")

    def gather_transformation(self, frame1, time1, frame2, time2, wait_time):
        """
        Check availability of transformation for given frames
        :param frame1:
        :param time1:
        :param frame2:
        :param time2:
        :param wait_time:
        :return:
        """
        self.listener.waitForTransformFull(
            frame1, time1, frame2, time2, self.origin_frame, wait_time)
        return self.listener.canTransformFull(frame1, time1, frame2, time2, self.origin_frame)

    def record_curr_pose_and_damping(self, wait_time):
        """
        Record vertex for current position to pose graph
        Record vertex, edge and importance matrix to pose graph for reducing damping
        :param wait_time:
        :return:
        """
        self.curr_record_time = self.listener.getLatestCommonTime(
            "real_device", self.origin_frame)
        if self.gather_transformation(self.origin_frame, self.curr_record_time, "real_device", self.curr_record_time,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull(self.origin_frame, self.curr_record_time, "real_device",
                                                             self.curr_record_time, self.origin_frame)
            # add vertex to pose graph for current position
            self.curr_pose = self.pose_graph.add_odometry_vertices(
                self.pose_vertex_id, trans, rot, False)
            # print("RECORDED VERTEX: current pose")
            # add vertex, edge, importance for reducing damping
            self.pose_graph.add_damping(self.curr_pose)
            # print("RECORDED VERTEX & EDGE: damping")
            return True
        else:
            self.pose_failure = True
            return False

    def record_pose_at_tag_frame(self, wait_time, record_time):
        if self.gather_transformation(self.origin_frame, record_time, "real_device", record_time,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull(self.origin_frame,
                                                             record_time, "real_device", record_time, self.origin_frame)
            # incremnet pose vertex id
            self.pose_vertex_id += 2
            # add vertex to pose graph for current position
            pose = self.pose_graph.add_odometry_vertices(
                self.pose_vertex_id, trans, rot, False)
            # print("RECORDED VERTEX: current pose")
            # add vertex, edge, importance for reducing damping
            self.pose_graph.add_damping(pose)
            # print("RECORDED VERTEX & EDGE: damping")
            return pose
        else:
            "RECORD FAILURE: Pose at tag frame transformation"
            return None

    def record_tag_vertex(self, tag, transformed_pose, wait_time):
        """
        Record vertex for new tag seen to pose graph
        :param tag: Apriltag object
        :param transformed_pose:
        :param wait_time: Time to wait for receiving transformation
        """
        if self.gather_transformation(self.origin_frame, tag.pose.header.stamp, "tag_" + str(tag.id),
                                      tag.pose.header.stamp,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull(self.origin_frame, tag.pose.header.stamp,
                                                             "tag_" +
                                                             str(tag.id),
                                                             tag.pose.header.stamp, self.origin_frame)
            self.pose_graph.add_tag_vertices(
                tag.id, trans, rot, transformed_pose)
            print("RECORDED VERTEX: tag " + str(tag.id))
            return True
        else:
            return False

    def record_waypoint_vertex(self, waypoint_id):
        """

        :param waypoint_id:
        :return:
        """
        self.pose_graph.add_waypoint_vertices(waypoint_id, self.curr_pose)
        print("RECORD VERTEX: waypoint" + waypoint_id)

    def record_pose_to_pose_edge(self, wait_time):
        """
        Record edge between past odometry and current odometry to pose graph
        """
        if self.gather_transformation("real_device", self.last_record_time, "real_device", self.curr_record_time,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull("real_device", self.last_record_time, "real_device",
                                                             self.curr_record_time, self.origin_frame)
            if self.curr_record_time - self.last_record_time > self.pose_failure_wait_time:
                self.pose_failure = True
                print "POSE FAILURE: Most recent transform time is too far from current time"
            if not self.pose_failure:  # set edge importance to 1
                self.pose_graph.add_pose_to_pose(self.curr_pose, trans, rot, 1)
            else:  # set edge importance to 0
                self.pose_graph.add_pose_to_pose(self.curr_pose, trans, rot, 0)
            print "RECORDED EDGE: Pose to pose transformation"
            return True
        else:
            self.pose_failure = True
            return False

    def record_curr_pose_to_pose_at_tag_frame_edge(self, wait_time, pose_at_tag_frame, tag_stamp):
        """
        Record edge between past odometry and current odometry to pose graph
        """
        if self.gather_transformation("real_device", self.curr_record_time, "real_device", tag_stamp,
                                      wait_time):
            (trans, rot) = self.listener.lookupTransformFull("real_device", self.curr_record_time, "real_device",
                                                             tag_stamp, self.origin_frame)
            self.pose_graph.add_cur_pose_to_pose_at_tag_frame(
                self.curr_pose, pose_at_tag_frame, trans, rot, 1)
            print "RECORDED EDGE: Current pose to pose at tag frame transformation"
            return True
        else:
            return False

    def record_pose_to_tag_edge(self, tag, wait_time):
        """
        Record edge between current odometry and one of the tag detected to pose graph
        """
        if tag.id not in self.pose_graph.tag_vertices.keys():
            print "RECORD FAILURE: Tag vertex does not exist"
            return
        tag_stamp = tag.pose.header.stamp

        if self.gather_transformation("real_device", self.curr_record_time, "tag_" + str(tag.id), tag_stamp, wait_time):
            # (trans, rot) = self.listener.lookupTransformFull("real_device", tag_stamp, "tag_" + str(tag.id),
            #                                                  tag_stamp, self.origin_frame)
            origin_to_tag_trans, origin_to_tag_rot = self.listener.lookupTransformFull(
                self.origin_frame, tag_stamp, "tag_" + str(tag.id), tag_stamp, self.origin_frame)
            device_to_origin_trans, device_to_origin_rot = self.listener.lookupTransformFull(
                'real_device', self.curr_record_time, self.origin_frame, self.curr_record_time, self.origin_frame)

            print("Origin Transform:\n", origin_to_tag_trans, origin_to_tag_rot)
            print("Device Transform:\n",
                  device_to_origin_trans, device_to_origin_rot)

            origin_to_tag_matrix = np.eye(4)
            origin_to_tag_matrix[:3, :3] = R.from_quat(
                origin_to_tag_rot).as_dcm()
            origin_to_tag_matrix[:3, 3] = origin_to_tag_trans
            print("Origin to Tag Matrix:\n", origin_to_tag_matrix)

            device_to_origin_matrix = np.eye(4)
            device_to_origin_matrix[:3, :3] = R.from_quat(
                device_to_origin_rot).as_dcm()
            device_to_origin_matrix[:3, 3] = device_to_origin_trans
            print("Device to Origin Matrix:\n", device_to_origin_matrix)

            transform = origin_to_tag_matrix.dot(device_to_origin_matrix)
            trans = transform[:3, 3]
            rot = R.from_dcm(transform[:3, :3]).as_quat()
            marker_message = Marker(header=Header(stamp=self.curr_record_time, frame_id='real_device'), pose=Pose(position=Point(trans)), scale=Vector3(1,1,1), color=ColorRGBA(0.1, 0, .9, 1), id=1, type=Marker.SPHERE)
            self.marker_pub.publish(marker_message)

            print("Device to Tag Matrix:\n", transform)

            if self.pose_graph.add_pose_to_tag(self.curr_pose, tag.id, trans, rot):
                # print "RECORDED EDGE: Pose to tag transformation"
                # update last record time for this tag
                self.tagtimes[tag.id] = tag.pose.header.stamp
        else:
            print "RECORD FAILURE: Pose to tag transformation"

    def record_all_pose_to_tag_edge(self):
        """
        Record all edges between current odometry to all tags detected to pose graph
        """
        if self.tags_detected:
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
            waypoint_id = raw_input(
                "What do you want to name this waypoint with?")
            self.record_waypoint_vertex(waypoint_id)
            self.record_pose_to_waypoint_edge(waypoint_id)
            self.curr_record_time = self.listener.getLatestCommonTime(
                "real_device", self.origin_frame)
        self.Waypoint_Find_Try = False

    def determine_data_collection_mode(self):
        mode = raw_input("New data collection or continue with old data? 1/2")
        if mode == "1":
            self.origin_frame_presence = True
            self.start_record()
        elif mode == "2":
            # load old pose graph
            with open(path.join(self.data_folder, self.filename), 'rb') as file:  # read pose graph file
                self.pose_graph = pickle.load(file)  # load pickle
                # print "pose graph:", self.pose_graph.origin_tag
                if self.pose_graph.origin_tag is not None:
                    self.origin_frame = self.map_frame
                self.pose_vertex_id = max(
                    self.pose_graph.odometry_vertices.keys()) + 1
                print("Posegraph LOADED FOR DATA COLLECTION")
                self.start_record_with_map_frame()
        else:
            print "Invalid Input"
            self.determine_data_collection_mode()

    def start_record_with_map_frame(self):
        while not self.origin_frame_presence:
            self.map_frame_published_client()
        self.start_record()

    def start_record(self):
        """
        Start recording for AR Calibration
        """
        if self.AR_calibration and self.recording == False:  # If we aren't recording and AR Calibration is on.
            try:
                if self.record_curr_pose_and_damping(rospy.Duration(1)):
                    print('RECORDING START: first odometry recorded')
                    # set last record time to nowtime as well.
                    self.last_record_time = self.curr_record_time
                    self.pose_vertex_id += 2  # add to the vertex id
                    self.recording = True  # set recording true
                else:
                    self.start_record()  # if fail to get transformation, try again
            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.Exception,
                    ValueError) as e:
                print "START RECORD Exception: " + str(e)
                self.start_record()

    def record(self):
        """
        Once first odometry vertex is recorded to pose graph, continue recording to pose graph.
        """
        try:
            # Record vertex of current pose, edge of damping correction, and edge of past and present pose
            if self.record_curr_pose_and_damping(self.transform_wait_time) and self.record_pose_to_pose_edge(
                    self.transform_wait_time):
                # Record edge of current pose to all tags detected
                self.record_all_pose_to_tag_edge()
                self.record_waypoint_vertices_edges()  # Record waypoints
                self.last_record_time = self.curr_record_time
                self.pose_vertex_id += 2  # increment vertex id
                self.pose_failure = False
            else:
                self.pose_failure_count += 1
                self.last_record_time = self.curr_record_time
                print "Pose failure count:", self.pose_failure_count

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            print "AR_CALIBRATION: recordTime Exception: " + str(e)
            self.pose_failure = True
            self.last_record_time = self.curr_record_time
            self.pose_failure_count += 1
            print "Pose failure count:", self.pose_failure_count

    def run(self):
        r = rospy.Rate(10)  # Attempts to run loop at 10 times per second
        print "Starting Up."
        system('espeak "WELCOME TO DATA COLLECTION"')
        while not rospy.is_shutdown():
            if self.AR_calibration and self.recording:  # if calibration_AR and recording
                # find the time offset between last recorded time and now
                t_offset = rospy.Time.now() - self.last_record_time
                if t_offset > self.record_interval:
                    self.record()  # Record to pose graph
            if self.data_saved:
                break
            else:
                r.sleep()

        print "Data Collection Finished."


if __name__ == '__main__':
    map_collection = DataCollection('data_collected.pkl')
    map_collection.run()

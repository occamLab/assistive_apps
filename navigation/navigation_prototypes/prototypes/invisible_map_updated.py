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

    def __init__(self):
        rospy.init_node('ar_waypoint_test')  # Initialization of another node.

        #### Transformation Parameters ####
        self.listener = tf.TransformListener()  # The transform listener
        self.broadcaster = tf.TransformBroadcaster()  # The transform broadcaster

        #### Data Collection Parameters ####
        self.record_interval_normal = rospy.Duration(.1)  # Interval of time between recording normally
        self.record_interval_tag_seen = rospy.Duration(.1)  # Interval of time between recording when tag is seen
        self.nowtime = None
        self.last_record_time = None

        #### Data Collection Mode ####
        self.AR_calibration = False
        self.recording = False

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

    def start_record_AR(self):
        if self.AR_calibration and self.recording == False:  # If we aren't recording and AR Calibration is on.
            try:
                self.nowtime = rospy.Time.now()  # Set Nowtime to Now
                self.listener.waitForTransform("odom", "real_device", self.nowtime, rospy.Duration(2))
                if self.listener.canTransform("odom", "real_device", self.nowtime):
                    # look up the transform for phone's current position.
                    (trans, rot) = self.listener.lookupTransform("odom", "real_device", self.nowtime)

                    # add vertex for current position

                    # add vertex and edge for fixing damping



            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.Exception,
                    ValueError) as e:
                print "START RECORD Exception: " + str(e)

    def record_odom_vertex_edge(self):
        try:
            self.nowtime = rospy.Time.now()  # set nowtime to now
            self.listener.waitForTransform("odom", "real_device", self.nowtime, rospy.Duration(.5))
            if self.listener.canTransform("odom", "real_device", self.nowtime):
                (trans, rot) = self.listener.lookupTransform("odom", "real_device", self.nowtime)  # lookup current pose

                # lookup relative pose from last recorded tango pose to this tango pose
                self.listener.waitForTransformFull("real_device", self.last_record_time, "real_device", self.nowtime,
                                                   "odom", rospy.Duration(.5))
                (trans2, rot2) = self.listener.lookupTransformFull("real_device", self.last_record_time, "real_device",
                                                                   self.nowtime, "odom")



        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            print "AR_CALIBRATION: recordTime Exception: " + str(e)

    def record_tag_vertex(self):
        try:
            pass

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            print "recordTag Exception: " + str(e)

    def record_odom_tag_edge(self):
        try:
            tagtrans = {}  # dictionary of tag translations
            tagrot = {}  # dictionary of tag rotations


        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.Exception,
                ValueError) as e:
            print "recordTag Exception: " + str(e)

    def run(self):
        pass


class Vertex(object):
    def __init__(self, ID, trans, rot, type, fix_status=False):
        self.ID = ID
        self.type = type
        self.translation = trans
        self.rotation = rot
        self.fix_status = fix_status

    def write_to_g2o(self, datatype="VERTEX_SE3:QUAT"):
        """
        Write to g2o for recorded vertices.
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

    @staticmethod
    def null(matrix, rtol=1e-5):
        u, s, v = np.linalg.svd(matrix)
        rank = (s > rtol * s[0]).sum()
        return rank, v[rank:].T.copy()

    def compute_basis_vector(self):
        # Generate a rotation matrix to rotate a small amount around the z axis
        q2 = quaternion_from_euler(0, 0, .05)
        qsecondrotation = quaternion_multiply(q2, self.start.rotaton)
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
        tri = np.zeros((size, size))
        tri[np.triu_indices(size, 0)] = uppertri
        tri_updated = tri + np.tril(tri.T, -1)
        return tri_updated

    def compute_importance_matrix(self):
        if self.damping_status:
            I = self.compute_basis_vector()
            indeces = np.triu_indices(3)
            importance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + I[indeces]
            for ind in np.cumsum([0] + range(6, 1, -1))[3:6]:
                importance[ind] += self.eigenvalue_offset
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        elif self.end.type == "tag":
            w_t = self.tag_importance
            importance = [w_t, 0, 0, 0, 0, 0, w_t, 0, 0, 0, 0, w_t, 0, 0, 0, w_t, 0, 0, w_t, 0, w_t]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        else:
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
        Write to g2o for recorded edges.
        """
        return datatype + "%i %i %f %f %f %f %f %f %f" % tuple(
            [self.start.ID, self.end.ID] + self.translation + self.rotation)


class PoseGraph(object):
    def __init__(self, num_tags=587):
        self.origin_tag = None
        self.num_tags = num_tags

        #### vertices, edges ####
        self.pose_vertex_id = self.num_tags + 1
        self.odometry_vertices = OrderedDict()
        self.odometry_edges = OrderedDict()
        self.tag_vertices = OrderedDict()
        self.odometry_tag_edges = OrderedDict()

        #### g2o Recording ###
        self.g2o_data = None  # Have variable to be prepared for file reading and writing
        self.g2o_data_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data.g2o'  # path to the data compiled into the g2o file
        self.g2o_data_copy_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/data_cp.g2o'  # copy of the unedit data
        self.g2o_result_path = path.expanduser(
            '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/result.g2o'  #
        open(self.g2o_data_path, 'wb+').close()  # Overwrite current g2o data file

    def add_odometry_vertices(self, ID, trans, rot, fix_status):
        self.odometry_vertices[ID] = Vertex(ID, trans, rot, "odometry", fix_status)

    def add_odometry_edge(self, v_start, v_end, trans, rot, damping_status):
        self.odometry_edges[v_start.ID] = Edge(v_start, v_end, trans, rot, damping_status)

    def add_tag_vertices(self, ID, trans, rot):
        self.tag_vertices[ID] = Vertex(ID, trans, rot, "tag", fix_status=True)

    def add_odometry_tag_edges(self, v_odom, v_tag, trans, rot):
        if v_tag.ID not in self.odometry_tag_edges.keys():
            self.odometry_tag_edges[v_tag.ID] = []
        self.odometry_tag_edges[v_tag.ID].append(Edge(v_odom, v_tag, trans, rot))

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

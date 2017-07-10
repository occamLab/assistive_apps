#!/usr/bin/env python

import numpy as np
import rospy
from copy import deepcopy
from keyboard.msg import Key
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import pyttsx
import math
from mobility_games.utils.helper_functions import angle_diff
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA

class Breadcrumbs():
    def __init__(self):
        #   TODO use dynamic reconfigure
        self.path_width = 1.5
        self.crumb_threshold = 3
        self.backtrack = 0
        self.crumb_interval = 1
        self.crumb_radius = 1.5
        self.small_marker_radius = 0.15

        #   Set initial conditions
        self.crumb_list = None
        self.keypoint_list = None
        self.pose = None
        self.yaw = None
        self.drop_crumbs = False
        self.crumbs_dropped = False
        self.follow_crumbs = False

        #   Initialize text to speech engine
        self.has_spoken = False
        self.engine = pyttsx.init()

        #   Map keys to state changes
        self.start_crumb_key = ord("d")
        self.stop_crumb_key = ord("s")
        self.start_nav_key = ord("c")
        self.stop_nav_key = ord("x")

        #   Dictionary for clock directions
        self.direction_dict = {12: "Continue straight toward 12 o'clock",
                            1: "Turn slightly right toward 1 o'clock",
                            2: "Turn right toward 2 o'clock",
                            3: "Turn right toward 3 o'clock",
                            4: "Turn right toward 4 o'clock",
                            5: "Turn around toward 5 o'clock",
                            6: "Turn around toward 6 o'clock",
                            7: "Turn around toward 7 o'clock",
                            8: "Turn left toward 8 o'clock",
                            9: "Turn left toward 9 o'clock",
                            10: "Turn left toward 10 o'clock",
                            11: "Turn slightly left toward 11 o'clock"}

        #   Establish ROS subscribers
        rospy.init_node('breadcrumbs')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)
        self.vis_pub = rospy.Publisher('/key_point', MarkerArray, queue_size=10)
        self.crm_pub = rospy.Publisher('/crm_point', MarkerArray, queue_size=10)

    def process_pose(self, msg):
        """ Determine Tango position as a 1x3 numpy array """
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.pose = np.asarray([[x, y, z]])
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def key_pressed(self, msg):
        """ Change state based on keypresses. """

        #   Start laying out path on keypress
        if msg.code == self.start_crumb_key:
            self.drop_crumbs = True
            self.crumbs_dropped = False
            self.follow_crumbs = False
            print "PATH RECORDING STARTED"
            self.engine.say("Path recording started.")
            self.crumb_list = None
            self.keypoint_list = None
            self.clear_all_markers()

        #   Stop recording path on keypress
        if self.drop_crumbs and msg.code == self.stop_crumb_key:
            self.drop_crumbs = False
            self.crumbs_dropped = True
            self.crumb_list = self.crumb_list[::-1]
            self.keypoint_list = self.calculate_keypoints(self.crumb_list)
            print "PATH RECORDING STOPPED"
            self.engine.say("Path recording stopped.")

        #   Start following path on keypress
        if self.crumbs_dropped and msg.code == self.start_nav_key:
            self.follow_crumbs = True
            print "PATH FOLLOWING STARTED"
            self.engine.say("Path navigation started.")
            self.create_marker((), "keypoint")

        #   Stop following path on keypress
        if self.follow_crumbs and msg.code == self.stop_nav_key:
            self.follow_crumbs = False
            print "PATH FOLLOWING STOPPED"
            self.engine.say("Path navigation stopped.")
            self.clear_all_markers()

    def calculate_keypoints(self, crumb_list):
        """ Takes in list of points in an x by 3 numpy
        array. Calculates keypoints where path turns """

        #   Create a keypoint for the first point in list
        keypoints = crumb_list[0, :].reshape(1, 3)
        edible_crumbs = deepcopy(crumb_list)

        #   Continue to add keypoints at turns until cycled through all points
        while len(edible_crumbs):
            last_keypoint = keypoints[-1, :]
            next_keypoint_index = self.get_next_keypoint(last_keypoint,
                                                        edible_crumbs)
            next_keypoint = edible_crumbs[next_keypoint_index, :]
            edible_crumbs = edible_crumbs[next_keypoint_index + 1:, :]
            keypoints = np.vstack((keypoints, next_keypoint))

        return keypoints

    def get_next_keypoint(self, last_keypoint, edible_crumbs):
        """ Calculates the index of the next keypoint in a path, given a list
        of points and the position of the last keypoint. """

        for index, crumb in enumerate(edible_crumbs):
            current_crumbs = deepcopy(edible_crumbs)
            current_crumbs = current_crumbs[:index + 1, :]

            #   Calculate vector between current point and last keypoint
            point_vec = crumb - last_keypoint
            point_vec = point_vec.T

            #   Rotate point_vec 90 degrees, project onto X-Y plane, and
            #   find unit normal vectors
            norm_vec = np.matmul(np.asarray([[0, 1, 0],
                                            [-1, 0, 0],
                                            [0, 0, 0]]), point_vec)
            unit_norm_vec = norm_vec/np.linalg.norm(norm_vec)
            unit_point_vec = point_vec/np.linalg.norm(point_vec)
            unit_norm_vec_2 = np.cross(unit_point_vec, unit_norm_vec)

            #   TODO make sure that calculating vectors in 3 dimensions
            #   doesn't reduce accuracy

            #   Multiply list of crumbs by unit normal vectors to find distance
            #   from center of path of each point.
            list_of_distances_ud = np.matmul(current_crumbs - last_keypoint,
                                        unit_norm_vec_2)
            list_of_distances_lr = np.matmul(current_crumbs - last_keypoint,
                                        unit_norm_vec)
            list_of_distances = np.sqrt(list_of_distances_ud**2 + \
                                        list_of_distances_lr**2)
            list_of_distances = np.sort(list_of_distances)[::-1]

            if self.has_turned(list_of_distances):
                return index - self.backtrack
        return len(edible_crumbs) - 1

    def has_turned(self, list_of_distances):
        """ Given a list of distances from the centerline, determines whether
        the path has turned. """

        for i, item in enumerate(list_of_distances):

            #   Path has turned if there are more deviant crumbs than threshold
            if i >= self.crumb_threshold:
                return True

            #   Path has not turned if one of first few crumbs is within path
            elif item <= self.path_width:
                return False

        #   Path has not turned if list is very short and cycles through
        return False

    def get_clock_angle(self, target_point):
        """ Determine angle from Tango, in clock numbers, to a target point. """

        #   Determine angle between current pose and target point
        target_point = target_point.reshape(1, 3)
        xdif = target_point[0][0] - self.pose[0][0]
        ydif = target_point[0][1] - self.pose[0][1]
        target_yaw = math.atan2(ydif, xdif)
        difference = angle_diff(self.yaw, target_yaw)

        #   Round to the nearest clock direction
        difference = difference * 6/math.pi + 0.5
        clock_direction = int(difference % 12)
        if clock_direction == 0:
            clock_direction = 12
        return clock_direction

    def start_speech_engine(self):
        """ Initialize pyttsx engine, for text to speech. """

        if not self.has_spoken:
            self.engine.say("Starting up.")
            a = self.engine.runAndWait()
            self.engine.say("Hello.")
            a = self.engine.runAndWait()
            self.has_spoken = True

    def run(self):
        """ Runs the main loop. """

        r = rospy.Rate(10)
        last_crumb = rospy.Time.now()
        self.start_speech_engine()
        while not rospy.is_shutdown():

            #   Wait until Tango starts receiving pose
            if self.pose != None:

                #   Loop for path recording mode
                if self.drop_crumbs:
                    if rospy.Time.now() - last_crumb > \
                                rospy.Duration(self.crumb_interval):
                        last_crumb = rospy.Time.now()
                        if self.crumb_list == None:
                            self.crumb_list = self.pose
                        else:
                            self.crumb_list = np.vstack((self.crumb_list,
                                                        self.pose))
                        self.create_marker(self.crumb_list,
                                            marker_type = "crumb")

                #   Loop for path following mode
                if self.follow_crumbs:
                    self.create_marker(self.keypoint_list, "keypoint")
                    diff_vec = self.pose - self.keypoint_list[0, :]
                    self.dist = np.linalg.norm(diff_vec)

                    #   If within some range of keypoint, navigate to next one.
                    if self.dist <= self.crumb_radius and \
                            len(self.keypoint_list) > 1:
                        print("KEYPOINT FOUND")
                        self.keypoint_list = self.keypoint_list[1:, :]
                        new_diff_vec = self.pose - self.keypoint_list[0, :]
                        new_diff_vec[0][2] = 0
                        self.new_dist = np.linalg.norm(new_diff_vec)
                        self.engine.say(self.announce_directions())
                    elif self.dist <= self.crumb_radius:
                        self.engine.say("You have arrived.")
                        self.follow_crumbs = False
            r.sleep()

    def create_marker(self, marker_pos, marker_type = "keypoint", clear = False):
        """ Publishes a ros marker to visualize keypoints and breadcrumbs. """

        #   Change marker attributes and publisher depending on type
        if marker_type == "keypoint":
            radius = self.crumb_radius * 2
            marker_color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.2)
            pub = self.vis_pub
        elif marker_type == "crumb":
            radius = self.small_marker_radius * 2
            marker_color = ColorRGBA(r=0.3, g=0.6, b=1.0, a=1.0)
            pub = self.crm_pub

        #   Set alpha to 0 if clear is true
        if clear:
            marker_color = ColorRGBA(r=0, g=0, b=0, a=0)

        #   Assemble marker array of all crumb points
        marker_array = MarkerArray()
        for i, item in enumerate(marker_pos):
            marker = Marker(header=Header(frame_id="odom",
                    stamp=rospy.Time.now()),
                    type=Marker.SPHERE,
                    pose=Pose(position=Point(x=item[0], y=item[1])),
                    scale=Vector3(x=radius, y=radius, z=radius),
                    color=marker_color,
                    lifetime=rospy.Duration(9999),
                    id=i)
            marker_array.markers.append(marker)

            if marker_type == "keypoint":
                inner_marker = deepcopy(marker)
                inner_marker.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=1)
                inner_marker.scale = Vector3(x=radius/4.0,
                                            y=radius/4.0,
                                            z=radius/4.0)
                inner_marker.id = 500 + i
                marker_array.markers.append(inner_marker)

        #   Publish markers to rviz
        pub.publish(marker_array)

    def clear_all_markers(self):
        """ Clears all rviz markers, up to 1000. """

        list_of_markers = []
        for i in range(0, 1000):
            list_of_markers.append([0, 0, 0])

        self.create_marker(list_of_markers, marker_type="keypoint", clear=True)
        self.create_marker(list_of_markers, marker_type="crumb", clear=True)


    def announce_directions(self, stairs = True, distances = True):
        """ Generate text for giving auditory directions."""

        clock_direction = self.get_clock_angle(self.keypoint_list[0, :])
        direction = self.direction_dict[clock_direction]

        #   If height changes significantly, give instructions for stairways
        if stairs:
            height_diff = self.keypoint_list[0, 2] - self.pose[0][2]
            slope = height_diff/self.new_dist
            if slope >= 0.5:
                direction = direction + "and walk up the stairs. "
            elif slope <= -0.5:
                direction = direction + "and walk down the stairs. "

        #   Give information for distance to next waypoint
        if distances:
            direction = direction + "for %s meters." % round(self.new_dist, 1)

        return direction


if __name__ == '__main__':
    a = Breadcrumbs()
    a.path_width = 0.7
    a.crumb_threshold = 1
    a.backtrack = 1
    a.crumb_interval = 0.5
    a.crumb_radius = 1
    a.run()

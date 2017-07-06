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
from tf.transformations import euler_from_quaternion
import tf
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import pyttsx
from rospkg import RosPack
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header, ColorRGBA
from keyboard.msg import Key
# from dynamic_reconfigure.server import Server
# from mobility_games.cfg import SemanticWaypointsConfig


class ArWaypointTest(object):
    def __init__(self):
        self.calibration_mode = True        # Flag to keep track of game state -- Calibration/Run Modes
        self.waypoints = {}
        self.waypoints_detected = {}

        self.origin_tag = None                  # Destination tag_id
        self.tag_seen = False
        self.translations = None

        ####Testing Materials####
        self.origin_msg = None
        self.lastpose = None
        self.distance_traveled = [0, 0, 0, 0]
        self.testing_tag_id = 1
        #########################


        # self.start_run = False              # Flag indicating state of gameplay in Run Mode
        # self.start_time = None              # Starting time -- starts when user types in destination
        # self.end_time = None                # End time -- stops when user finds destination
        # self.distance_to_destination = 999  # Distance to destination from current position
        self.proximity_to_destination = 4 #rospy.get_param('~proximity_to_destination', 0.8)

        self.x = None                       # x position of Tango. Start at None because no data have been received yet.
        self.y = None                       # z position of Tango
        self.z = None
        self.yaw = None                     # yaw of Tango

        self.last_play_time = None          #   Last time a beeping noise has been made
        self.last_say_time = None           #   Last time voice instructions have been made                   #   Radius, in meters, game takes place (from initial position)
        self.goal_found = None

        #self.

        self.has_spoken = False
        self.engine = pyttsx.init()         # Speech engine
        top = RosPack().get_path('mobility_games')
        self.sound_folder = path.join(top, 'auditory/sound_files')

        rospy.init_node('ar_waypoint_test')
        self.listener = tf.TransformListener()
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/fisheye_undistorted/tag_detections',
                        AprilTagDetectionArray,
                        self.tag_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)
        # srv = Server(SemanticWaypointsConfig, self.config_callback)

    # def config_callback(self, config, level):
    #     self.proximity_to_destination = config['proximity_to_destination']
    #     return config


    # FOR DEMO ONLY
    #   - Creates fake messages
    # def create_test_messages(self):
    #    messages = []
    #    message_1 = "Hi there! You found it!"
    #    messages.append(message_1)
    #    self.tag_messages[3] = messages

    def process_pose(self, msg):       #    Onngoing function that updates current x, y, and yaw
        if self.origin_tag >= 0:
            if (self.lastpose):
                movement = [msg.pose.position.x - self.lastpose.pose.position.x,
                            msg.pose.position.y - self.lastpose.pose.position.y,
                            msg.pose.position.z - self.lastpose.pose.position.z]
                for i in range(3):
                    self.distance_traveled[i] += abs(movement[i])
            try:
                msg.header.stamp = rospy.Time(0)
                newitem = self.listener.transformPose('AR', msg)
                self.x = newitem.pose.position.x
                self.y = newitem.pose.position.y
                self.z = newitem.pose.position.z

                angles = euler_from_quaternion([msg.pose.orientation.x,
                                                msg.pose.orientation.y,
                                                msg.pose.orientation.z,
                                                msg.pose.orientation.w])
                self.yaw = angles[2]

                if not self.calibration_mode:
                    for waypoint in self.waypoints:
                        #print math.sqrt((self.waypoints[waypoint][0] - self.x)**2 + (self.waypoints[waypoint][1] - self.z)**2)
                        disttopoint = math.sqrt((self.waypoints[waypoint][0] - self.x)**2 + (self.waypoints[waypoint][1] - self.z)**2)
                        if(disttopoint < self.proximity_to_destination):
                            if not self.waypoints_detected[waypoint]:
                                mesg = "Found %s" % waypoint
                                print mesg
                                print "distance to point: " + str(disttopoint)
                                self.engine.say(mesg)
                                self.waypoints_detected[waypoint] = True
                                break
                            else:
                                print "distance to point: " + str(disttopoint)
                        else:
                            self.waypoints_detected[waypoint] = False
            except Exception as inst:
                pass
                print "Exception is", inst
        self.lastpose = msg;

    def det_speech(self, yaw, cur_pos, goal_pos):
        print type(cur_pos[0])

        #   Takes in angle, position, and position of goal. Determines relative
        #   position of goal to Tango and returns text instructions.
        dx = goal_pos[0] - cur_pos[0]
        dy = goal_pos[1] - cur_pos[1]
        angle = math.atan2(dy, dx) - yaw
        angle %= 2*math.pi      #   Makes sure angle is between 0 and 2pi
        if angle < math.pi/6:
            text = "You're on the right path!"
        elif angle < math.pi/3:
            text = "The goal is slightly to your left."
        elif angle < 2*math.pi/3:
            text = "The goal is to your left."
        elif angle < 4*math.pi/3:
            text = "The goal is behind you."
        elif angle < 5*math.pi/3:
            text = "The goal is to your right."
        elif angle < 2*math.pi:
            text = "The goal is slightly right."
        else:
            text = "Something funny happened."
        return text

    # Adds string labels to AR Tags
    # def calibrate_tag(self, tag_id):
    #     self.visited_tags_calibrate.append(tag_id)
    #     self.engine.say("Name the AR Tag!")
    #     tag_name = raw_input("Name the AR Tag: ")
    #     confirm_msg = "Labeled " + tag_name
    #     self.engine.say(confirm_msg)
    #     self.tag_id_to_name[tag_id] = tag_name
    #     self.tag_name_to_id[tag_name] = tag_id
    #     print self.tag_id_to_name

    # Gives option to users to leave a message at an AR tag with tag_id
    # def collect_message(self, tag_id):
    #     prompt = "Would you like to leave a message at " + self.tag_id_to_name[tag_id] + "? [y/n] "
    #     self.engine.say(prompt)
    #     message_prompt = raw_input(prompt)
    #     if message_prompt == 'y':
    #         self.engine.say("Leve a message")
    #         message = raw_input("Leave a message: ")
    #         if tag_id in self.tag_messages:
    #             self.tag_messages[tag_id].append(message)
    #         else:
    #             messages = []
    #             messages.append(message)
    #             self.tag_messages[tag_id] = messages

    # Displays all messages at AR Tag with tag_id
    # def display_messages(self, tag_id):
    #     if tag_id in self.tag_messages:
    #         print "Someone has left you a message:"
    #         self.engine.say('Someone has left you a message')
    #         for message in self.tag_messages[tag_id]:
    #             print message + '\n'
    #             self.engine.say(message)

    def update_origin_pose(self):
        if (self.listener.frameExists("odom")
                and self.listener.frameExists('AR')):

            try:
                t = self.listener.getLatestCommonTime("odom", 'AR')

                # get transform to make tag_frame (parent) & odom (child)
                trans, rot = self.listener.lookupTransform(
                        "odom", 'AR', t)

                self.translations = trans
                # self.rotations = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e

    # Starts new game
    #   - Prompt user to enter destination
    #   - Restart timer
    #   - End game by typing 'done'
    # def start_new_game(self):
    #     self.engine.say("What would you like to find?")
    #     search_tag_name = raw_input("What would you like to find? ")
    #     if search_tag_name in self.tag_name_to_id:
    #         confirm_msg = "Looking for %s" % search_tag_name
    #         self.engine.say(confirm_msg)
    #         self.tag_id = self.tag_name_to_id[search_tag_name]

    #         self.update_destination_pose()

    #         self.start_run = True
    #         self.start_time = time.time()
    #     elif search_tag_name == 'done':
    #         print "Goodbye!"
    #     else:
    #         err_msg = '"%s" does not exist. Try again!' % search_tag_name
    #         self.engine.say(err_msg)
    #         print err_msg
    #         self.start_new_game()

    # def finish_run(self, tag_id):
    #     system('aplay ' + path.join(self.sound_folder, 'ding.wav'))
    #     self.start_run = False
    #     confirm_msg = "Found " + self.tag_id_to_name[tag_id]
    #     print confirm_msg
    #     self.engine.say(confirm_msg)

    #     self.end_time = time.time()
    #     elapsed_time = self.end_time - self.start_time
    #     time_msg = "Took %s seconds!" % round(elapsed_time, 1)
    #     self.engine.say(time_msg)
    #     print time_msg

    #     self.display_messages(tag_id)
    #     self.collect_message(tag_id)
    #     print '==========================' + '\n'
    #     self.start_new_game()

    def tag_callback(self, msg):
        # Processes the april tags currently in Tango's view.
        # Ask user to name april tags
        if msg.detections:
            curr_tag = msg.detections[0]
            curr_tag_pose = curr_tag.pose
            curr_tag_pose.header.stamp = rospy.Time(0)
            curr_tag_transformed_pose = self.listener.transformPose('odom', curr_tag_pose)
            tag_id = curr_tag.id
            if self.calibration_mode:
                # Only prompt user to input tag name once
                if not self.tag_seen:
                    print "Origin Tag Found! "
                    print tag_id
                    self.origin_msg = curr_tag_transformed_pose;
                    self.origin_tag = tag_id
                    self.tag_seen = True
                    self.distance_traveled = [0,0,0,0]
            if self.tag_seen and tag_id == self.testing_tag_id:
                #print total distance traveled since seeing the origin tag
                #print total distance between this tag and the origin tag

                self.distance_traveled[3] = math.sqrt(math.pow(self.distance_traveled[0], 2) + math.pow(self.distance_traveled[1], 2) + math.pow(self.distance_traveled[2], 2))
                #print(curr_tag.pose.pose.position)
                #print(self.origin_msg.pose.pose.position)
                pose_now = curr_tag_transformed_pose.pose.position
                pose_old = self.origin_msg.pose.position
                drift_dist = [pose_now.x-pose_old.x, pose_now.y-pose_old.y, pose_now.z-pose_old.z, 0]

                """[curr_tag_pose.pose.position.x - self.origin_msg.pose.pose.position.x,
                curr_tag_pose.pose.position.y - self.origin_msg.pose.pose.position.y,
                curr_tag.pose.pose.position.z - self.origin_msg.pose.pose.position.z,0]"""

                drift_dist[3] = math.sqrt(math.pow(drift_dist[0], 2) + math.pow(drift_dist[1], 2) + math.pow(drift_dist[2], 2))
                print("Distance Traveled: " + str(self.distance_traveled))
                print("Distance Drifted: " + str(drift_dist));
        # if self.origin_tag:
        #     if msg.detections:
        #         tag_id = msg.detections[0].id
        #         if tag_id == self.origin_tag:
        #             pass #UPDATE TRANSFORM HERE


    def key_pressed(self, msg):
        # Switch to run mode
        if msg.code == ord('a'):
            print "\nStarting Run Mode..."
            self.calibration_mode = False
            # self.start_new_game()

        # Mark a waypoint
        if msg.code == ord('b'):
            waypoint_location = [self.x, self.z, self.y]
            waypoint_name = raw_input("Name the Waypoint: ")
            self.waypoints[waypoint_name] = waypoint_location
            self.waypoints_detected[waypoint_name] = False
            print self.waypoints

    	# Save waypoints
    	if msg.code == ord('s'):
            print("WAYPOINTS SAVED")
            # TODO(rlouie):
            with open('/home/juicyslew/catkin_ws/saved_calibration.pkl', 'wb') as f:
                pickle.dump(self.waypoints, f)

        # Load waypoints
        if msg.code == ord('l'):
            print("WAYPOINTS LOADED")
            with open('/home/juicyslew/catkin_ws/saved_calibration.pkl', 'rb') as f:
                self.waypoints = pickle.load(f)
                print(self.waypoints)

    def start_speech_engine(self):
        if not self.has_spoken:
            self.engine.say("Starting up.")
            a = self.engine.runAndWait()
            self.engine.say("Hello.")
            a = self.engine.runAndWait()
            self.has_spoken = True

    def run(self):
        r = rospy.Rate(10)  #   Runs loop at 10 times per second
        # self.start_speech_engine()
        print("Searching for Tango...")
        self.start_speech_engine()
        print "Here we go"
        while not rospy.is_shutdown():

            # if self.start_run and self.distance_to_destination > self.proximity_to_destination and self.x and self.y and self.translations:
            #     self.update_destination_pose()
            #     if not self.last_say_time or rospy.Time.now() - self.last_say_time > rospy.Duration(7.0):
            #         #   If it has been ten seconds since last speech, give voice instructions
            #         self.last_say_time = rospy.Time.now()
            #         speech = self.det_speech(self.yaw, (self.x, self.y), (self.translations[0], self.translations[1]))
            #         self.engine.say(speech)
            #         # print self.distance_to_destination

            #     if ((not self.last_play_time or
            #         rospy.Time.now() - self.last_play_time > rospy.Duration(6.0/(1+math.exp(-self.distance_to_destination*.3))-2.8)) and
            #         (not self.last_say_time or
            #         rospy.Time.now() - self.last_say_time > rospy.Duration(2.5))):

            #         self.last_play_time = rospy.Time.now()
            #         system('aplay ' + path.join(self.sound_folder, 'beep.wav'))
            #         # print self.distance_to_destination

            r.sleep()

if __name__ == '__main__':
    waypoint_test = ArWaypointTest()
    waypoint_test.run()

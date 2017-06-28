#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class Zoom():

    def __init__(self):
        rospy.init_node("zoom_node")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/zoomed_image", Image)
        self.sub = rospy.Subscriber("/color_camera/image_raw",
                                    Image,
                                    self.get_image)
        self.cam_info = rospy.Subscriber("/color_camera/camera_info",
                                    CameraInfo,
                                    self.get_camera_info)
        self.cam_image = None
        self.cam_D = None
        self.cam_K = None
        self.cam_R = None
        self.cam_P = None

    def get_image(self, msg):
        """ Reads image from ros topic and converts to open_cv format. """
        self.cam_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_camera_info(self, msg):
        """ Reads camera information from rospy topic """

        #   Camera properties
        self.cam_D = msg.D
        self.cam_K = msg.K
        self.cam_R = msg.R
        self.cam_P = msg.P

    def zoom_percent(self, img, percent = 200.0, center = None):
        """ Returns a cropped image by the given zoom amount. A zoom of 100
            returns the same image.

            Params: cv2 image

            Optional params: percent (default 200) """

        #   Determine dimensions of output image
        prop = 100.0/percent
        width, height, channels = img.shape
        new_width = int(prop * width)
        new_height = int(prop * height)

        #   Determine center point of image, if not passed
        if center:
            x_center = center[0]
            y_center = center[1]
        else:
            x_center = int(width/2)
            y_center = int(height/2)

        #   Determine corner pixels of new image
        left_x = x_center - new_width/2
        right_x = x_center + new_width/2
        top_y = y_center - new_height/2
        bot_y = y_center + new_width/2

        #   Crop image
        image_cropped = img[left_x:right_x, top_y:bot_y]
        return image_cropped

    def point_to_pixel(self, point):
        """ Takes a 3D point in color camera frame and returns the corresponding
            pixel coordinates. """

        #   Set parameters for projection
        rvec = np.asarray([0.0, 0.0, 0.0])
        tvec = np.asarray([0.0, 0.0, 0.0])
        cam_matrix = np.asarray(self.cam_K).reshape((3, 3))
        distortion = np.asarray([self.cam_D[0],
                                self.cam_D[1],
                                0,
                                0,
                                self.cam_D[2]])

        #print(rvec.shape, tvec.shape, cam_matrix.shape, distortion.shape, point.shape)
        #print(rvec.dtype, tvec.dtype, cam_matrix.dtype, distortion.dtype, point.dtype)

        #   Calculate projection
        res, _ = cv2.projectPoints(point, rvec, tvec, cam_matrix, distortion)
        x_coord = res[0][0][0]
        y_coord = res[0][0][1]
        res = int(x_coord), int(y_coord)
        print(res)
        return res

    def run(self):
        """ Runs the main loop"""

        while not rospy.is_shutdown():
            r = rospy.Rate(10)
            if self.cam_image != None and self.cam_K != None:

                #   Determine odometry point to focus on
                #   TODO add ability to select point in odometry

                #   Transform odometry point to camera frame
                #   TODO use actual point instead of arbitrary one
                point = np.asarray([[0.1, 0.0, 6]])

                #   Determine focal point pixel
                pixel = self.point_to_pixel(point)

                #   Publish zoomed image
                zoomed_img = self.zoom_percent(self.cam_image, 200)
                self.pub.publish(self.bridge.cv2_to_imgmsg(zoomed_img, "bgr8"))

            r.sleep()

if __name__ == '__main__':
    node = Zoom()
    node.run()

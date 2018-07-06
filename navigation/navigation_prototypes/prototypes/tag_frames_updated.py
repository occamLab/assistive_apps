#!/usr/bin/env python

import rospy
import tf

class Frames:

    def __init__(self):
        rospy.init_node('AR_frame')
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.first_tag = None
        self.translations = None
        self.rotations = None

    def tags_detected(self):
        """
        Returns
        -------
        tags_seen, list of str
            the list of frame strings for the tags which
            have been seen over the history of the listener.
        """
        tags_seen = []
        # extract all frames broadcasted
        frame_strings = self.listener.getFrameStrings()
        for frame in frame_strings:
            # a tag of some id was found
            if 'tag_' in frame:
                tags_seen.append(frame)
        return tags_seen

    def update_AR_odom_transform(self, tag_frame):
        """ Will cache the transform that would make AR
        the parent node and odom as the direct child.
        If a transform fails to lookup, an error is printed,
        and the previous transform remains cached """

        try:
            t = self.listener.getLatestCommonTime("odom", tag_frame)
            if self.listener.canTransform("odom", tag_frame, t):
                # get transform to make tag_frame (parent) & odom (child)
                trans, rot = self.listener.lookupTransform(
                    tag_frame, "odom", t)

                # print trans, rot
                self.translations[tag_frame] = trans
                self.rotations[tag_frame] = rot
                # print ("AR_broadcasted")

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException) as e:
            pass
            # print "error condition 2", e

    def broadcast_AR_odom_transform(self):
        """ Will broadcast the transform between parent node
        AR with odom as the direct child """

        try:
            # print "broadcasting AR_odom transform"
            # print "self.translations", self.translations
            # print "self.rotations", self.rotations

            self.broadcaster.sendTransform(
                self.translations[self.first_tag],  # translation of first tag in list
                self.rotations[self.first_tag],  # rotation of first tag in list
                rospy.get_rostime(),
                "odom",  # child
                "AR")  # parent

        except (KeyError) as e:
            pass
            # print e

    @staticmethod
    def parse_tag_id(tag_x):
        return tag_x.split('_')[1]



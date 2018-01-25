#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import tf
from mobility_games.utils.helper_functions import convert_pose_inverse_transform, convert_translation_rotation_to_pose
import cv2
import os
import pickle as pk

def distance_calc(p1, p2):
    return np.sqrt(sum([(p1[i] - p2[i])**2 for i in range(3)]))

class PathFinding():
    def __init__(self):
        self.vertices = []
        self.threshold = 4 #distance threshold in meters.
        self.g2o_result_path = '/home/juicyslew/catkin_ws/result.g2o'
        self.junction_result_path = os.path.dirname(os.path.abspath(__file__)) + '/testing.path'
        self.detections = []
        self.inds = []
        self.lookthreshold = 2
        self.deletethreshold = 1
    def GatherData(self):
        self.vertices = []
        matlen = 0
        #count = 0
        #first = 1
        lastpos = [0,0,0]
        with open(self.g2o_result_path, 'rb') as g2o_result:
            for line in g2o_result:
                if line.startswith("VERTEX_SE3:QUAT "):
                    line = line.strip().split(' ')
                    #print(line[1:])
                    line = [float(i) for i in line[1:]]
                    if line[0] % 2 == 1 and line[0] >= 587:
                        #if count % 3 == 0:
                        dist = distance_calc(line[1:4], lastpos)
                        lastpos = line[1:4]
                        print()
                        self.vertices.append(tuple(line[0:4]))
                        print("found vertex: " + str(line[0]))
                        matlen += 1
                        #first = 0
                    #count += 1
                #if matlen > 2000:
                #    break;

    def Clean(self):
        ind = 0;
        print("Length: " + str(len(self.vertices)))
        while True:
            if (ind >= len(self.vertices)):
                break;
            ind2 = 0;
            while True:
                if (ind2 >= len(self.vertices)):
                    break;
                if (ind != ind2 and distance_calc(self.vertices[ind][1:4], self.vertices[ind2][1:4]) < self.deletethreshold):
                    print("removed: " + str(self.vertices[ind2][0]))
                    del self.vertices[ind2];
                else:
                    ind2 += 1;
            ind += 1;

    def Save(self):
        #Flesh This out later

    def run(self):
        self.GatherData()
        self.Clean()



if __name__ == "__main__":
    pathFind = PathFinding()
    pathFind.run()

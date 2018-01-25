#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import tf
from mobility_games.utils.helper_functions import convert_pose_inverse_transform, convert_translation_rotation_to_pose
import cv2
import os
import pickle as pk

def CostFunction(p1, p2, threshold):
    distance = distance_calc(p1, p2)#np.sqrt(sum([(p1[i] - p2[i])**2 for i in range(3)]))
    #print(distance)
    if distance > threshold:
        return threshold
    else:
        return 0

def distance_calc(p1, p2):
    return np.sqrt(sum([(p1[i] - p2[i])**2 for i in range(3)]))

def detection_scrub(detections, radius = 10, ijradius = 10, percentile = .97, percentvalue = .01):
    #threshold = np.percentile(detections, percentile);
    dims=np.shape(detections)
    flatdetect = detections.flatten()
    inds = np.argsort(flatdetect)[::-1];
    inds = inds[0:int(len(inds)*(1-percentile))]
    threshold = flatdetect[inds[-1]]
    threshold2 = detections.max() * percentvalue
    #print(len(inds))
    #print(np.shape(inds))
    #print(inds)
    print(dims)
    print(len(inds))
    print(threshold)
    newdetect = []
    count = 0
    for i in inds:
        #print(i)
        #print(orig_ind)
        orig_ind = (int(i/dims[0]), i%dims[0])
        if abs(orig_ind[0] - orig_ind[1]) < ijradius:
            detections[orig_ind[0], orig_ind[1]] = 0.0
        if detections[orig_ind[0], orig_ind[1]] > threshold and detections[orig_ind[0], orig_ind[1]] > threshold2:
            count += 1
            for j in np.arange(-radius/2, radius/2, 1):
                for k in np.arange(-radius/2, radius/2, 1):
                    ind = (orig_ind[0] + j, orig_ind[1] + k)
                    if (0 <= ind[0] and ind[0] < dims[0]) and (0 <= ind[1] and ind[1] < dims[1]) and not (j == 0 and k == 0):
                        detections[ind[0], ind[1]] = 0.0
            newdetect.append(orig_ind[0])
            #newdetect[orig_ind[0], orig_ind[1]] = 100.0
    print(count)
    return newdetect


class Junction_Function():
    def __init__(self):
        self.vertices = []
        self.threshold = 4 #distance threshold in meters.
        self.g2o_result_path = '/home/juicyslew/catkin_ws/result.g2o'
        self.junction_result_path = os.path.dirname(os.path.abspath(__file__)) + '/testing.junc'
        self.matrix = None
        self.detections = []
        self.inds = []
        self.movethreshold = .5
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
                        if dist > self.movethreshold:
                            lastpos = line[1:4]
                            print()
                            self.vertices.append(tuple(line[0:4]))
                            print("found vertex: " + str(line[0]))
                            matlen += 1
                        #first = 0
                    #count += 1
                #if matlen > 2000:
                #    break;
    def MatrixCreation(self):
        vsize = len(self.vertices)
        self.matrix = np.zeros((vsize, vsize))
        self.inds = np.zeros((len(self.vertices), 1))
        for i in range(vsize):
            self.inds[i] = self.vertices[i][0];
            for j in range(vsize):
                if i > j:
                    self.matrix[i,j] = self.threshold;
                else:
                    self.matrix[i,j] = (CostFunction(self.vertices[i][1:4], self.vertices[j][1:4], self.threshold))
    def rescale(self, image):
        return image * 255.0/image.max()


    def FindJunctions(self):
        #filename = 'TestImage.png'
        img = self.rescale(self.matrix).astype(dtype=np.uint8)
        #print(type(img))
        gray = img #cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        #print(np.max(gray))
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray,5,25,0.04)

        print(dst)
        self.detections = detection_scrub(dst, radius = 50, ijradius = 50, percentile = 0.9)
        #print(dst.max())
        #print(dst)


        #### Image Plotting ####
        #result is dilated for marking the corners, not important
        #dst = cv2.dilate(dst,None)
        #dst = cv2.dilate(dst,None)
        #dst = cv2.dilate(dst,None)

        # Threshold for an optimal value, it may vary depending on the image.
        #img[dst>.01*dst.max()]=128

        #cv2.imshow('dst',img)  #Plot the image
        #if cv2.waitKey(0) & 0xff == 27:
        #    cv2.destroyAllWindows()

    def WriteJunctions(self):
        f = open(self.junction_result_path, 'wb')
        pk.dump(self.detections, f)



    def run(self, dataloaded):
        if not dataloaded:
            self.GatherData()

        self.MatrixCreation()
        self.FindJunctions()
        self.WriteJunctions()
        #self.matrix = self.rescale(self.matrix)
        #print(np.shape(self.matrix))

        #cv2.imshow('whatever I want', self.matrix.astype(dtype=np.uint8))
        #if cv2.waitKey(0) & 0xff == 27:
        #    cv2.destroyAllWindows()
        #plt.imshow(self.matrix)
        #plt.colorbar()
        #plt.show()

if __name__ == "__main__":
    juncfunc = Junction_Function()
    juncfunc.run(False)

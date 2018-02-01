#!/usr/bin/env python

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import tf
from mobility_games.utils.helper_functions import convert_pose_inverse_transform, convert_translation_rotation_to_pose
import cv2
import os
import pickle as pk
from sys import float_info
from Queue import PriorityQueue
from copy import copy


def distance_calc(p1, p2):
    return np.sqrt(sum([(p1[i] - p2[i])**2 for i in range(3)]))

class PathFinding():
    def __init__(self):
        self.vertices = []
        self.graphDict = {}
        self.pathinfo = []
        self.g2o_result_path = '/home/juicyslew/catkin_ws/result.g2o'
        self.clean_path = os.path.dirname(os.path.abspath(__file__)) + '/clean.verts'
        self.node_graph = os.path.dirname(os.path.abspath(__file__)) + '/nodes.graph'
        self.chosen_path = os.path.dirname(os.path.abspath(__file__)) + '/test.path'
        self.detections = []
        self.inds = []
        self.connectthreshold = 2
        self.deletethreshold = 1 #minimum distance threshold in meters
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

        f = open(self.clean_path, 'wb')
        pk.dump(self.vertices, f)
        print("Cleaning Finished")

    def CreateGraph(self):
        self.graphDict = {}
        for ind in range(0, len(self.vertices)):
            self.graphDict[ind] = []
            for ind2 in range(0, len(self.vertices)):
                if ind != ind2:
                    dist = distance_calc(self.vertices[ind][1:4], self.vertices[ind2][1:4])
                    if (dist < self.connectthreshold):
                        print("connected " + str(ind) + " to " + str(ind2))
                        self.graphDict[ind].append((ind2, dist))

        f = open(self.node_graph, 'wb')
        pk.dump(self.graphDict, f)
        print(self.graphDict)
        print("Graph Finished")
        pass



    def Astar(self, start, end):
        self.pathinfo = []
        startdist = float_info.max;
        startind = -1;
        for ind in range(0, len(self.vertices)):
            tmpdist = distance_calc(start, self.vertices[ind][1:4]);
            if (tmpdist < startdist):
                startdist = tmpdist;
                startind = ind;


        enddist = float_info.max;
        endind = -1;
        enddists = [];
        for ind in range(0, len(self.vertices)):
            tmpdist = distance_calc(end, self.vertices[ind][1:4]);
            enddists.append(tmpdist)
            if (tmpdist < enddist):
                enddist = tmpdist;
                endind = ind;

        print("startInd = " + str(startind))
        print("endInd = " + str(endind))

        #Astar Algorithm
        current_node = startind;
        endpos = self.vertices[endind][1:4]
        #visited_nodes_total_cost = {startind : enddists[startind]};
        visited_nodes_travel_cost = {startind : 0};
        WaitingNodes = PriorityQueue()
        nodelist = []
        while True:
            nodelist.append(current_node);
            for node in self.graphDict[current_node]:
                if (node[0] in visited_nodes_travel_cost.keys()):
                    if (visited_nodes_travel_cost[current_node] + node[1] < visited_nodes_travel_cost[node[0]]):
                        visited_nodes_travel_cost[node[0]];
                else:
                    WaitingNodes.put_nowait((visited_nodes_travel_cost[current_node] + node[1] + enddists[node[0]], node[0], copy(nodelist)))

            if current_node == endind:
                print ("Path Found!")
                print ("Path: " + str(nodelist))
                print("Path Length: " + str(visited_nodes_travel_cost[current_node]))
                self.pathinfo.append(visited_nodes_travel_cost[current_node])
                self.pathinfo.append(nodelist)
                break;

                #For verifying the path makes sense
                """for nodeind in range(0, len(nodelist)-1):
                    print(self.graphDict[nodelist[nodeind]])
                    print(nodelist[nodeind+1])
                    success = False;
                    for node in self.graphDict[nodelist[nodeind]]:
                        if (nodelist[nodeind+1] == node[0]):
                            success = True;
                            break;

                    if success:
                        print("success")
                    else:
                        print("failure")"""

            print(len(nodelist))
            tmp = WaitingNodes.get_nowait()
            nodelist = tmp[2]
            current_node = tmp[1]
            visited_nodes_travel_cost[current_node] = tmp[0] - enddists[current_node]
            print(current_node)


        f = open(self.chosen_path, 'wb')
        pk.dump(self.pathinfo, f)
        print("Pathfinding Finished")
        pass

    def Plot(self):
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        x = []
        y = []
        z = []
        for node in self.vertices:
            x.append(node[1])
            y.append(node[2])
            z.append(node[3])

        pathx = []
        pathy = []
        pathz = []
        for node in self.pathinfo[1]:
            vert = self.vertices[node]
            pathx.append(vert[1])
            pathy.append(vert[2])
            pathz.append(vert[3])

        Nodes, = plt.plot(x, y, z, 'b*', label = 'Cleaned Nodes')
        Path, = plt.plot(pathx, pathy, pathz, 'm-', label = 'Best Path')

        plt.legend(handles = [Nodes, Path]) #old_path,
        plt.xlabel('X')
        plt.ylabel('Y')
        #plt.zlabel('Z')
        plt.title('Pathfinding Verification Graph')
        plt.grid(True)

        plt.show()

    def Save(self):
        #Flesh This out later
        pass

    def run(self, start, end, clean = False, graph = False, path = False):
        if not (clean or graph or path):
            self.GatherData()
            self.Clean()
        else:
            with open(self.clean_path, 'rb') as cleaned_path:
                self.vertices = pk.load(cleaned_path)

        if not (graph or path):
            self.CreateGraph()
        else:
            with open(self.node_graph, 'rb') as nodegraph:
                self.graphDict = pk.load(nodegraph)

        if not path:
            self.Astar(start, end)
        else:
            with open(self.chosen_path, 'rb') as path:
                self.pathinfo = pk.load(path);

        self.Plot()




if __name__ == "__main__":
    pathFind = PathFinding()
    pathFind.run((50,60,-12), (-30,0,6), True, True)

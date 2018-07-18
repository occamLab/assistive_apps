#!/usr/bin/env python

from pose_graph import PoseGraph, Vertex, Edge
import pickle
from rospkg import RosPack
from os import path


class Optimization:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.processed_data_folder = path.join(self.package, 'data/processed_data')
        self.raw_data_folder = path.join(self.package, 'data/raw_data')
        with open(path.join(self.raw_data_folder, filename), 'rb') as data:
            self.posegraph = pickle.load(data)

    def g2o(self):
        self.posegraph.process_graph()
        with open(path.join(self.processed_data_folder, "data_processed.pkl"), 'wb') as file:
            pickle.dump(self.posegraph, file)
        self.posegraph.optimize_pose()

    def parse_g2o_result(self):
        pass


if __name__ == "__main__":
    process = Optimization("data_collected_copy.pkl")

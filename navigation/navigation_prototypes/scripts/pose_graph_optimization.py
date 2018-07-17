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
        with open(path.join(self.raw_data_folder, filename), 'rb') as f:
            self.posegraph = pickle.load(f)

    def process_graph(self):
        self.posegraph.construct_graph()
        self.posegraph.bfs(self.posegraph.origin_tag)
        self.posegraph.update_posegraph()
        with open(path.join(self.processed_data_folder, "data_processed.pkl"), 'wb') as f:
            pickle.dump(self.posegraph, f)
            print("DATA PROCESSED")


if __name__ == "__main__":
    process = Optimization("data_collected.pkl")
    process.process_graph()

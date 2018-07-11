#!/usr/bin/env python

from pose_graph_optimization import PoseGraph
import pickle
from rospkg import RosPack
from os import path


class Optimization:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.data_folder = path.join(self.package, 'prototypes/processed_data')
        with open(filename, 'rb') as f:
            self.posegraph = pickle.load(f)

    def process_graph(self):
        self.posegraph.construct_graph()
        self.posegraph.bfs(self.posegraph.origin_tag)
        self.posegraph.update_posegraph()
        with open(path.join(self.data_folder, "data_processed.pkl"), 'wb') as f:
            pickle.dump(self.posegraph, f)
            print("DATA PROCESSED")


if __name__ == "__main__":
    process = Optimization("raw_data/tango_7.10.2018.18.00.pkl")
    process.process_graph()

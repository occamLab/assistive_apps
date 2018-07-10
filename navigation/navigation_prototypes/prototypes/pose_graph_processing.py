#!/usr/bin/env python

from pose_graph_optimization import PoseGraph
import pickle
from rospkg import RosPack
from os import path


class ProcessGraph:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.data_folder = path.join(self.package, 'prototypes/processed_data')
        with open(filename, 'rb') as f:
            self.posegraph = pickle.load(f)

    def run(self):
        self.posegraph.construct_graph()
        self.posegraph.bfs(self.posegraph.origin_tag)
        self.posegraph.update_posegraph()
        with open(path.join(self.data_folder, "data_processed.pkl"), 'wb') as f:
            pickle.dump(self.posegraph, f)
            print("DATA PROCESSED")


if __name__ == "__main__":
    process = ProcessGraph("raw_data/iOS_data1.pkl")
    process.run()

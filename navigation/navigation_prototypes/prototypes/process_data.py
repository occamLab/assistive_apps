#!/usr/bin/env python

import numpy as np
from os import path
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_from_matrix, quaternion_from_matrix
from rospkg import RosPack

class ProcessData(object):
    def __init__(self, processed_data_path, g2o_data_path):
        top = RosPack().get_path("navigation_prototypes/prototypes")
        self.g2o_data_path = path.join(top, g2o_data_path)
        self.g2o_processed_data_path = path.join(top, processed_data_path)
        self.g2o_processed_data = None
        open(self.g2o_processed_data_path, 'wb+').close()  # Overwrite current processed g2o data file
        self.allvertices = {}
        self.fixed_vertices = {}
    def GatherData(self):
        self.g2o_processed_data = open(self.g2o_processed_data_path, 'ab')
        with open(self.g2o_data_path, 'rb') as g2o_data:
            for line in g2o_data:
                if line.startswith("VERTEX_SE3:QUAT"):
                    line = [float(data) for data in line.strip().split()[1:]]
                    self.allvertices[int(line[0])] = line[1:]

if __name__ == "__main__":
    process_data_test = ProcessData()
    
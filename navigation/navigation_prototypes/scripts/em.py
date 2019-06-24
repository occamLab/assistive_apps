#!/usr/bin/env python
import pickle
from os import path
from rospkg import RosPack
import numpy as np
from regmodel import EMModel


def edge2vectors(edge, mode):
    observations = np.zeros([6, 18])
    observations[:, mode*6:(mode+1)*6] = np.eye(6)
    errors = edge.translation_diff + edge.rotation_diff
    return (observations.tolist(), errors)


class Analysis:
    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.optimized_data_folder = path.join(
            self.package, 'data/optimized_data')
        self.analyzed_data_folder = path.join(
            self.package, 'data/analyzed_data')
        with open(path.join(self.optimized_data_folder, filename), 'rb') as data:
            self.posegraph = pickle.load(data)

        self.get_observations_and_errors()
        self.success = False
        self.w = np.zeros(18)

    def get_observations_and_errors(self):
        observations = []
        errors = []

        for startid in self.posegraph.odometry_edges:
            for endid in self.posegraph.odometry_edges[startid]:
                edge = self.posegraph.odometry_edges[startid][endid]
                if edge.damping_status:
                    obs, err = edge2vectors(edge, 2)
                    observations.extend(obs)
                    errors.extend(err)
                else:
                    obs, err = edge2vectors(edge, 0)
                    observations.extend(obs)
                    errors.extend(err)

        for startid in self.posegraph.odometry_tag_edges:
            for endid in self.posegraph.odometry_tag_edges[startid]:
                edge = self.posegraph.odometry_tag_edges[startid][endid]
                obs, err = edge2vectors(edge, 1)
                observations.extend(obs)
                errors.extend(err)

        self.observations = np.array(observations)
        self.errors = np.array(errors)

    def getvariance(self):
        res = EMModel(self.observations, self.w, self.errors).run()
        success = res.success

        self.success = success
        self.w = np.array(res.x)
        self.variance = np.exp(self.w)

    def updateEdges(self):
        importance = 1 / np.sqrt(self.variance)
        for startid in self.posegraph.odometry_edges:
            for endid in self.posegraph.odometry_edges[startid]:
                if self.posegraph.odometry_edges[startid][endid].damping_status:
                    self.posegraph.odometry_edges[startid][endid].importance_matrix = np.diag(
                        importance[12:18])
                else:
                    self.posegraph.odometry_edges[startid][endid].importance_matrix = np.diag(
                        importance[0:6])

        for startid in self.posegraph.odometry_tag_edges:
            for endid in self.posegraph.odometry_tag_edges[startid]:
                self.posegraph.odometry_tag_edges[startid][endid].importance_matrix = np.diag(
                    importance[6:12])

    def writePosegraph(self):
        self.posegraph.write_g2o_data()

        with open(path.join(self.analyzed_data_folder, 'data_analyzed.pkl'), 'wb') as data:
            pickle.dump(self.posegraph, data)


def main():
    analysis = Analysis('data_optimized.pkl')
    analysis.getvariance()
    analysis.updateEdges()
    analysis.writePosegraph()
    return analysis


if __name__ == '__main__':
    x = main()
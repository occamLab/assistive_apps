#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from sklearn.neighbors import KernelDensity
import pickle
from os import path
from rospkg import RosPack
from optimize_weights import Heteroskedastic
from pose_graph import Edge


class Edge_Importance:
    def __init__(self, start=None, end=None, x=None, y=None, z=None, yaw=None, pitch=None, roll=None):
        self.start = start
        self.end = end
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll


class Error:
    def __init__(self, x, y, z, yaw, pitch, roll, start, end):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

        self.start = start
        self.end = end

        self.x_sd = []
        self.y_sd = []
        self.z_sd = []
        self.yaw_sd = []
        self.pitch_sd = []
        self.roll_sd = []

        self.count = len(self.x)

    def standard_deviation_no_feature(self):
        x_sd = np.std(np.array(self.x)[:, np.newaxis])
        y_sd = np.std(np.array(self.y)[:, np.newaxis])
        z_sd = np.std(np.array(self.z)[:, np.newaxis])
        yaw_sd = np.std(np.array(self.yaw)[:, np.newaxis])
        pitch_sd = np.std(np.array(self.pitch)[:, np.newaxis])
        roll_sd = np.std(np.array(self.roll)[:, np.newaxis])
        for i in range(self.count):
            self.x_sd.append(x_sd)
            self.y_sd.append(y_sd)
            self.z_sd.append(z_sd)
            self.yaw_sd.append(yaw_sd)
            self.pitch_sd.append(pitch_sd)
            self.roll_sd.append(roll_sd)

    @staticmethod
    def compute_variance_regression_based_one_feature(x_i, w, e):
        """
        :param x_i: a predictor/feature vector
        :param w: array of guessed coefficients for the regression model
        :param e: array of error(response variable)
        :return: the vector of coefficients for the regression model
        """
        regress = Heteroskedastic(x_i, w, e)
        weights = regress.run()
        variance = np.exp(np.matmul(weights, regress.X))
        return variance

    @staticmethod
    def euclidean_distance_to_tag(edges):
        distance = []
        for startid in edges:
            for endid in edges[startid]:
                trans = edges[startid][endid]
                distance.append(
                    np.sqrt((trans[0]) ** 2 + (trans[1]) ** 2 + (trans[2]) ** 2))
        return distance

    def compute_variance_regression_based(self, tag_edge_data, w):
        """
        Given tag edge data, compute the Euclidean distance
        :param edge_data:
        :param error_data:
        :param w:
        :return:
        """
        x_i = np.array(Error.euclidean_distance_to_tag(tag_edge_data))
        for i in range(self.count):
            self.x_sd.append(Error.compute_variance_regression_based_one_feature(
                x_i, w, np.array(self.x)))
            self.y_sd.append(Error.compute_variance_regression_based_one_feature(
                x_i, w, np.array(self.y)))
            self.z_sd.append(Error.compute_variance_regression_based_one_feature(
                x_i, w, np.array(self.z)))
            self.yaw_sd.append(Error.compute_variance_regression_based_one_feature(
                x_i, w, np.array(self.yaw_sd)))
            self.pitch_sd.append(Error.compute_variance_regression_based_one_feature(
                x_i, w, np.array(self.pitch_sd)))
            self.roll_sd.append(Error.compute_variance_regression_based_one_feature(
                x_i, w, np.array(self.roll_sd)))


class Analysis:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.optimized_data_folder = path.join(
            self.package, 'data/optimized_data')
        self.analyzed_data_folder = path.join(
            self.package, 'data/analyzed_data')
        with open(path.join(self.optimized_data_folder, filename), 'rb') as data:
            self.posegraph = pickle.load(data)  # pose graph optimized
            print("Pose graph loaded")
        self.w_initial = np.asarray([0, 1])

        self.odom_error = None
        self.tag_error = None
        self.waypoint_error = None
        self.odom_damping_error = None  # dummy nodes

        self.odom_importance = dict()
        self.tag_importance = dict()
        self.waypoint_importance = dict()
        self.odom_damping_importance = dict()  # dummy nodes

    @staticmethod
    def compute_importance(sd):
        if sd != 0:
            return 1 / sd
        else:
            return 1e5

    def extract_edges_error_class(self, edge, regression_based=False):
        x_error = []
        y_error = []
        z_error = []
        yaw_error = []
        pitch_error = []
        roll_error = []
        start_id = []
        end_id = []
        for startid in edge:
            for endid in edge[startid]:
                x_error.append(edge[startid][endid].translation_diff[0])
                y_error.append(edge[startid][endid].translation_diff[1])
                z_error.append(edge[startid][endid].translation_diff[2])
                yaw_error.append(edge[startid][endid].rotation_diff[0])
                pitch_error.append(edge[startid][endid].rotation_diff[1])
                roll_error.append(edge[startid][endid].rotation_diff[2])
                start_id.append(start_id)
                end_id.append(end_id)

        errors = Error(x_error, y_error, z_error, yaw_error,
                       pitch_error, roll_error, start_id, end_id)
        if regression_based:
            errors.compute_variance_regression_based(
                self.posegraph.odometry_tag_edges, self.w_initial)
        else:
            errors.standard_deviation_no_feature()
        return errors

    @staticmethod
    def compute_importance_class(errors, dummy=False):
        importance = dict()
        for i in range(errors.count):
            start = errors.start[i]
            if start not in importance.keys():
                importance[start] = dict()
            end = errors.end[i]

            if dummy:
                x_w = 0
                y_w = 0
                z_w = 0
            else:
                x_sd = errors.x_sd[i]
                x_w = Analysis.compute_importance(x_sd)

                y_sd = errors.y_sd[i]
                y_w = Analysis.compute_importance(y_sd)

                z_sd = errors.z_sd[i]
                z_w = Analysis.compute_importance(z_sd)

            yaw_sd = errors.yaw_sd[i]
            yaw_w = Analysis.compute_importance(yaw_sd)

            pitch_sd = errors.pitch_sd[i]
            pitch_w = Analysis.compute_importance(pitch_sd)

            roll_sd = errors.roll_sd[i]
            roll_w = Analysis.compute_importance(roll_sd)

            importance[start][end] = Edge_Importance(
                start, end, x_w, y_w, z_w, yaw_w, pitch_w, roll_w)

        return importance

    def compute_importance_graph(self):
        self.odom_damping_importance = Analysis.compute_importance_class(
            self.odom_damping_error, dummy=True)
        self.odom_importance = Analysis.compute_importance_class(
            self.odom_error)
        self.tag_importance = Analysis.compute_importance_class(self.tag_error)
        self.waypoint_importance = Analysis.compute_importance_class(
            self.waypoint_error)

    def sd_based_analysis(self, damping_edges, odom_edges):
        # extract error
        self.odom_damping_error = self.extract_edges_error_class(damping_edges)
        self.odom_error = self.extract_edges_error_class(odom_edges)
        self.tag_error = self.extract_edges_error_class(
            self.posegraph.odometry_tag_edges)
        self.waypoint_error = self.extract_edges_error_class(
            self.posegraph.odometry_waypoints_edges)

        self.compute_importance_graph()

    def regression_based_analysis(self, damping_edges, odom_edges):
        # extract error
        self.odom_error = self.extract_edges_error_class(odom_edges)
        self.odom_damping_error = self.extract_edges_error_class(damping_edges)
        self.tag_error = self.extract_edges_error_class(
            self.posegraph.odometry_tag_edges, True)
        self.waypoint_error = self.extract_edges_error_class(
            self.posegraph.odometry_waypoints_edges)

        # compute importance
        self.compute_importance_graph()

    def plot_kernal_density(self, data):
        data = np.array(data)[:, np.newaxis]
        N = np.shape(data)[0]
        X_plot = np.linspace(min(data), max(data), 1000)[:, np.newaxis]

        true_dens = (0.3 * norm(0, 1).pdf(X_plot[:, 0])
                     + 0.7 * norm(5, 1).pdf(X_plot[:, 0]))

        fig, ax = plt.subplots()

        for kernel in ['gaussian']:
            kde = KernelDensity(kernel=kernel, bandwidth=0.01).fit(data)
            log_dens = kde.score_samples(X_plot)
            ax.plot(X_plot[:, 0], np.exp(log_dens), '-',
                    label="kernel = '{0}'".format(kernel))

        ax.text(6, 0.38, "N={0} points".format(N))

        ax.legend(loc='upper left')
        ax.plot(data[:, 0], -0.005 - 0.01 *
                np.random.random(data.shape[0]), '+k')

        plt.show()

    def separate_odom_damping_edge(self):
        damping_edges = {}
        odom_edges = {}
        for vstart in self.posegraph.odometry_edges.keys():
            for vend in self.posegraph.odometry_edges[vstart].keys():
                if self.posegraph.odometry_edges[vstart][vend].damping_status:
                    damping_edges[vstart] = {}
                    damping_edges[vstart][vend] = self.posegraph.odometry_edges[vstart][vend]
                else:
                    odom_edges[vstart] = {}
                    odom_edges[vstart][vend] = self.posegraph.odometry_edges[vstart][vend]
        return damping_edges, odom_edges

    @staticmethod
    def compute_importance_matrix_one_edge(edge, edge_importance):
        x, y, z = edge_importance.x, edge_importance.y, edge_importance.z
        yaw, pitch, roll = edge_importance.yaw, edge_importance.pitch, edge_importance.roll
        I = Edge.compute_basis_vector(edge.start.rotation, yaw, pitch, roll)
        # get indices of upper triangular entry of a 3x3 matrix
        indeces = np.triu_indices(3)
        importance = [x, 0, 0, 0, 0, 0, y, 0, 0,
                      0, 0, z, 0, 0, 0] + list(I[indeces])
        edge.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)

    @staticmethod
    def compute_importance_matrix(edges, edges_importance):
        for vstart in edges.keys():
            for vend in edges[vstart].keys():
                Analysis.compute_importance_matrix_one_edge(
                    edges[vstart][vend], edges_importance[vstart][vend])

    def update_importance(self, mode=1, odom=True, damping=True, tag=True, waypoint=True):
        """
        :param mode: analysis method
        :param odom: Flag to indicate whether to update odometry importance weights
        :param damping:
        :param tag:
        :param waypoint:
        :return:
        """
        damping_edges, odom_edges = self.separate_odom_damping_edge()
        if mode == 1:
            self.sd_based_analysis(damping_edges, odom_edges)
        elif mode == 2:
            self.regression_based_analysis(damping_edges, odom_edges)

        # compute new importance weights based on data
        if odom:
            Analysis.compute_importance_matrix(
                odom_edges, self.odom_importance)
            print("New importance weights computed for odometry")
        if damping:
            Analysis.compute_importance_matrix(
                damping_edges, self.odom_damping_importance)
            print("New importance weights computed for damping edges")
        if tag:
            Analysis.compute_importance_matrix(
                self.posegraph.odometry_tag_edges, self.tag_importance)
            print("New importance weights computed for tags")
        if waypoint:
            Analysis.compute_importance_matrix(
                self.posegraph.odometry_waypoints_edges, self.waypoint_importance)
            print("New importance weights computed for waypoints")

        # combine odometry and damping edges
        self.combine_odom_damping_edges(damping_edges, odom_edges)

    def combine_odom_damping_edges(self, odom, damping):
        for edge in [odom, damping]:
            for vstart in edge.keys():
                for vend in edge[vstart].keys():
                    importance_matrix = edge[vstart][vend].importance_matrix[:]
                    self.posegraph.odometry_edges[vstart][vend].importance_matrix = importance_matrix

    def print_result(self, edge_error):
        print "x:", edge_error.x_sd
        print "y:", edge_error.y_sd
        print "z:", edge_error.z_sd
        print "yaw:", edge_error.yaw_sd
        print "pitch", edge_error.pitch_sd
        print "roll", edge_error.roll_sd

    def run(self, mode=1, plot=False, odom=True, damping=True, tag=True, waypoint=True):
        self.update_importance(mode=mode, odom=odom,
                               damping=damping, tag=tag, waypoint=waypoint)
        print "Finish updating importance"
        with open(path.join(self.analyzed_data_folder, "data_analyzed.pkl"), 'wb') as data:
            pickle.dump(self.posegraph, data)
            print("Analyzed pose graph saved")

        # print standard deviation
        print "tag standard deviation:"
        self.print_result(self.tag_error)

        print "odom standard deviation:"
        self.print_result(self.odom_error)

        print "dummy nodes standard deviation"
        self.print_result(self.odom_damping_error)

        print "waypoint standard deviation"
        self.print_result(self.waypoint_error)

        if plot:
            data = self.tag_error.roll
            self.plot_kernal_density(data)
            data = self.tag_error.yaw
            self.plot_kernal_density(data)
            data = self.tag_error.pitch
            self.plot_kernal_density(data)


if __name__ == "__main__":
    # distribution = Analysis("academic_center.pkl")
    distribution = Analysis("data_optimized.pkl")
    distribution.run(mode=1, plot=False, odom=True,
                     damping=True, tag=True, waypoint=False)

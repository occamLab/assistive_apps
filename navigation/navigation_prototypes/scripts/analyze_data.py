import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from sklearn.neighbors import KernelDensity
import pickle
from os import path
from rospkg import RosPack
import tf
from pose_graph import Edge


class Analysis:

    def __init__(self, filename):
        self.package = RosPack().get_path('navigation_prototypes')
        self.optimized_data_folder = path.join(self.package, 'data/optimized_data')
        self.analyzed_data_folder = path.join(self.package, 'data/analyzed_data')
        with open(path.join(self.optimized_data_folder, filename), 'rb') as data:
            self.posegraph = pickle.load(data)  # pose graph optimized
            print("Pose graph loaded")
        self.error_sd = dict()
        self.error_sd["odom"] = dict()
        self.error_sd["odom_damping"] = dict()
        self.error_sd["tag"] = dict()
        self.error_sd["waypoint"] = dict()

    @staticmethod
    def extract_edges_error(edge):
        error_sd = dict()
        error_sd["x"] = []
        error_sd["y"] = []
        error_sd["z"] = []
        error_sd["yaw"] = []
        error_sd["pitch"] = []
        error_sd["roll"] = []
        for startid in edge:
            for endid in edge[startid]:
                error_sd["x"].append(edge[startid][endid].translation_diff[0])
                error_sd["y"].append(edge[startid][endid].translation_diff[1])
                error_sd["z"].append(edge[startid][endid].translation_diff[2])
                error_sd["yaw"].append(edge[startid][endid].rotation_diff[0])
                error_sd["pitch"].append(edge[startid][endid].rotation_diff[1])
                error_sd["roll"].append(edge[startid][endid].rotation_diff[2])

        # compute sd
        error_sd["x_sd"] = np.std(np.array(error_sd["x"])[:, np.newaxis])
        error_sd["y_sd"] = np.std(np.array(error_sd["y"])[:, np.newaxis])
        error_sd["z_sd"] = np.std(np.array(error_sd["z"])[:, np.newaxis])
        error_sd["yaw_sd"] = np.std(np.array(error_sd["yaw"])[:, np.newaxis])
        error_sd["pitch_sd"] = np.std(np.array(error_sd["pitch"])[:, np.newaxis])
        error_sd["roll_sd"] = np.std(np.array(error_sd["roll"])[:, np.newaxis])

        # compute importance
        error_sd["x_importance"] = Analysis.compute_importance(error_sd["x_sd"])
        error_sd["y_importance"] = Analysis.compute_importance(error_sd["y_sd"])
        error_sd["z_importance"] = Analysis.compute_importance(error_sd["z_sd"])
        error_sd["yaw_importance"] = Analysis.compute_importance(error_sd["yaw_sd"])
        error_sd["pitch_importance"] = Analysis.compute_importance(error_sd["pitch_sd"])
        error_sd["roll_importance"] = Analysis.compute_importance(error_sd["roll_sd"])

        return error_sd

    @staticmethod
    def compute_importance(data):
        if data != 0:
            return 1 / data
        else:
            return 1e5

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
        ax.plot(data[:, 0], -0.005 - 0.01 * np.random.random(data.shape[0]), '+k')

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
    def compute_importance_matrix(edges, importance_value):
        for vstart in edges.keys():
            for vend in edges[vstart].keys():
                x, y, z = importance_value["x_importance"], importance_value["y_importance"], importance_value[
                    "z_importance"]
                yaw, pitch, roll = importance_value["yaw_importance"], importance_value["pitch_importance"], \
                                   importance_value["roll_importance"]
                I = Edge.compute_basis_vector(edges[vstart][vend].start.rotation, yaw, pitch, roll)
                indeces = np.triu_indices(3)  # get indices of upper triangular entry of a 3x3 matrix
                importance = [x, 0, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0] + list(I[indeces])
                edges[vstart][vend].importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)

    def update_importance(self, odom=True, damping=True, tag=True, waypoint=True):
        """

        :param odom: Flag to indicate whether to update odometry importance weights
        :param damping:
        :param tag:
        :param waypoint:
        :return:
        """
        damping_edges, odom_edges = self.separate_odom_damping_edge()
        # compute error standard deviation: example: self.error_sd["odom_damping"]["x_importance"] = 0.001
        self.error_sd["odom_damping"] = Analysis.extract_edges_error(damping_edges)
        self.error_sd["odom"] = Analysis.extract_edges_error(odom_edges)
        self.error_sd["tag"] = Analysis.extract_edges_error(self.posegraph.odometry_tag_edges)
        self.error_sd["waypoint"] = Analysis.extract_edges_error(self.posegraph.odometry_waypoints_edges)

        self.error_sd["odom_damping"]["x_importance"] = 0
        self.error_sd["odom_damping"]["y_importance"] = 0
        self.error_sd["odom_damping"]["z_importance"] = 0
        # self.error_sd["odom_damping"]["yaw_importance"] = 0.01
        # self.error_sd["odom_damping"]["pitch_importance"] = 10000
        # self.error_sd["odom_damping"]["roll_importance"] = 10000

        # compute new importance weights based on data
        if odom:
            Analysis.compute_importance_matrix(odom_edges, self.error_sd["odom"])
            print("New importance weights computed for odometry")
        if damping:
            Analysis.compute_importance_matrix(damping_edges, self.error_sd["odom_damping"])
            print("New importance weights computed for damping edges")
        if tag:
            Analysis.compute_importance_matrix(self.posegraph.odometry_tag_edges, self.error_sd["tag"])
            print("New importance weights computed for tags")
        if waypoint:
            Analysis.compute_importance_matrix(self.posegraph.odometry_waypoints_edges, self.error_sd["waypoint"])
            print("New importance weights computed for waypoints")

        # combine odometry and damping edges
        self.combine_odom_damping_edges(damping_edges, odom_edges)

    def combine_odom_damping_edges(self, odom, damping):
        for edge in [odom, damping]:
            for vstart in edge.keys():
                for vend in edge[vstart].keys():
                    importance_matrix = edge[vstart][vend].importance_matrix[:]
                    self.posegraph.odometry_edges[vstart][vend].importance_matrix = importance_matrix

    def run(self, plot=False, odom=True, damping=True, tag=True, waypoint=True):
        self.update_importance(odom=odom, damping=damping, tag=tag, waypoint=waypoint)
        print "Finish updating importance"
        with open(path.join(self.analyzed_data_folder, "data_analyzed.pkl"), 'wb') as data:
            pickle.dump(self.posegraph, data)
            print("Analyzed pose graph saved")

        # print new computed importance weights
        for edgetype in self.error_sd:
            print edgetype, self.error_sd[edgetype]["x_importance"]
            print edgetype, self.error_sd[edgetype]["y_importance"]
            print edgetype, self.error_sd[edgetype]["z_importance"]
            print edgetype, self.error_sd[edgetype]["yaw_importance"]
            print edgetype, self.error_sd[edgetype]["pitch_importance"]
            print edgetype, self.error_sd[edgetype]["roll_importance"]

        if plot:
            data = self.error_sd["odom_damping"]["roll"]
            self.plot_kernal_density(data)
            data = self.error_sd["odom_damping"]["yaw"]
            self.plot_kernal_density(data)
            data = self.error_sd["odom_damping"]["pitch"]
            self.plot_kernal_density(data)


if __name__ == "__main__":
    #distribution = Analysis("academic_center.pkl")
    distribution = Analysis("data_optimized.pkl")
    distribution.run(plot=False, odom=True, damping=True, tag=True, waypoint=False)

from collections import OrderedDict
from tf.transformations import quaternion_from_euler, quaternion_multiply
from os import path, system
import numpy as np
from rospkg import RosPack
from collections import deque


class Vertex(object):
    def __init__(self, id, trans, rot, type, fix_status=False):
        self.id = id
        self.type = type
        self.translation = trans
        self.rotation = rot
        self.fix_status = fix_status

    def write_to_g2o(self, datatype="VERTEX_SE3:QUAT"):
        """
        Write to g2o for recorded vertices
        """
        content = datatype + "%i %f %f %f %f %f %f %f\n" % tuple([self.id] + self.translation + self.rotation)
        if self.fix_status:
            return content + "FIX %i\n" % self.id
        else:
            return content


class Edge(object):
    def __init__(self, v_start, v_end, trans, rot, damping_status=False):
        self.start = v_start
        self.end = v_end
        self.translation = trans
        self.rotation = rot
        self.damping_status = damping_status
        self.translation_computed = None
        self.rotation_computed = None

        #### importance ####
        self.importance_matrix = None
        self.eigenvalue_offset = 10 ** -3
        self.odometry_importance = 1
        self.tag_importance = 100
        self.waypoint_importance = 100
        self.yaw_importance = 0.001
        self.pitch_importance = 1000
        self.roll_importance = 1000
        self.eigenvalue_PSD = False

    @staticmethod
    def null(matrix, rtol=1e-5):
        u, s, v = np.linalg.svd(matrix)
        rank = (s > rtol * s[0]).sum()
        return rank, v[rank:].T.copy()

    def compute_basis_vector(self):
        # Generate a rotation matrix to rotate a small amount around the z axis
        q2 = quaternion_from_euler(0, 0, .05)
        # Rotate current pose by 0.05 degrees in yaw
        qsecondrotation = quaternion_multiply(q2, self.start.rotation)
        # Get difference in rotated pose with current pose.
        change = (qsecondrotation[0:3] - self.start.rotation[0:3])
        # Determine which direction is the yaw direction and then make sure that direction is diminished in the information matrix
        change = change / np.linalg.norm(change)
        v = change / np.linalg.norm(change)
        _, u = Edge.null(v[np.newaxis])
        basis = np.hstack((v[np.newaxis].T, u))
        # place high information content on pitch and roll and low on changes in yaw
        I = basis.dot(np.diag([self.yaw_importance, self.pitch_importance, self.roll_importance])).dot(basis.T)
        return I

    @staticmethod
    def convert_uppertri_to_matrix(uppertri, size):
        """
        Convert a matrix in uppertriangular form to full matrix form.
        """
        tri = np.zeros((size, size))
        tri[np.triu_indices(size, 0)] = uppertri
        tri_updated = tri + np.tril(tri.T, -1)
        return tri_updated

    @staticmethod
    def convert_matrix_uppertri_list(matrix, size):
        return list(matrix[np.triu_indices(size)])

    def compute_importance_matrix(self):
        if self.damping_status:  # if the edge is for damping correction
            I = self.compute_basis_vector()
            indeces = np.triu_indices(3)  # get indices of upper triangular entry of a 3x3 matrix
            importance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + list(I[indeces])
            for ind in np.cumsum([0] + range(6, 1, -1))[3:6]:  # increase eigenvalue of rotation importance
                importance[ind] += self.eigenvalue_offset
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        elif self.end.type == "tag":  # if the edge is between current position and a tag detected
            w_t = self.tag_importance
            importance = [w_t, 0, 0, 0, 0, 0, w_t, 0, 0, 0, 0, w_t, 0, 0, 0, w_t, 0, 0, w_t, 0, w_t]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)

        elif self.end.type == "waypoint":  # if the edge is between current position and a waypoint
            w_w = self.waypoint_importance
            importance = [w_w, 0, 0, 0, 0, 0, w_w, 0, 0, 0, 0, w_w, 0, 0, 0, w_w, 0, 0, w_w, 0, w_w]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)
        else:  # if the edge is between past pose to current pose
            w_o = self.odometry_importance
            importance = [w_o, 0, 0, 0, 0, 0, w_o, 0, 0, 0, 0, w_o, 0, 0, 0, w_o, 0, 0, w_o, 0, w_o]
            self.importance_matrix = Edge.convert_uppertri_to_matrix(importance, 6)

    def check_importance_matrix_PSD(self):
        value = np.linalg.eigvals(self.importance_matrix)
        if min(value) < self.eigenvalue_offset and min(value) != 0:
            print "Found an unexpectedly low Eigenvalue", min(value)
            return False
        else:
            return True

    def write_to_g2o(self, datatype="EDGE_SE3:QUAT"):
        """
        Write to g2o for recorded edges
        """
        return datatype + "%i %i %f %f %f %f %f %f %f" % tuple(
            [self.start.id, self.end.id] + self.translation + self.rotation)

    def write_to_g2o_importance(self):
        importance_uppertri = Edge.convert_matrix_uppertri_list(self.importance_matrix, 6)
        return "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % tuple(importance_uppertri)


class PoseGraph(object):
    def __init__(self, test_tag_id, num_tags=587):
        #### tag and waypoints recording specific parameters ####
        self.num_tags = num_tags
        self.origin_tag = None  # First tag seen
        self.origin_tag_pose = None
        self.supplement_tags = {}
        self.waypoints = {}
        self.distance_traveled = [0, 0, 0]

        #### vertices, edges ####
        self.odometry_vertices = OrderedDict()
        self.odometry_edges = OrderedDict()
        self.tag_vertices = OrderedDict()
        self.odometry_tag_edges = OrderedDict()
        self.waypoints_vertices = OrderedDict()
        self.odometry_waypoints_edges = OrderedDict()

        #### graph ####
        self.graph = {}
        self.visited_nodes = deque([])

        #### g2o parameters ####
        self.package = RosPack().get_path('navigation_prototypes')
        self.g2o_data = None
        self.g2o_data_path = path.join(self.package, 'data/data_g2o/data.g2o')  # path to compiled g2o file
        self.g2o_data_copy_path = path.join(self.package, 'data/data_g2o/data_cp.g2o')  # copy of the unedit data
        self.g2o_result_path = path.join(self.package, 'data/data_g2o//result.g2o')

        #### test data ####
        self.test_tag_id = test_tag_id
        self.test_data_tag = {}
        self.test_data_path = {}
        self.g2o_test_data = None
        self.testfile = path.join(self.package, 'data/data_g2o/naive.txt')

    def add_odometry_vertices(self, id, trans, rot, fix_status):
        self.odometry_vertices[id] = Vertex(id, trans, rot, "odometry", fix_status)
        return self.odometry_vertices[id]

    def add_odometry_edges(self, v_start, v_end, trans, rot, damping_status):
        self.odometry_edges[v_start.id] = Edge(v_start, v_end, trans, rot, damping_status)
        return self.odometry_edges[v_start.id]

    def add_tag_vertices(self, id, trans, rot, transformed_pose):
        if self.origin_tag is None:
            self.origin_tag = id
            self.origin_tag_pose = transformed_pose  # make this tag the origin tag
            self.tag_vertices[id] = Vertex(id, trans, rot, "tag", True)
            print "AR_CALIBRATION: Origin Tag Found: " + str(id)
        elif not (id == self.origin_tag or id in self.supplement_tags.keys()):
            self.supplement_tags[id] = transformed_pose  # set new supplemental AR Tag
            self.tag_vertices[id] = Vertex(id, trans, rot, "tag", False)
            print "AR_CALIBRATION: Supplementary Tag Found: " + str(id)
            print(self.supplement_tags.keys())
        elif id == self.origin_tag:
            self.origin_tag_pose = transformed_pose  # Reset the origin tag
            print "AR_CALIBRATION: Origin Tag Refound: " + str(id)
        else:
            print "AR_CALIBRATION: Found Old Tag: " + str(id)

    def add_odometry_tag_edges(self, v_odom, v_tag, trans, rot):
        if v_tag.id not in self.odometry_tag_edges.keys():
            self.odometry_tag_edges[v_tag.id] = {}
        self.odometry_tag_edges[v_tag.id][v_odom.id] = Edge(v_odom, v_tag, trans, rot)
        return self.odometry_tag_edges[v_tag.id][v_odom.id]

    def add_waypoint_vertices(self, id, curr_pose):
        if id not in self.waypoints_vertices.keys():
            self.waypoints[id] = curr_pose  # store the pose of waypoint
            self.waypoints_vertices[id] = Vertex(id, curr_pose.translation, curr_pose.rotation, "waypoint")
            print "AR_CALIBRATION: Waypoint Found: " + str(id)
            print(self.waypoints.keys())
            return self.waypoints_vertices[id]
        else:
            print "AR_CALIBRATION: Found Old Waypoint: " + str(id)

    def add_odometry_waypoint_edges(self, v_odom, v_waypoints):
        if v_waypoints not in self.odometry_waypoints_edges.keys():
            self.odometry_waypoints_edges[v_waypoints.id] = {}
        self.odometry_waypoints_edges[v_waypoints.id][v_odom.id] = Edge(v_odom, v_waypoints, [0, 0, 0], [0, 0, 0, 1])
        return self.odometry_waypoints_edges[v_waypoints.id]

    def add_damping(self, curr_pose):
        """
        Add a vertex and edge for correcting damping
        :param curr_pose: Vertex object for current pose
        """
        damping_vertex = self.add_odometry_vertices(curr_pose.id + 1, [0, 0, 0], curr_pose.rotation, True)
        damping_edge = self.add_odometry_edges(curr_pose, damping_vertex, [0, 0, 0], [0, 0, 0, 1], True)
        # compute importance matrix
        damping_edge.compute_importance_matrix()
        if damping_edge.check_importance_matrix_PSD():
            damping_edge.eigenvalue_PSD = True

    def add_pose_to_pose(self, curr_pose, trans, rot, importance):
        """
        Add an edge between vertices of current pose and last pose
        :param curr_pose: Vertex object of current pose
        :param trans: translation
        :param rot: rotation
        :param importance: Importance of this new edge
        """
        pose_edge = self.add_odometry_edges(self.odometry_vertices[curr_pose.id - 1], curr_pose, trans, rot, False)
        pose_edge.odometry_importance = importance
        # compute importance matrix
        pose_edge.compute_importance_matrix()
        if pose_edge.check_importance_matrix_PSD():
            pose_edge.eigenvalue_PSD = True

    def add_pose_to_tag(self, curr_pose, tag_id, trans, rot):
        """
        Add an edge between vertices of current pose and current tag detected
        :param curr_pose: Vertex object of current pose
        :param tag_id: id current tag detected
        :param trans: translation
        :param rot: rotation
        """
        if tag_id in self.tag_vertices.keys():
            tag = self.tag_vertices[tag_id]
            pose_tag_edge = self.add_odometry_tag_edges(curr_pose, tag, trans, rot)
            # compute importance matrix
            pose_tag_edge.compute_importance_matrix()
            if pose_tag_edge.check_importance_matrix_PSD():
                pose_tag_edge.eigenvalue_PSD = True
            return True
        else:
            return False

    def add_pose_to_waypoint(self, curr_pose, waypoint_id):
        if waypoint_id in self.waypoints_vertices.keys():
            waypoint = self.waypoints_vertices[waypoint_id]
            pose_waypoint_edge = self.add_odometry_waypoint_edges(curr_pose, waypoint)
            pose_waypoint_edge.compute_importance_matrix()
            if pose_waypoint_edge.check_importance_matrix_PSD():
                pose_waypoint_edge.eigenvalue_PSD = True
            return True
        else:
            return False

    def add_test_data_tag(self, id, trans, rot):
        self.test_data_tag[id] = trans + rot

    def add_test_data_path(self, id, trans, rot):
        self.test_data_path[id] = trans + rot

    def initialize_nodes(self, vertices):
        for vertex in vertices.keys():
            self.graph[vertex] = []

    def add_nodes_to_graph(self, source_node, neighbours):
        for neighbour in neighbours:
            self.graph[source_node].append(neighbour)
            self.graph[neighbour].append(source_node)

    def construct_graph(self):
        self.graph = {}
        for vertices in [self.odometry_vertices, self.tag_vertices, self.waypoints_vertices]:
            self.initialize_nodes(vertices)
        # print self.graph

        for edge in self.odometry_edges.values():
            start_node, end_node = edge.start.id, edge.end.id
            if np.sum(edge.importance_matrix.diagonal()) != 0:  # check if the importance value is zero
                self.add_nodes_to_graph(start_node, [end_node])
            else:
                print "Discard Unconnected Edge"
        print "Odometry Nodes Added"

        for tag in self.odometry_tag_edges.keys():
            start_node, neighbour_node = tag, self.odometry_tag_edges[tag].keys()
            # start_node, neighbour_node = tag, []
            # for odom in self.odometry_tag_edges[tag].keys():
            #     neighbour_node.append(odom.id)
            self.add_nodes_to_graph(start_node, neighbour_node)
        print "Tag Nodes Added"

        for waypoint in self.odometry_waypoints_edges.keys():
            start_node, neighbour_node = waypoint, self.odometry_waypoints_edges[waypoint].keys()
            print self.odometry_waypoints_edges[waypoint]
            # start_node, neighbour_node = waypoint, []
            # for odom in self.odometry_waypoints_edges[waypoint].keys():
            #     neighbour_node.append(odom.id)
            self.add_nodes_to_graph(start_node, neighbour_node)

    def bfs(self, source):
        self.visited_nodes = deque([])
        self.visited_nodes.append(source)
        node_queue = deque([])
        node_queue.append(source)
        while node_queue:
            node = node_queue.popleft()
            for neighbour in self.graph[node]:
                if neighbour not in self.visited_nodes:
                    node_queue.append(neighbour)
                    self.visited_nodes.append(neighbour)

    def remove_unconnected_portion(self, vertices, edges):
        for vertex_id in vertices.keys():
            if vertex_id not in self.visited_nodes:
                print "DELETE VERTEX:", vertex_id
                del vertices[vertex_id]
                if vertex_id in edges.keys():
                    del edges[vertex_id]

    def update_posegraph(self):
        self.remove_unconnected_portion(self.odometry_vertices, self.odometry_edges)
        self.remove_unconnected_portion(self.tag_vertices, self.odometry_tag_edges)
        self.remove_unconnected_portion(self.waypoints_vertices, self.odometry_waypoints_edges)

    def process_graph(self):
        self.construct_graph()
        self.bfs(self.origin_tag)
        self.update_posegraph()
        print("DATA PROCESSED")

    def write_g2o_vertices(self, *vertices_list):
        for vertices in vertices_list:
            for id in vertices.keys():
                data = vertices[id].write_to_g2o()
                self.g2o_data.write(data)

    def write_g2o_edges_pose(self):
        for edge_id in self.odometry_edges.keys():
            data = self.odometry_edges[edge_id].write_g2o_edges()
            importance = self.odometry_edges[edge_id].write_to_g2o_importance()
            self.g2o_data.write(data + " " + importance)

    def write_g2o_edges_landmark(self, *landmark_edges_list):
        """
        Write edges data to g2o for landmark such as tags and waypoints.
        :param landmark_edges_list: dictionary of edges with the keys being the name of the landmark
        :return: NA
        """
        for edges in landmark_edges_list:
            for landmark_id in edges.keys():
                for landmark_odom in edges[landmark_id].keys():
                    data = edges[landmark_id][landmark_odom].write_g2o_edges()
                    importance = edges[landmark_id][landmark_odom].write_g2o_importance()
                    self.g2o_data.write(data + " " + importance)

    def write_g2o_data(self):
        """
        Write to g2o data file
        """
        self.g2o_data = open(self.g2o_data_path, 'wb+')
        self.write_g2o_vertices(self.odometry_vertices, self.tag_vertices, self.waypoints_vertices)
        self.write_g2o_edges_pose()
        self.write_g2o_edges_landmark(self.odometry_tag_edges, self.odometry_waypoints_edges)
        print "g2o data has wrote to file"

    def write_g2o_test_data(self):
        """
        Write to g2o test data
        """
        self.g2o_test_data = open(self.testfile, 'wb+')
        for tags in self.test_data_tag.values():
            self.g2o_test_data.write("TAG %f %f %f %f %f %f %f\n" % tuple(tags))
        for paths in self.test_data_path.values():
            self.g2o_test_data.write("PATH %f %f %f %f %f %f %f\n" % tuple(paths))
        print "test data has wrote to file"

    def optimize_pose(self):
        """
        Run g2o
        """
        self.write_g2o_data()
        self.write_g2o_test_data()
        self.g2o_data.close()
        self.g2o_test_data.close()
        system("g2o -o %s %s" % (self.g2o_result_path, self.g2o_data_path))  # Run G2o
        system("cp %s %s" % (self.g2o_data_path, self.g2o_data_copy_path))  # Copy Original Data
        print "OPTIMIZATION COMPLETED"

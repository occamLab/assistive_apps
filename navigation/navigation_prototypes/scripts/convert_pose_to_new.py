from pose_graph import PoseGraph
from posegraph import Vertex, Edge, Graph, VertexType


def convert(posegraph):
    vertices = {}
    for startid in posegraph.odometry_edges:
        for endid in posegraph.odometry_edges[startid]:
            edge = posegraph.odometry_edges[startid][endid]
            endpoints = [edge.start, edge.end]
            for vertex in endpoints:
                if vertex.type == 'tag':
            # start = edge.start
            # end = edge.end
            # vertex = Vertex()

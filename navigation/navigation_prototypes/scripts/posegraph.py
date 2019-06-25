class VertexType:
    ODOMETRY = 0
    TAG = 1
    DUMMY = 2


class Vertex:
    def __init__(self, mode, uid, value):
        self.mode = mode
        self.uid = uid
        self.value = value


class Edge:
    def __init__(self, startuid, enduid, importance):
        self.startuid = startuid
        self.enduid = enduid
        self.importance = importance


class Graph:
    def __init__(self, vertices, edges):
        self.edges = edges
        self.vertices = vertices

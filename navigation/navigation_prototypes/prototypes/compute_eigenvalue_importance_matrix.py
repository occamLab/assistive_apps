import numpy as np
from os import path
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply


def null(a, rtol=1e-5):
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol * s[0]).sum()
    return rank, v[rank:].T.copy()


def compute_basis_vector(rotation):
    # for step in np.arange(0, 3*np.pi, .05):
    #print "rot", rotation
    q2 = quaternion_from_euler(0, 0,
                               .05)  # for generating a rotation matrix to rotate a small amount around the z axis
    qsecondrotation = quaternion_multiply(q2, rotation)
    change = (qsecondrotation[0:3] - rotation[0:3])
    change = change / np.linalg.norm(
        change)  # Determine which direction is the yaw direction and then make sure that direction is diminished in the information matrix
    v = change / np.linalg.norm(change)
    _, u = null(v[np.newaxis])
    basis = np.hstack((v[np.newaxis].T, u))
    # place high information content on pitch and roll and low on changes in yaw
    I = basis.dot(np.diag([.001, 1000, 1000])).dot(basis.T)
    return I

def compute_eigenvalue(rotation):
    tri = np.zeros((6, 6))
    I = compute_basis_vector(rotation)
    indeces = np.triu_indices(3)
    #print "merged:", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + I[indeces].tolist()
    tri[np.triu_indices(6, 0)] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + I[indeces].tolist()
    tri_updated = tri + np.tril(tri.T, -1)
    tri_regularized = tri_updated + np.identity(6) * 1
    value = np.linalg.eigvals(tri_updated)
    for v in value.tolist():
        #print "v", type(v)
        if v.real < 0:
            print "WARNING: NEGATIVE!"
    print "value:", value
    return value


def gather_rotation(data_path):
    rotations = {}
    with open(data_path, 'rb') as g2o_data:
        for line in g2o_data:
            if line.startswith("VERTEX_SE3:QUAT "):
                contents = line.split()
                newline = []
                for data in contents[1:]:
                    newline.append(float(data))
                if newline[0] > 587 and newline[0] % 2 == 0:
                    rotations[int(newline[0])] = newline[4:8]
                    #print("record rotation for vertex: " + str(newline[0]))
    return rotations

def run(data_path):
    eigenvalues = {}
    rotations = gather_rotation(data_path)
    for vertex_id in rotations.keys():
        eigenvalues[vertex_id] = compute_eigenvalue(rotations[vertex_id])

if __name__ == "__main__":
    g2o_data = path.expanduser(
        '~') + '/catkin_ws/src/assistive_apps/navigation/navigation_prototypes/prototypes/data_g2o/result.g2o'
    run(g2o_data)

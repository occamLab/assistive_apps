@staticmethod
def rotation_diff_q2e(rot1, rot2):
    rot1euler = euler_from_quaternion(rot1)
    rot2euler = euler_from_quaternion(rot2)
    return [rot1euler[i] - rot2euler[i] for i in range(3)]


def compare_tag_odom_edges(self):
    for tag_id in self.posegraph:
        odom_ids = self.posegraph.odometry_tag_edges[tag_id].keys().sort()
        counter = 0
        num_odom = len(odom_ids)
        while counter < num_odom:
            odom1 = odom_ids[counter]
            for n in range(1, num_odom - counter, 1):
                odom2 = odom_ids[counter + n]
                if self.check_odom_position_closeness(odom1, odom2):
                    transdiff, rotdiff = self.compute_tag_odom_edges_diff(tag_id, odom1, odom2)
            counter += 1


def check_odom_position_closeness(self, odom1_id, odom2_id):
    trans_diff_threshold = 0.5  # 0.5m
    trans1 = self.posegraph.odometry_vertices[odom1_id].translation
    trans2 = self.posegraph.odometry_vertices[odom2_id].translation
    trans_diff = [trans1[i] - trans2[i] for i in range(3)]
    trans_diff_mag = math.sqrt(sum(i ** 2 for i in trans_diff))
    if trans_diff_mag > trans_diff_threshold:
        return True
    else:
        return False


def compute_tag_odom_edges_diff(self, tagid, odom1_id, odom2_id):
    trans1 = self.posegraph.odometry_tag_edges[tagid][odom1_id].translation
    rot1 = self.posegraph.odometry_tag_edges[tagid][odom1_id].rotation
    trans2 = self.posegraph.odometry_tag_edges[tagid][odom2_id].translation
    rot2 = self.posegraph.odometry_tag_edges[tagid][odom2_id].rotation
    trans_diff = [trans1[i] - trans2[i] for i in range(3)]
    trans_diff_mag = math.sqrt(sum(i ** 2 for i in trans_diff))
    rot_diff = Optimization.rotation_diff_q2e(rot1, rot2)
    return trans_diff_mag, rot_diff
"""
Converts Important Facets of the Pose Graph to JSON file for upload to Firebase and use on iOS.

by Daniel Connolly, 2018

Last Modified August, 2018

This script will:
- Read in a pickle file from data/optimized_data folder. The file contains a pose graph constructed from running
  data_collection.py
- Convert several aspects of the pose graph to a Json file and uploads that file to Firebase.

Dependencies:
pyrebase - to install, run: sudo pip install pyrebase

To use:
- Open Terminal and run the code below:

python pkl_json.py

"""

import pickle
import json
from collections import OrderedDict
import pyrebase

class PickleJson:

    def __init__(self, filename, firebase_filename):
        self.config = {
          "apiKey": " AIzaSyC-obgg0mihoW3oaVWlv3bQqE05CSa2Pxg",
          "authDomain": "invisible-map.firebaseapp.com",
          "databaseURL": "https://invisible-map.firebaseio.com",
          "storageBucket": "invisible-map.appspot.com",
          "serviceAccount": "my_firebase_key.json"
        }

        self.firebase = pyrebase.initialize_app(self.config)
        self.db = self.firebase.database()
        self.storage = self.firebase.storage()

        f = open('../data/optimized_data/data_optimized.pkl')
        self.pose_graph = pickle.load(f)
        self.data = {}
        self.filename = filename
        self.firebase_filename = firebase_filename

    def create_json_file(self):
        self.data["waypoints_vertices"] = []
        for vertex in self.pose_graph.waypoints_vertices:
            self.data['waypoints_vertices'].append({'id': self.pose_graph.waypoints_vertices[vertex].id, 'translation': {'x': self.pose_graph.waypoints_vertices[vertex].translation[0], 'y': self.pose_graph.waypoints_vertices[vertex].translation[1], 'z': self.pose_graph.waypoints_vertices[vertex].translation[2]}, 'rotation': {'x': self.pose_graph.waypoints_vertices[vertex].rotation[0], 'y': self.pose_graph.waypoints_vertices[vertex].rotation[1], 'z': self.pose_graph.waypoints_vertices[vertex].rotation[2], 'w': self.pose_graph.waypoints_vertices[vertex].rotation[3]}})

        self.data["odometry_vertices"] = []
        for vertex in self.pose_graph.odometry_vertices:
            self.data['odometry_vertices'].append({'id': self.pose_graph.odometry_vertices[vertex].id, 'translation': {'x': self.pose_graph.odometry_vertices[vertex].translation[0], 'y': self.pose_graph.odometry_vertices[vertex].translation[1], 'z': self.pose_graph.odometry_vertices[vertex].translation[2]}, 'rotation': {'x': self.pose_graph.odometry_vertices[vertex].rotation[0], 'y': self.pose_graph.odometry_vertices[vertex].rotation[1], 'z': self.pose_graph.odometry_vertices[vertex].rotation[2], 'w': self.pose_graph.odometry_vertices[vertex].rotation[3]}})

        self.data['tag_vertices'] = []
        for vertex in self.pose_graph.tag_vertices:
            self.data['tag_vertices'].append({'id': self.pose_graph.tag_vertices[vertex].id, 'translation': {'x': self.pose_graph.tag_vertices[vertex].translation[0], 'y': self.pose_graph.tag_vertices[vertex].translation[1], 'z': self.pose_graph.tag_vertices[vertex].translation[2]}, 'rotation': {'x': self.pose_graph.tag_vertices[vertex].rotation[0], 'y': self.pose_graph.tag_vertices[vertex].rotation[1], 'z': self.pose_graph.tag_vertices[vertex].rotation[2], 'w': self.pose_graph.tag_vertices[vertex].rotation[3]}})

        with open(self.filename, 'w') as f:
            json.dump(self.data, f)

    def push_to_firebase(self):
        self.storage.child(self.firebase_filename).put(self.filename)

if __name__ == "__main__":
    converter = PickleJson('data.json', 'milas_hall.json')
    converter.create_json_file()
    converter.push_to_firebase()

#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler


class GlobalPathCreator:
    def __init__(self):
        self.global_path_topic = "/global_path"
        self.global_path_csv = rospy.get_param("~global_path_csv", "global_path.csv")

        self.global_path = Path()
        self.global_path.header.frame_id = "/map"
        self.global_path.header.stamp = rospy.Time.now()

        # create path publisher
        self.path_pub = rospy.Publisher(self.global_path_topic, Path, queue_size=1)

        self.create_path()

        while not rospy.is_shutdown():
            self.path_pub.publish(self.global_path)

    def create_path(self):
        waypoints = self.get_waypoints(self.global_path_csv)
        self.prepare_waypoint_poses(waypoints)

    def get_waypoints(self, waypoints_csv):
        """
        Establishes waypoints from csv of sensor readings
        ----------------
        Params: csv_file
        csv file of data
        ----------------
        Returns: waypoints filtered from csv_file as a dataframe
        """

        data = pd.read_csv(waypoints_csv)
        data = data[280:-180]

        # get waypoints every 15 datapts
        data = data.iloc[::15, :]

        # add the starting point to end to form loop
        path_15 = data.append(data.iloc[0], ignore_index=True)

        return path_15

    def make_pose(self, row):
        """
        Makes Pose() with position and orientation
        ----------------
        Params: row
        odom reading at a single point in time
        ----------------
        Returns: Pose() object
        """
        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = row.x
        pose.pose.position.y = row.y
        pose.pose.position.z = row.z
        pose.pose.orientation.x = row.qx
        pose.pose.orientation.y = row.qy
        pose.pose.orientation.z = row.qz
        pose.pose.orientation.w = row.qw

        return pose

    def prepare_waypoint_poses(self, waypoints: pd.DataFrame):
        """
        Makes PoseArray() with waypoint position & orientation
        ----------------
        Params: waypoints
        dataset of waypoints
        ----------------
        Returns: None
        """

        for i in range(len(waypoints)):
            pose = self.make_pose(waypoints.iloc[i])
            self.global_path.poses.append( pose )


class SVGPathCreator:
    def __init__(self):
        self.global_path_topic = "/global_path"
        global_path_svg = rospy.get_param("~global_path_svg", "global_path_svg.txt")

        # create path publisher
        self.path_pub = rospy.Publisher(self.global_path_topic, Path, queue_size=1)

        parsed_points = self.parse_svg(global_path_svg)
        global_path = self.create_path(parsed_points)

        while not rospy.is_shutdown():
            self.path_pub.publish(global_path)

    def parse_svg(self, svg_path: str):
        with open(svg_path) as svg:
            path = svg.readline()
            path = path.replace("M", " ")
            path = path.replace("C", " ")
            path = path.replace("Z", " ")

            path_list = path.split(" ")
            path_list = path_list[1: -1]

            path_array = np.array(path_list, dtype=float)
            path_array = path_array.reshape((int(path_array.shape[0]/2), 2))
            path_array = np.flip(path_array)
        return path_array

    def create_path(self, points):
        global_path = Path()
        global_path.header.frame_id = "/map"
        global_path.header.stamp = rospy.Time.now()

        for i, point in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = "/map"
            pose.header.stamp = rospy.Time.now()

            # 0.25 inches per pixel * 0.0254 meters per inch
            pixel_to_meters = 0.25 * 0.0254
            pose.pose.position.x = point[0] * pixel_to_meters
            pose.pose.position.y = point[1] * pixel_to_meters
            pose.pose.position.z = 0

            if i + 1 == len(points):
                q = self.get_orientation(point, points[0])
            else:
                q = self.get_orientation(point, points[i+1])

            # set orientation of pose
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            global_path.poses.append(pose)

        return global_path

    def get_orientation(self, point_1, point_2):
        angle = math.atan2(point_2[1] - point_1[1], point_2[0] - point_1[0])
        q = quaternion_from_euler(0, 0, angle)
        return q



if __name__ == "__main__":
    try:
        rospy.init_node("global_path_creator", anonymous=True)
        GlobalPathCreator()
        # SVGPathCreator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

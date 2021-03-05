#!/usr/bin/env python 

# Preparation of waypoints as PoseArray(). 
# Test by: 
# rospy.init_node('testing')
# waypoints = get_waypoints('saved_waypoints.csv')
# waypoint_poses = prepare_waypoint_poses(waypoints)
# rospy.loginfo("Published {} waypoints.".format(len(waypoint_poses.poses))) 

import pandas as pd
import numpy as np
import math 
import os, rospkg

from geometry_msgs.msg import (
        Pose, 
        PoseStamped,
        PoseArray,
        Point,
        Quaternion
    )

# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
reload(gc)
# Import AlvinXY transformation module
import alvinxy.alvinxy as axy
reload(axy)
import rospy
import tf
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry


def get_waypoints(csv_file):
    """
    Establishes waypoints fron list of odom readings
    ----------------
    Params: csv_file
    csv file of data
    ----------------
    Returns: waypoints filtered from csv_file as a dataframe
    """
    rp = rospkg.RosPack()
    data_path = os.path.join(rp.get_path("waypoints"), "data", csv_file)

    data = pd.read_csv(data_path)
    data = data[280:-180]
    # get waypoints every 15 datapts
    data = data.iloc[::15, :]
    # add the starting point to end to form loop
    path_15 = data.append(data.iloc[0], ignore_index=True)    
    return path_15


def make_pose(row):
    """
    Makes Pose() with position and orientation
    ----------------
    Params: row
    odom reading
    ----------------
    Returns: pose()
    """
    pose = PoseStamped()
    pose.pose.position.x = row.x
    pose.pose.position.y = row.y
    pose.pose.position.z = row.z
    pose.pose.orientation.x = row.qx
    pose.pose.orientation.y = row.qy
    pose.pose.orientation.z = row.qz
    pose.pose.orientation.w = row.qw

    return pose

def prepare_waypoint_poses(waypoints):
    """
    Makes Path() with waypoint position & orientation as Poses
    ----------------
    Params: waypoints
    dataset of waypoints
    ----------------
    Returns: Path() of waypoint poses
    """
    path = Path()
    # map or base_link?
    path.header.frame_id = "/map"
    path.header.stamp = rospy.Time.now()

    for i in range(len(waypoints)):
        pose = make_pose(waypoints.iloc[i])            
        path.poses.append( pose )

    return path

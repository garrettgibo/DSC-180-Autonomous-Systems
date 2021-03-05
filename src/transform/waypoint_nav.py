#!/usr/bin/env python 

# Preparation of waypoints as PoseArray(). 
# Test waypoints path: 
# rospy.init_node('testing')
# waypoints = get_waypoints('saved_waypoints.csv')
# waypoint_poses = prepare_waypoint_poses(waypoints)
# rospy.loginfo("Published {} waypoints.".format(len(waypoint_poses.poses))) 
# Test waypoint navigation:
# rospy.init_node('testing')
# waypoints = get_waypoints('saved_waypoints.csv')
# waypoint_poses = prepare_waypoint_poses(waypoints)

navigate_to_waypoint_x(waypoint_poses.poses[1].pose.position, 0, waypoint_poses)
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

def euclidean_dist(x1, x2, y1, y2):
    """
    Calculates Euclidean distance.
    -------------
    Params: xy coordinates
    -------------
    Returns: distance 
    """
    return np.sqrt(np.square(x1 - x2) + np.square(y1 - y2)) 

def get_steering(curr_pos, waypoint, curr_heading):
    """
    Calculates steering angle to point 
    towards next waypoint
    -------------
    Params: 
    curr_pos: current vehicle position in x, y 
    waypoint: upcoming waypoint pose()
    curr_heading: current vehicle heading in (degrees/radians?)
    -------------
    Returns:
    Steering angle 
    """
    # angle between waypoint and current position in radians 
    desired_heading = math.atan((waypoint.pose.position.y - curr_pos.pose.position.y)/(waypoint.pose.position.x - curr_pos.pose.position.x))

    if curr_heading != desired_heading:
        #rotate steering_angle degrees
        print("placeholder")

def navigate_to_waypoint_x(curr_pos, curr_heading, path):
    """
    Handles waypoint selection and waypoint navigation
    -------------
    Params:
    curr_pos: current vehicle position as point object
    curr_heading: current vehicle heading in (degrees/radians?)
    path: waypoints as Path()
    """
    amt_waypoints = len(path.poses)
    endpt = path.poses[-1]

    # waypt radius buffer in ?? units
    # choose CEP? but make sure waypoints are far away enough from 
    # each other to be greather than CEP
    buffer_radius = 2

    # maximum rotation angle
    max_rotation = 0    
    
    for i in range(amt_waypoints):
        print(i)
        curr_waypt = path.poses[i]
        # Calculate the distance between the current waypoint and endpoint. 
        dist_waypt_endpt = euclidean_dist(endpt.pose.position.x, curr_waypt.pose.position.x, endpt.pose.position.y, curr_waypt.pose.position.y)
        # Calculate the distance between the endpoint and current vehicle position. 
        dist_waypt_pt = euclidean_dist(endpt.pose.position.x, curr_pos.x, endpt.pose.position.y, curr_pos.y)

        print(dist_waypt_endpt, dist_waypt_pt)
        # continue towards curr_waypt
        if dist_waypt_endpt < dist_waypt_pt:
            print(" heading towards curr waypoint ")
            # send steering angle

            # send throttle value

            # continuously check if current_pos is within buffer radius
            # should be getting new curr_pos updates here
            dist_waypt_pt = euclidean_dist(endpt.pose.position.x, curr_pos.x, endpt.pose.position.y, curr_pos.y)

            if dist_waypt_pt < buffer_radius:
                print("within buffer radius, head towards next one")
                continue
            else:
                print("Curr_waypoint already reached, heading to next one")

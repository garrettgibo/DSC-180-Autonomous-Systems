# Preparation of waypoints as PoseArray(). 
# Test by: 
# rospy.init_node('testing')
# waypoints = get_waypoints('saved_waypoints.csv')
# waypoint_poses = prepare_waypoint_poses(waypoints)


def get_waypoints(csv_file):
    """
    Establishes waypoints from csv of sensor readings
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
    odom reading at a single point in time
    ----------------
    Returns: Pose() object
    """
    pose = Pose()
    pose.position.x = row.x
    pose.position.y = row.y
    pose.position.z = row.z
    pose.orientation.x = row.qx
    pose.orientation.y = row.qy
    pose.orientation.z = row.qz
    pose.orientation.w = row.qw

    return pose

def prepare_waypoint_poses(waypoints):
    """
    Makes PoseArray() with waypoint position & orientation
    ----------------
    Params: waypoints
    dataset of waypoints
    ----------------
    Returns: poseArray() of waypoints
    """
    path = PoseArray()
    # map or base_link?
    path.header.frame_id = "/map"
    path.header.stamp = rospy.Time.now()

    for i in range(len(waypoints)):
        pose = make_pose(waypoints.iloc[i])            
        path.poses.append( pose )

    return path


string name # Waypoint name, referenced in the map definition dict.

bool has_angle            # say whether the 'theta' attribute in pose is important
string frame_id           # Position tf frame
geometry_msgs/Pose2D pose # X, Y, THETA values corrsponding to the Waypoint name.
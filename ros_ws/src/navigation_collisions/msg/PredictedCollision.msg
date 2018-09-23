# Collision context
uint8 LEVEL_STOP      = 0  # Collision will happen very soon, stop the robot immediately!
uint8 LEVEL_DANGER    = 1  # Path needs to be modified (while driving is fine) or a collision will happen.
uint8 LEVEL_POTENTIAL = 2  # A dynamic object will probably pass through at some time, maybe not (replan optional).
uint8 danger_level

# float32 distance_left  # Gives how far away the collision is (supposing the robot will follow the current path).
# float32 time_left      # Gives how much time's left before hitting the obstacle (predicted based on the current speed).

# Collision obstacle description
uint8 TYPE_POINT  = 0
uint8 TYPE_RECT   = 1
uint8 TYPE_CIRCLE = 2
uint8 obstacle_type

geometry_msgs/Pose2D obstacle_pos # For all obstacle types.

float32 obstacle_width   # For Rects.
float32 obstacle_height  # For Rects.
float32 obstacle_radius  # For Circles.
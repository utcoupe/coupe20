#More precise MapObject (with wheelbase for example)

string name

geometry_msgs/Pose2D pose

# Shape
uint8 SHAPE_RECT   = 0
uint8 SHAPE_CIRCLE = 1
uint8 SHAPE_POINT  = 2
uint8 shape_type

float32 width
float32 height

float32 radius

float32 wheelbase

# Metadata
string[] tags
string color
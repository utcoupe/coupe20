# belt_interpreter

This node listen for sensor data from the belt, converts the ranges into rectangles taking in account the cone angle of the sensors and the range precision. It then splits the rectangles into two groups : those that belong to the map, and those that are unknown objects and may be dangerous.

- Listens `/drivers/ard_others/belt_ranges`
- Publishes on `/processing/belt_interpreter/rects_filtered`
- Fetches definition in `processing/belt.xml`

## Initialization

At the start of the node, it fetches all map objects from its layer (`/terrain/walls/layer_belt/*`) thanks to the `memory_map` package.If it fails doing that, the node shuts down.

It is also responsible for publishing the static transforms, one for each sensor, relative to the `/robot` transform. The transforms are computed from each sensor position and angle relative to the robot, all defined in the `belt.xml` definition file that get fetched thanks to the `memory_definition` package.

## Processing the data
When the node receive data from the `drivers_ard_other` packages, it adds it to a stack.

It then processes all data on the stack at a fixed rate.
If the range is greater than the max range defined in the definition file, the sensor get skipped.

The rectangle's time stamp is the lastest common time between the `/map` tranform and the static transform of the sensor.
If this timestamp is too old (greater than `TF_TIME_THRESH` seconds), a warning is printed.

The width and height of the rectangle are calculated base on the range, the range precision and the cone angle.

Once we have a rectangle, to check if it is unknown or not, we check how much of it intersects elements of the map. If more than a certain percentage (`POINTS_PC_THRESHOLD`) of the rectangle overlaps map objects, it is considered static.

To check this, the node turns the rectangle into a discrete number of points (with the `RESOLUTION_LONG` and `RESOLUTION_LONG` constants). However, there is a maximum number of points a rectangle can be turned into, defined with the constant `PRECISION_RANGE_THRESH`. If the range is greater than this constant, the rectangle will be turned into N points, where N is the number of points that would divide a rectangle at the `PRECISION_RANGE_THRESH` range. Thanks to this, higher ranges don't take too much time to compute.

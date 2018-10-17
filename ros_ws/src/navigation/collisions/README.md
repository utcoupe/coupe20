Inputs :

- robot's current position and speed *from drivers/ard_asserv or navigation/navigator*
- robot's state (driving, stopped, etc) and current path (remaining waypoints to pass through *from memory/map or navigation/navigator*
- belt dangerous objects and static objects *from processing/qmsk*
- enemies positions *from recognition/enemy_finder*

Outputs :

- publishes on a topic when a collision is predicted. navigator will subscribe to it and stop the robot when this message is published.

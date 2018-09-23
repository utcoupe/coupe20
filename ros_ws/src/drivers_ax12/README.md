# drivers_ax12

This package mainly provides an action server to move an AX-12A to an angle, or to switch it in wheel mode. Both modes need to specify a speed in the action goal.
It comes also with a service to set specific registers of an AX-12A, such as the punch, etc. It uses the Dynamixel SDK 3 ros package.
# Topic to move an actuator. Specify the type, actuator id and desired position.

int16 order_nb # Optional : you can enter an order nb, arduino will reply through
               # The topic /move_responses with your nb and whether the move succeeded. 

uint8 TYPE_DIGITAL = 0 # On/Off devices
uint8 TYPE_PWM     = 1 # PWM regulated devices (DC motors...)
uint8 TYPE_SERVO   = 2 # Real servos (drived with Servo.h in arduino)
uint8 TYPE_STEPPER = 3 # Stepper motors which take a number of steps
uint8 type       # Actuator type.
uint8 id         # Actuator ID number defined in the arduino code before flashing.

int16 dest_value # Can be a position, speed, state...
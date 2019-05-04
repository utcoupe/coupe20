//delay for presure propagation
#define PNEU_DELAY 150 // in ms TODO:OPTIMIZE THIS
#define PUCK_TIME_TO_MOVE 600 // in ms TODO:OPTIMIZE THIS
#define PUCKS_DOOR_TIME_TO_MOVE 1000// in ms TODO:OPTIMIZE THIS
// ELECTROVANNES Ventouse pins
#define PUMP 1
#define TEST1 3
#define VENT1_TO_PUMP 2
#define VENT2_TO_PUMP 3
#define VENT3_TO_PUMP 4
#define VENT1_TO_AIR 5
#define VENT2_TO_AIR 6
#define VENT3_TO_AIR 7


//messages to ROS
#define AWAITING_ORDER_MSG 1
#define PUCKS_SUCKED_UP_MSG 2
#define PUCKS_SORTED_MSG 3
#define PUCKS_DOOR_RESET_MSG 4
#define TOWER_OPEN 5

//SELECTOR_PWM for pucks sorting
#define SELECTOR_PWM 8
#define SELECTOR_SCALE_POS 128 //TODO: set this
#define SELECTOR_PILE_POS 128 //TODO: set this
#define SELECTOR_TIME_TO_MOVE 1000 // in ms TODO:OPTIMIZE THIS
//atom tower for PR's pickup
#define TOWER_TOP_DELAY 1000
#define TOWER_TOP_PWM 9
#define TOWER_TOP_OPEN_POS 120
#define TOWER_TOP_CLOSED_POS 100
#define TOWER_CLOSURE_DELAY 5000
#define TOWER_CLOSURE_PWM 10
#define TOWER_CLOSURE_OPEN_POS 128
#define TOWER_CLOSURE_CLOSED_POS 128
//atom tower for PR's pickup
#define PUCKS_DOOR_PWM 11

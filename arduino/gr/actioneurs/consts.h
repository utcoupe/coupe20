//pins
#define PUMP_1 2
#define PUMP_2 3
#define PUMP_3 4
#define PUMP_ENABLE 5

#define TOWER_PIN 8
#define TOWER_LIMIT_PIN A4

#define SCALE_DOOR_PIN 7
#define SCALE_DOOR_LIMIT_PIN A6

#define PUCKS_DOOR_DIR_PIN 9
#define PUCKS_DOOR_STEP_PIN 10
#define PUCKS_DOOR_EN_PIN 11 

#define SELECTOR_PIN 13

#define PUCKS_DOOR_UP_SWITCH_PIN 14

//delays
#define PUCKS_DOOR_MIN_DELAY 1000 //stepper
#define PUMP_DELAY 150 
#define PUCK_TIME_TO_MOVE 1500 // in ms TODO:OPTIMIZE THIS
#define SELECTOR_TIME_TO_MOVE 200 // in ms TODO:OPTIMIZE THIS

//Calibrations
#define SELECTOR_TO_TOWER 125
#define SELECTOR_TO_SCALE 45
#define PUCKS_DOOR_STEP_NB 1570
#define CAMION_POUBELLE_STEP 20

//Reponse event type 
#define EVENT_PUCKS_TAKE 0
#define EVENT_PUCKS_RAISE_SORT 1

#define EVENT_RAISE_TOWER 2
#define EVENT_RAISE_SCALE_DOOR 3

//Things to raise
#define RAISE_AND_SORT_ORDER 0
#define RAISE_TOWER_ORDER 1
#define RAISE_SCALE_DOOR_ORDER 2

//Game status
#define GAME_OFF 0
#define GAME_ON 1

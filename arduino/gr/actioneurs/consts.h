//pins
#define SELECTOR_PIN 1

#define PUMP_1 2
#define PUMP_2 3
#define PUMP_3 4
#define PUMP_ENABLE 5

#define TOWER_PIN 6
#define TOWER_LIMIT_PIN A2

#define SCALE_DOOR_PIN 7
#define SCALE_DOOR_LIMIT_PIN A4

#define PUCKS_DOOR_DIR_PIN 9
#define PUCKS_DOOR_STEP_PIN 10
#define PUCKS_DOOR_EN_PIN 11 

//delays
#define PUCKS_DOOR_MIN_DELAY 1000 //stepper
#define PUMP_DELAY 150 // in ms TODO:OPTIMIZE THIS
#define PUCK_TIME_TO_MOVE 600 // in ms TODO:OPTIMIZE THIS
#define SELECTOR_TIME_TO_MOVE 200 // in ms TODO:OPTIMIZE THIS

//Calibrations
#define SELECTOR_TO_TOWER 130
#define SELECTOR_TO_SCALE 40
#define PUCKS_DOOR_STEP_NB 1000

#define PUCK_GOES_TO_TOWER false
#define PUCK_GOES_TO_SCALE true

//Reponse event type 
#define EVENT_PUCKS_TAKE 0
#define EVENT_PUCKS_RAISE_SORT 1

#define EVENT_RAISE_TOWER 2
#define EVENT_RAISE_SCALE_DOOR 3

//Game status
#define GAME_OFF 0
#define GAME_ON 1

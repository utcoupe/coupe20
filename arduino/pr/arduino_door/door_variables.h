
// ----- Variables -------

// ~ PIN ~ 
int servo_door_1_pin      = 26 ; // ?
int servo_door_2_pin      = 28 ; // ? 
int servo_flipper_1_pin   = 22 ; // ? left flipper 
int servo_flipper_2_pin   = 24 ; // ? right flipper 


// ~ Constant ~ 
#define POS_FLIPPER_1_INIT      50  // ? 
#define POS_FLIPPER_1           50  // ? 
#define POS_FLIPPER_2_INIT      50  // ? 
#define POS_FLIPPER_2           50  // ? 
#define POS_DOOR_1_CLOSE        50  // ? 
#define POS_DOOR_1_OPEN         50  // ? 
#define POS_DOOR_2_CLOSE        50  // ? 
#define POS_DOOR_2_OPEN         50  // ? 

 
// ~ Status ~ 
int door_status          = -1 ;  // 1 : open door    0 : close door 
int current_door_status  = -1 ; 
int flipper_status       = -1 ;  // 1 : move flipper  0 : do nothing 
int init_status          = -1 ;  // 1 : initalize 
int side_status          = -1 ;  // 0 : left  1 : right 
int game_status          = -1 ;  // 1 : in game    0 : end game  


// ------ Functions --------- 
void door_initialize() ; 
void open_door() ; 
void close_door() ; 
void  flipper() ; 

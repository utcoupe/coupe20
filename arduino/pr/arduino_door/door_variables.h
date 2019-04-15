
// ----- Variables -------

// ~ PIN ~ 
int servo_door_1_pin      = 26 ; // ?
int servo_door_2_pin      = 28 ; // ?
int servo_flipper_1_pin   = 22 ; // ? 
int servo_flipper_2_pin   = 24 ; // ? 
 
// ~ Status ~ 
int door_status          = -1 ;  // 1 : open door    0 : close door 
int flipper_status       = -1 ;  // 1 : move flipper  0 : do nothing 
int init_status          = -1 ;  // 1 : initalize 
int side_status          = -1 ;  // 0 : left  1 : right 
int game_status          = -1 ;  // 1 : in game    0 : end game  


// ------ Functions --------- 
void door_initialize() ; 
int open_door() ; 
int close_door() ; 
int flipper() ; 


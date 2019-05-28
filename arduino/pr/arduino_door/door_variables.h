
// ----- Variables -------

// ~ PIN ~ 
int servo_door_1_pin      = 10 ; // ? left door 
int servo_door_2_pin      = 11 ; // ? right door 
int servo_flipper_1_pin   = 13 ; // ? left flipper 
int servo_flipper_2_pin   = 12 ; // ? right flipper 
int sucker_pin_1 = 6 ; 
int sucker_pin_2 = 7 ; 



// ~ Constant ~ 
#define POS_FLIPPER_1_IN        0     // left flipper 
#define POS_FLIPPER_1_OUT       50   
#define POS_FLIPPER_2_IN        150   // right flipper 
#define POS_FLIPPER_2_OUT       50  
#define POS_DOOR_1_CLOSE        100   // left door 
#define POS_DOOR_1_OPEN         0 
#define POS_DOOR_2_CLOSE        50    // ? right door  
#define POS_DOOR_2_OPEN         50    // ? 


 
// ~ Status ~ 
int door_status          = -1 ;  // 1 : open door    0 : close door 
int current_door_status  = -1 ; 
int flipper_status       = -1 ;  // 1 : move flipper  0 : do nothing 
int init_status          = -1 ;  // 1 : initalize 
int sucker_status        = -1 ;  // 0 : off     1 : on 
int game_status          =  1 ;  // 1 : in game    0 : end game  
// DANGER game_status = 1 for test but should be -1 
int side_status          = 1 ; // side of the game 0:left 1:right 
// DANGER and URGENT side_status = 1 for test but should be -1 

// ------ Functions --------- 
void door_initialize() ; 
void flipper_initialize(); 
void sucker_initialize(); 
void flipper() ;
void open_door() ; 
void close_door() ; 
void sucker_on() ; 
void sucker_off() ;  

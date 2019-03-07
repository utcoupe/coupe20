
// ----- Variables -------

// ~ PIN ~ 
int dir_pin        = 10 ;
int step_pin       = 9 ; 
int en_pin         = 11; 
uint16_t min_delay = 1000 ; 

int servo_unload_pin = 22 ; 
 
// ~ Servo ~ Unload Atom 
#define POS_UNLOAD_INIT  90  
#define POS_UNLOAD       0  //store servo position


// ~ Stepper ~ Load Atom 
int lift_position = 0 ; // no postion for now 
#define H_SAS_LOW  100  // ? 
#define H_FLOOR_5  85   // ? 
#define H_FLOOR_4  75   // ? 
#define H_FLOOR_3  65   // ? 
#define H_FLOOR_2  55   // ? 
#define H_FLOOR_1  45   // ? 
#define H_GROUND   0    // ? 
#define H_ATOM     4    // ? 



// ~ Number of atoms ~ 
#define MAX_ATOM_SAS 2  
float nb_atom_in_sas  = 0 ; //number of atom loaded in the sas (for slide)
float nb_atom_in      = 0 ; //number of atom loaded in total 
float nb_atom_out     = 0 ; 


// ~ Status ~ 
int load_content          = -1 ; // actions given to stepper  => 0 for nothing // 1 for load ONE atom // 2 for load tower of atoms 
int load_content_nb       = -1 ; // number of atoms to load if load_content = 2 
int load_content_position = -1 ; 
int unload_content        = -1 ; // actions given to stepper  => 0 for nothing // 1 for unoload 
int load_success          = -1 ; // info on stepper status 
int unload_success        = -1 ; // info on stepper status 
int game_status           = -1 ; // info on game status  // 1 ingame 


// ------ Functions --------- 
void tower_initialize() ; 
int move_lift(int wanted_position) ; 
int lift_atoms_to_sas() ; 
void unload_atom() ; 
int unload_atom_slider() ; 
int unload_atom_pliers() ; 
void load_atom() ; 
void load_atom_single() ; 
void load_atom_tower() ; 

// ----- Variables -------

// ~ PIN ~ 
int dir_pin        = 10 ;
int step_pin       = 9 ; 
int en_pin         = 11; 
uint16_t min_delay = 1000 ; 

int servo_unload_pin_2 = 13 ; //number 2 right side 
int servo_unload_pin_1 = 12 ; //number 1 left side 

const int button_up_pin = 2 ;   // button up for lift 
const int button_down_pin = 3 ; // button down for lift  
 
// ~ Servo ~ Unload Atom 
#define POS_UNLOAD_INIT_2   125//90   
#define POS_UNLOAD_2        200  //store servo position
#define POS_UNLOAD_INIT_1   50
#define POS_UNLOAD_1        0

// ~ Stepper ~ Load Atom 
int lift_position = 0 ; // no postion for now 
#define H          300      // H_FLOOR_1 = 300 * 2 
#define H_SAS_LOW  2000     // 1800 avant 
#define H_FLOOR_3  1200     // ? 
#define H_FLOOR_2  1000     // ? 
#define H_FLOOR_1  500      // ? 
#define H_GROUND   0        // ? 
#define H_ATOM     200      // Verified 200~160
#define H_ERREUR   200      // voir pour meilleure précision


// ~ Number of atoms ~ 
#define MAX_ATOM_SAS 3    // number given by mecas (correspond to the sas capacity)  
int nb_atom_in_sas  = 0 ; //number of atom loaded in the sas (for slide)
int nb_atom_in      = 0 ; //number of atom loaded in total 
int nb_atom_out     = 0 ; 


// ~ Status ~ 
int load_content                        = -1 ; // actions given to stepper  => 0 for nothing // 1 for load ONE atom // 2 for load tower of atoms 
int load_content_nb_tower_wanted        = -1 ; // number of atoms to load from tower if load_content = 2 
int load_content_nb_tower               = -1 ; // number of atoms in total in the tower if load_content = 2      
int unload_content                      = -1 ; // actions given to stepper  => 0 for nothing // 1 for unoload 
int load_success                        = -1 ; // info on stepper status 
int unload_success                      = -1 ; // info on stepper status 
int game_status                         = 1 ; // info on game status  // 1 ingame 
// DANGER game_status = 1 for test but should be -1 

// ------ Functions --------- 
void tower_initialize() ; 
void lift_sas() ; 
void lift_ground(); 
int move_lift(int wanted_position) ; 
//int lift_atoms_to_sas() ; 
int unload_atom_sas() ; 
int load_atom_sas() ; 
void unload_atom() ; 
int unload_atom_slider() ; 
int unload_atom_pliers() ; 
void load_atom() ; 
int load_atom_single() ; 
int load_atom_tower(float nb_atom_tower, float nb_atom_wanted); 

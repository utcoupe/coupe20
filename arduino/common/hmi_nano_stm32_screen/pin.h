
// Buttons Pins 
const int button_team     = A1;     // pushbutton pin

// Led Pins 
const int PIN_LED_ALIVE  =  10 ;     // green light of game status 
const int PIN_LED_INIT   =  12 ;     // blue light for init 
const int PIN_LED_JACK   =  11 ;     // red light for jack 

// Jack Pins 
const int jackPin = A2 ; // high when jack not in and low when jack in 
// SCK is D5 for STM32 
// SDA is D4 for STM32 

// Screen Pins 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16


// Status 
int game_status      = -1 ;   
int first_msg        = -1 ;  // first msg receive by game status  -1 for game status and 1 for chosen_stratety and chosen_team
int init_status      = -1 ; 

int chosen_strategy_id  = 0  ;  // no strategy to chose  here 
int chosen_team_id      = -1 ; // 0 : left and 1 : right given with the button 

int ask_jack_status   = -1 ;   // Sent by ai/scheduler when it strarts waiting for the jack.  // 1 waiting for jack and 0 
int jack_state        = -1 ; 

int score             = -1 ; // score send by scheduler 
float timeLeft        = -1 ; // time  send by 

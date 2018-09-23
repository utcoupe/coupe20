/*
Using the Arduino IDE, install the library "ESP8266 SSD1306 OLED" Display lib.
This code was developed with a Wemos D1 Mini (ESP8266) and a 128x64 i2c OLED.
Author: Pierre LACLAU, December 2017, UTCoupe.
*/

#include "pins.h"

// Including rosserial
#include <ros.h>
#include <drivers_ard_hmi/SetStrategies.h>  // ROS sets strategy names for displaying on hmi.
#include <drivers_ard_hmi/SetTeams.h>       // ROS sets teams names for displaying on hmi.
#include <drivers_ard_hmi/HMIEvent.h>       // HMI sends events : JACK, GAME_STOP
#include <drivers_ard_hmi/ROSEvent.h>       // ROS sends events : ASK_JACK, GAME_STOP
#include <ai_game_manager/GameStatus.h>      // ROS sends game and init status (to  determine when RPi ready).
#include <ai_game_manager/GameTime.h>              // ROS sends the timer status (for the ingame progress bar).
#include <ai_scheduler/AIScore.h>           // ROS sends the timer status (for the ingame progress bar).
ros::NodeHandle nh;

//Creating OLED display instances
#include <Wire.h>    // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplayUi.h"
SSD1306  display(0x3c, LCD_SDA, LCD_SCK);
OLEDDisplayUi ui(&display);

//Component variables
uint8_t current_frame;
uint8_t cursor_pos;

const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

//HMI variables
String strats[12];
uint8_t strats_count = 0;

String teams[12];
uint8_t teams_count = 0;

int chosen_strat = -1;
int chosen_team = -1;
int current_score = 0;

int game_status = -1;
int init_status = -1;
float game_duration = -1;
float elapsed_time = -1;

bool _is_arming = false;
bool _is_launching = false;

//Input
#define PIN_BTN_VCC_1 BTN_L2
#define PIN_BTN_VCC_2 BTN_L1
#define PIN_BTN_IN_1  BTN_R1
#define PIN_BTN_IN_2  BTN_R2
#define PIN_JACK      JACK

bool _prev_up_state    = false;
bool _prev_down_state  = false;
bool _prev_left_state  = false;
bool _prev_right_state = false;
bool _prev_jack_state = false;

bool up_pressed    = false;
bool down_pressed  = false;
bool left_pressed  = false;
bool right_pressed = false;
bool jack_pulled   = false;

void check_input() {
    digitalWrite(PIN_BTN_VCC_1, HIGH);
    digitalWrite(PIN_BTN_VCC_2, LOW);
    delay(2);
    up_pressed = digitalRead(PIN_BTN_IN_1) && !_prev_up_state;
    _prev_up_state = digitalRead(PIN_BTN_IN_1);
    down_pressed = digitalRead(PIN_BTN_IN_2) && !_prev_down_state;
    _prev_down_state = digitalRead(PIN_BTN_IN_2);

//    if (digitalRead(PIN_JACK) == 0) {
//        nh.loginfo("bas");
//    } else if (digitalRead(PIN_JACK) == 1) {
//        nh.loginfo("haut");
//    }

    jack_pulled = digitalRead(PIN_JACK) && !_prev_jack_state; // 1 when inserted
    _prev_jack_state = digitalRead(PIN_JACK);
  
    digitalWrite(PIN_BTN_VCC_1, LOW);
    digitalWrite(PIN_BTN_VCC_2, HIGH);
    delay(2);
    left_pressed = digitalRead(PIN_BTN_IN_1) && !_prev_left_state;
    _prev_left_state = digitalRead(PIN_BTN_IN_1);
    right_pressed = digitalRead(PIN_BTN_IN_2) && !_prev_right_state;
    _prev_right_state = digitalRead(PIN_BTN_IN_2);
}

//LEDs
#define PIN_LED_ALIVE LED_GREEN
#define PIN_LED_INIT LED_RED

void update_leds() {
    digitalWrite(PIN_LED_ALIVE, game_status != -1);

    if(init_status != -1) {
        if(init_status == 0) // some nodes have not responded yet
            analogWrite(PIN_LED_INIT, 127 * (1 + cos(float(millis()) / 200.0))); // wave
        else if(init_status == 1) // init finished and all nodes are ready
            analogWrite(PIN_LED_INIT, 255); //solid ON
        else if(init_status == 2) // init finished but at least one node failed
            analogWrite(PIN_LED_INIT, 255 * (millis() % 1000 < 700)); // blink
    }
    else digitalWrite(PIN_LED_INIT, LOW);
}


//ROS methods
void on_set_strategies(const drivers_ard_hmi::SetStrategies& msg){
  strats_count = msg.strategies_names_length;
  for(int i=0; i < msg.strategies_names_length; i++) {
    strats[i] = msg.strategies_names[i];
  }
}

void on_set_teams(const drivers_ard_hmi::SetTeams& msg){
  teams_count = msg.teams_names_length;
  for(int i=0; i < msg.teams_names_length; i++) {
    teams[i] = msg.teams_names[i];
  }
}

void on_game_status(const ai_game_manager::GameStatus& msg){
    game_status = msg.game_status;
    init_status = msg.init_status;

    if(game_status == 1 && current_frame != 5 /*INGAME*/)
        ui.switchToFrame(5);
}

void on_game_timer(const ai_game_manager::GameTime& msg){
    if(msg.is_active) {
        game_duration = msg.game_time_duration;
        elapsed_time = msg.game_elapsed_time;
    }
}

void on_score(const ai_scheduler::AIScore& msg){
    current_score = msg.score;
}

void on_ros_event(const drivers_ard_hmi::ROSEvent& msg){
    if(msg.event == 0) //asked to respond for JACK
        ui.switchToFrame(4);
}

ros::Subscriber<drivers_ard_hmi::SetStrategies> sub_strats     ("/feedback/ard_hmi/set_strategies", &on_set_strategies);
ros::Subscriber<drivers_ard_hmi::SetTeams>      sub_teams      ("/feedback/ard_hmi/set_teams", &on_set_teams);
ros::Subscriber<drivers_ard_hmi::ROSEvent>      sub_ros_events ("/feedback/ard_hmi/ros_event", &on_ros_event);
ros::Subscriber<ai_game_manager::GameStatus>    sub_game_status("/ai/game_manager/status",     &on_game_status);
ros::Subscriber<ai_game_manager::GameTime>             sub_game_timer ("/ai/game_manager/time",       &on_game_timer);
ros::Subscriber<ai_scheduler::AIScore>          sub_score      ("/ai/scheduler/score",         &on_score);

drivers_ard_hmi::HMIEvent hmi_event_msg;
ros::Publisher hmi_event_pub("/feedback/ard_hmi/hmi_event", &hmi_event_msg);



void drawFrameTitleComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String text) {
    display->setFont(ArialMT_Plain_16);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 0 + y, text);
}

void drawBigCentralMessageComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String text) {
    display->setFont(ArialMT_Plain_24);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 22 + y, text);
}

void drawSmallCentralMessageComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String text) {
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 28 + y, text);
}

void drawMCQComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String options[], uint8_t options_count) {
    if(options_count > 0) {
        int y_offset = 12;
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        
        int start = cursor_pos - 1;
        if(cursor_pos == 0) start = 0;
        if(cursor_pos == options_count - 1) {
            start = options_count - 3;
            if(options_count == 2) start = options_count - 2;
            if(options_count == 1) start = options_count - 1;
        }
        
        for(int i = 0; i < 3; i++) {
            display->drawString(11 + x, 16 + y + y_offset * i, options[start + i]);
            if(cursor_pos == start + i) {
                display->drawString(4 + x, 16 + y + y_offset * i, ">");
            }
        }
        
        char b[5];
        sprintf(b, "%d/%d",cursor_pos + 1,options_count);
        display->drawString(4 + x, 52 + y, b);
    }
    else {
        drawSmallCentralMessageComponent(display, state, x, y, "No options.");
    }
}

//Frames rendering methods
void drawHelloFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 0;
    drawFrameTitleComponent(display, state, x, y, "RPi init...");
    if(init_status == -1) drawBigCentralMessageComponent(display, state, x, y, "WAIT");
    else drawBigCentralMessageComponent(display, state, x, y, "READY");

    if(game_status == 0 && init_status != 0 && strats_count && teams_count)
        ui.nextFrame();
    if(right_pressed && game_status != -1) // go to next screen if rpi is alive only. 
        ui.nextFrame();
}

void drawTeamFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 1;
    drawFrameTitleComponent(display, state, x, y, "Team select");
    drawMCQComponent(display, state, x, y, teams, teams_count);

    if(right_pressed) {
        chosen_team = cursor_pos;
        cursor_pos = 0;
        ui.nextFrame();
    }
    if(left_pressed) {
        cursor_pos = 0;
        ui.previousFrame();
    }
    if(down_pressed && teams_count) {
        cursor_pos += 1;
        if(cursor_pos >= teams_count) cursor_pos = teams_count - 1;
    }
    if(up_pressed && teams_count) {
        if(cursor_pos > 0) cursor_pos -= 1;
    }
}

void drawStrategyFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 2;
    drawFrameTitleComponent(display, state, x, y, "Strat select");
    drawMCQComponent(display, state, x, y, strats, strats_count);

    if(right_pressed) {
        chosen_strat = cursor_pos;
        cursor_pos = 0;
        ui.nextFrame();
    }
    if(left_pressed) {
        cursor_pos = 0;
        ui.previousFrame();
    }
    if(down_pressed && strats_count) {
        cursor_pos += 1;
        if(cursor_pos >= strats_count) cursor_pos = strats_count - 1;
    }
    if(up_pressed && strats_count) {
        if(cursor_pos > 0) cursor_pos -= 1;
    }
}

void drawArmFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 3;
    drawFrameTitleComponent(display, state, x, y, "Calibration");
    if(!_is_arming) drawBigCentralMessageComponent(display, state, x, y, "ARM?");
    else drawBigCentralMessageComponent(display, state, x, y, "ARMING...");

    if(left_pressed) {
        _is_arming = false; //todo good ?
        ui.previousFrame();
    }
    if(right_pressed) {
        if(!_is_arming) {
            hmi_event_msg.event = hmi_event_msg.EVENT_START; // Will start ai/scheduler with selected config.
            hmi_event_msg.chosen_team_id = chosen_team;
            hmi_event_msg.chosen_strategy_id = chosen_strat;
            hmi_event_pub.publish(&hmi_event_msg); // send config.
            _is_arming = true;
        }
    }
}

void drawJackFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 4;
    drawFrameTitleComponent(display, state, x, y, "Jack");
    if(!_is_launching) {
        if(_prev_jack_state) drawBigCentralMessageComponent(display, state, x, y, "JACK?");
        else drawSmallCentralMessageComponent(display, state, x, y, "Insert jack.");
    }
    else drawBigCentralMessageComponent(display, state, x, y, "JACKED!");

    if(left_pressed) {
        _is_launching = false; //TODO good ?
        ui.previousFrame();
    }
    if(jack_pulled) {
        if(!_is_launching) {
            hmi_event_msg.event = hmi_event_msg.EVENT_JACK_FIRED; //JACKED
            hmi_event_msg.chosen_strategy_id = 0; // resetting values 
            hmi_event_msg.chosen_team_id     = 0; // from previous send.
            hmi_event_pub.publish(&hmi_event_msg); 
            _is_launching = true;
        }
    }
}

void drawInGameFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 5;
    ui.disableIndicator();
    drawFrameTitleComponent(display, state, x, y, "In Game...");
    
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_24);
    String text = String(current_score) + "pts";
    display->drawString(64 + x, 22 + y, text);
    
    int percentage = 0;
    if(elapsed_time != -1 && game_duration > 0) percentage = int(100 * elapsed_time / game_duration);
    display->drawProgressBar(x + 0, y + 54, 120, 8, percentage); // shows time left

    if(left_pressed) ui.previousFrame();
    if(right_pressed) ui.nextFrame();
}

// frames are the single views that slide in
FrameCallback frames[] = {drawHelloFrame, drawTeamFrame, drawStrategyFrame, drawArmFrame, drawJackFrame, drawInGameFrame};
int frameCount = 6;

void setup() {
    pinMode(PIN_BTN_VCC_1, OUTPUT); // init input buttons
    pinMode(PIN_BTN_VCC_2, OUTPUT);
    pinMode(PIN_BTN_IN_1,  INPUT);
    pinMode(PIN_BTN_IN_2,  INPUT);
    pinMode(PIN_JACK,      INPUT);

    pinMode(PIN_LED_ALIVE, OUTPUT);
    pinMode(PIN_LED_INIT, OUTPUT);

    nh.initNode();
    nh.subscribe(sub_strats);
    nh.subscribe(sub_teams);
    nh.subscribe(sub_game_status);
    nh.subscribe(sub_game_timer);
    nh.subscribe(sub_score);
    nh.subscribe(sub_ros_events);
    nh.advertise(hmi_event_pub);
    
    ui.setTargetFPS(30);
    
    ui.setActiveSymbol(activeSymbol);
    ui.setInactiveSymbol(inactiveSymbol);
    ui.setFrameAnimation(SLIDE_LEFT);
    ui.setFrames(frames, frameCount);
    
    ui.disableAutoTransition();
    ui.init();
    
    display.flipScreenVertically();
}


void loop() {
    check_input();
    update_leds();
    ui.update();

    if(game_status == -1) {
        hmi_event_msg.event = hmi_event_msg.EVENT_HMI_INITIALIZED; // tell ros that hmi is alive until it responds back
        hmi_event_pub.publish(&hmi_event_msg);
    }

    nh.spinOnce();
    delay(30); // Needed for input to work properly, no idea why...
}

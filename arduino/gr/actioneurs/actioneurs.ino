#include "consts.h"
#include "PololuA4983.h"
#include <Servo.h>

#include <ros.h>
#include <game_manager/GameStatus.h>
#include <ard_gr_front/PucksTake.h>
#include <ard_gr_front/PucksRaiseSort.h>
#include <ard_gr_front/RaiseScaleDoor.h>
#include <ard_gr_front/RaiseTower.h>
#include <ard_gr_front/ArduinoToAI.h>


ros::NodeHandle nh;
PololuA4983 stepper_pucks_door = PololuA4983(PUCKS_DOOR_STEP_PIN, PUCKS_DOOR_DIR_PIN,
                                 PUCKS_DOOR_EN_PIN, PUCKS_DOOR_MIN_DELAY);
Servo selector;

void moveServo(const int POS){
  selector.write(POS);
  delay(1500);
}

int game_status = GAME_OFF;
int pucks_door_steps_taken = 0;

bool puck_to_scale[3];
static int PUMP[3] = {PUMP_1, PUMP_2, PUMP_3};


// Publisher
ard_gr_front::ArduinoToAI  event_msg ;
ros::Publisher pub_response  ("actionneurs/ard_gr_front/event", &event_msg);

//Debugging
//void printInt(int value) {
//  char string[2];
//  nh.loginfo(itoa(value, string, 10));
//  nh.loginfo(string);
//}

void publish_response (int event_type, bool success) {
  event_msg.type = event_type;
  event_msg.success = success;
  pub_response.publish(&event_msg);
}

void initialize() {
  nh.loginfo("INITIALIZING");
  //selector.write(SELECTOR_TO_SCALE);
  //pucks_door_goes_up(false);
}

void on_game_status(const game_manager::GameStatus& msg) {

  game_status = GAME_ON;//msg.game_status;
  //printInt(game_status);
  // while (game_status != GAME_ON) {
  // 'infinite' loop for hard stop until msg.game_status == 1 again
  //}

  if (msg.init_status == 1 && game_status == GAME_ON)
    initialize();
}

void suck_up_pucks() {

  //digitalWrite(PUMP_ENABLE, HIGH);
  //delay(PUMP_DELAY);
  //for (int i = 0; i < 3; i++) {
    //digitalWrite(PUMP[i], HIGH);
  //}
  //delay(PUMP_DELAY);
}

void on_take_pucks(const ard_gr_front::PucksTake& msg) {
  nh.loginfo("Received take pucks");
  //printInt(game_status);
  if (game_status != GAME_ON) {
    publish_response(EVENT_PUCKS_TAKE, true);
    return;
  }

  int puck_to_scale[3] = {msg.P1, msg.P2, msg.P3};
  suck_up_pucks();
}

void free_puck_to_sort(int i) {
  delay(SELECTOR_TIME_TO_MOVE);
  digitalWrite(PUMP[i], LOW);
  delay(PUCK_TIME_TO_MOVE);
}

void pucks_door_goes_up(bool go_up) {
  if (go_up)
    stepper_pucks_door.moveStep(PUCKS_DOOR_STEP_NB, true);
  else
    stepper_pucks_door.moveStep(- pucks_door_steps_taken, true);

  while (stepper_pucks_door.getRemainingStep() > 0) {
    stepper_pucks_door.update();
    if (go_up)
      pucks_door_steps_taken++;
    else
      pucks_door_steps_taken--;

  }
  stepper_pucks_door.stop();
}

void dump_in_scale() {
  //selector.write(SELECTOR_TO_SCALE);
  delay(SELECTOR_TIME_TO_MOVE);

  for (int i = 0; i < 3; i++) {
    if (puck_to_scale[i] == PUCK_GOES_TO_SCALE)
      free_puck_to_sort(i);
  }
}

void dump_in_tower() {
  //selector.write(SELECTOR_TO_TOWER);
  for (int i = 0; i < 3; i++) {
    if (puck_to_scale[i] == PUCK_GOES_TO_TOWER)
      free_puck_to_sort(i);
  }
}
void on_raise_and_sort_pucks(const ard_gr_front::PucksRaiseSort& msg) {
  if (game_status != GAME_ON) {
    publish_response(EVENT_PUCKS_RAISE_SORT, true);
    return;
  }

  pucks_door_goes_up(true);

  dump_in_tower();
  dump_in_scale();
  
  digitalWrite(PUMP_ENABLE, LOW);
  //selector.write(SELECTOR_TO_TOWER);

  pucks_door_goes_up(false);

  publish_response(EVENT_PUCKS_RAISE_SORT, true);

}


void on_raise_scale_door(const ard_gr_front::RaiseScaleDoor& msg) {
  if (game_status != GAME_ON) {
    publish_response(EVENT_RAISE_SCALE_DOOR, false);
    return;
  }
  digitalWrite(SCALE_DOOR_PIN, HIGH);
  while (digitalRead(SCALE_DOOR_LIMIT_PIN) != HIGH) {
    // Wait
  }
  digitalWrite(SCALE_DOOR_PIN, LOW);

  publish_response(EVENT_RAISE_SCALE_DOOR, true);
}

void on_raise_tower(const ard_gr_front::RaiseTower& msg) {
  if (game_status != GAME_ON) {
    publish_response(EVENT_RAISE_TOWER, false);
    return;
  }

  digitalWrite(TOWER_PIN, HIGH);
  while (digitalRead(TOWER_LIMIT_PIN) != HIGH) {
    // Wait
  }
  digitalWrite(TOWER_PIN, LOW);

  publish_response(EVENT_RAISE_TOWER, true);
}


//Subscribers
ros::Subscriber<game_manager::GameStatus>     sub_game_status("ai/game_manager/status", &on_game_status);

ros::Subscriber<ard_gr_front::PucksRaiseSort> sub_raise_sort_pucks("actionneurs/raise_and_sort_pucks", &on_raise_and_sort_pucks);
ros::Subscriber<ard_gr_front::PucksTake>      sub_take_pucks("actionneurs/take_pucks", &on_take_pucks);

ros::Subscriber<ard_gr_front::RaiseScaleDoor>  sub_raise_scale_door("actionneurs/raise_scale_door", &on_raise_scale_door);
ros::Subscriber<ard_gr_front::RaiseTower>      sub_raise_tower("actionneurs/raise_tower", &on_raise_tower);


void connectPins() {

  pinMode(PUMP_ENABLE, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(PUMP_3, OUTPUT);

  pinMode(PUCKS_DOOR_DIR_PIN, OUTPUT);
  pinMode(PUCKS_DOOR_STEP_PIN, OUTPUT);
  pinMode(PUCKS_DOOR_EN_PIN, OUTPUT);

  pinMode(SCALE_DOOR_PIN, OUTPUT);
  pinMode(TOWER_PIN, OUTPUT);

  pinMode(SCALE_DOOR_LIMIT_PIN, INPUT);
  pinMode(TOWER_LIMIT_PIN, INPUT);

  selector.attach(SELECTOR_PIN);

}

void connectROS() {
  nh.initNode();

  nh.subscribe(sub_game_status);

  nh.subscribe(sub_take_pucks);
  nh.subscribe(sub_raise_sort_pucks);

  nh.subscribe(sub_raise_scale_door);
  nh.subscribe(sub_raise_tower);
  nh.advertise(pub_response);

  nh.getHardware()->setBaud(115200);

}
void setup() {

  connectPins();
  connectROS();

}


void loop() {
  moveServo(SELECTOR_TO_TOWER);
  moveServo(SELECTOR_TO_SCALE);
  nh.loginfo("moving");
  nh.spinOnce();
  //delay(5); // seen on a blog to not overwhelm master
}

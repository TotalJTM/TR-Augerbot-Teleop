#include <Kangaroo.h>
//#include "Kangaroo_Motion_Controller/Kangaroo.h" //doesnt build, library should be installed wherever Arduino installs local copies of libraries
#include "Sabertooth.h"
#include "AccelStepper/AccelStepper.h"
#include "rmc_stepper.h"

/*
   ON Kangaroo:
   Front Left Motor: 1 on Kangar;oo 1
   Front Right Motor: 2 on Kangaroo 1
   Back Left Motor: 1 on Kangaroo 2
   Back Right Motor: 2 on Kangaroo 2
*/

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
// Front Kangaroo Serial Port
KangarooSerial  KF(Serial2);

// Back Kangaroo Serial Port

KangarooSerial  KB(Serial1);

// Front Kangaroo Channel
KangarooChannel KLF(KF, '1', 128); //Front Left
KangarooChannel KRF(KF, '2', 128); //Front Right

// Back Kangaroo Channel
KangarooChannel KLB(KB, '1', 128); //Back Left
KangarooChannel KRB(KB, '2', 128); //Back Right


// Sabertooth digging and dumping systems
Sabertooth motors(129, Serial3);
Sabertooth lift(128, Serial3);

// Stepper
// dir pin is 4 pull pin is 5
// common fucking ground that shit
const unsigned char limit_pin = 3;
RmcStepper powerscrew(5, 4, 100.0f);
bool stepper_on = false;
long pause = -1;
long old_pause = -1;

// Motor speeds (for auger drive 1 is digging direction)
const char NUM_MOTOR_SPEEDS = 3;
const int MOTOR_SPEEDS[3] = {0, 127, -127};
const int BELT_SPEEDS[3] = {0, -127, 127};

// Type declarations
struct DriveMessage {
  long left_speed, right_speed;
  long auger_lift, auger_slide, auger_drive;
  long belt_lift, belt_drive;
};
struct FeedbackMessage {
  long left_front_speed, left_back_speed, right_front_speed, right_back_speed, stepper_pos;
};

// Serial communication buffers
const unsigned int BUFFER_SIZE = 256;
char message_buffer[BUFFER_SIZE];
char feedback_buffer[BUFFER_SIZE];

// Control variables
long left_speed = 0, right_speed = 0;
const unsigned int MAX_SPEED_CHANGE = 100; //orginally 40
const unsigned int DELAY_TIME = 10;
DriveMessage message;

//message should look like <command, int_or_long_values_seperated_with_commas>
void handle_message() {
  char* last_token = strtok(message_buffer, "<");
  char* next_token;

  int16_t command = strtol(last_token, &next_token, 10);
  last_token = next_token + 1;

  //if(message_buffer[last_token]

  // Consider cleaning this section up, maybe switch back to reinterpret casting
  // a packed struct?
  //<1,0,0,0,0,0,0,0>
  // left speed, right speed, char [auger lift, auger slide, auger drive, belt lift, belt drive]
  if(command == 1){
    message.left_speed = strtol(last_token, &next_token, 10);
    last_token = next_token + 1;
    message.right_speed = strtol(last_token, &next_token, 10);
    last_token = next_token + 1;
    message.auger_lift = strtol(last_token, &next_token, 10);
    last_token = next_token + 1;
    message.auger_slide = strtol(last_token, &next_token, 10);
    last_token = next_token + 1;
    message.auger_drive = strtol(last_token, &next_token, 10);
    last_token = next_token + 1;
    message.belt_lift = strtol(last_token, &next_token, 10);
    last_token = next_token + 1;
    message.belt_drive = strtol(last_token, &next_token, 10);
  }
  if(command == 0){
    publish_feedback();
  }
}

void update_motors() {
  /*if(message.left_speed == message.right_speed){  
    left_speed = constrain(message.left_speed, left_speed - MAX_SPEED_CHANGE, left_speed + MAX_SPEED_CHANGE);
    right_speed = constrain(message.right_speed, right_speed - MAX_SPEED_CHANGE, right_speed + MAX_SPEED_CHANGE);
  }
  else{
    left_speed = message.left_speed;
    right_speed = message.right_speed;
  }
  */
  //left_speed = constrain(message.left_speed, left_speed - MAX_SPEED_CHANGE, left_speed + MAX_SPEED_CHANGE);
  //right_speed = constrain(message.right_speed, right_speed - MAX_SPEED_CHANGE, right_speed + MAX_SPEED_CHANGE);
  //left_speed = message.left_speed;
  //right_speed = message.right_speed;

  KLF.s(left_speed);
  KLB.s(left_speed);

  KRF.s(right_speed);
  KRB.s(right_speed);
  //0 = STOP, 1 = UP, 2 = DOWN
  //Serial.println(message.auger_lift);
  if (message.auger_lift >= 0 && message.auger_lift < NUM_MOTOR_SPEEDS) {
    lift.motor(1, MOTOR_SPEEDS[message.auger_lift]);
  }

  if (message.belt_lift >= 0 && message.belt_lift < NUM_MOTOR_SPEEDS) {
    lift.motor(2, MOTOR_SPEEDS[message.belt_lift]);
  }

  if (message.belt_drive >= 0 && message.belt_drive < NUM_MOTOR_SPEEDS) {
    motors.motor(1, BELT_SPEEDS[message.belt_drive]);
  }

  if (message.auger_drive >= 0 && message.auger_drive < NUM_MOTOR_SPEEDS) {
    motors.motor(2, MOTOR_SPEEDS[message.auger_drive]);
  }

  switch (message.auger_slide)
  {
    case 0:
      // stop
      pause = powerscrew.getCurrentPos();
      powerscrew.stop();
      break;
    case 1:
      // Move down track (forward)
      if (old_pause != pause){
        powerscrew.setCurrentPos(pause);
      }
      powerscrew.setGoalPos(-78000);
      old_pause = pause;
      powerscrew.start();
      break;
    case 2:
      // Move up track (backward)
      // this will set the GoalPos to tbd
      if (old_pause != pause){
        powerscrew.setCurrentPos(pause);
      }
      powerscrew.setGoalPos(-2);
      old_pause = pause;
      powerscrew.start();
      break;
  }
}

void publish_feedback() {
  
  Serial.print('<');
  Serial.print(KLF.getS().value());
  Serial.print(',');
  Serial.print(KLB.getS().value());
  Serial.print(',');
  Serial.print(KRF.getS().value());
  Serial.print(',');
  Serial.print(KRB.getS().value());
  Serial.print(',');
  Serial.print(powerscrew.getCurrentPos());
  Serial.println('>');

//not sure why sprintf didn't work...
  //sprintf(feedback_buffer, "<%d,%d,%d,%d,%d,%d,%d,%d,%d\n>", feedback.left_front_speed, feedback.left_back_speed,
          //feedback.right_front_speed, feedback.right_back_speed, feedback.stepper_pos,
          //message.auger_lift, message.belt_lift, message.belt_drive, message.auger_drive);
  //Serial.println(feedback_buffer);
  // <0,0,0,0,0,0,0,0,0>
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  //lift.autobaud();
  motors.autobaud();

 Serial.println("<CONTROL_STARTED>");

  // Front motors
  KLF.start();
  KLF.home().wait();
  KRF.start();
  KRF.home().wait();

  // Back motors
  KLB.start();
  KLB.home().wait();
  KRB.start();
  KRB.home().wait();

  // Stepper Init (home top to zero)
  pinMode(limit_pin, INPUT_PULLUP);
  long initial_homing = 400;

//Serial.println("Stepper is Homing...");
  while (digitalRead(limit_pin)) {
    //Serial.println("in 1st loop");
    powerscrew.setGoalPos(initial_homing);
    initial_homing = initial_homing + 400;
    powerscrew.start();
    delay(5);
  }

  powerscrew.setCurrentPos(0);
  initial_homing = -400;

  while (!digitalRead(limit_pin)) {
    //Serial.println("in 2nd loop");
    powerscrew.setGoalPos(initial_homing);
    powerscrew.start();
    initial_homing = initial_homing - 400;
    delay(5);
  }
  powerscrew.stop();
  powerscrew.setCurrentPos(0);
  //Serial.println("Homing Complete");
}

void loop() {
  // Read all serial data up to end of packet
  if (Serial.available() > 0) {
    unsigned int message_length = Serial.readBytesUntil('>', message_buffer, 256);
    if (message_length > 0) {
      message_buffer[message_length] = '\0';
      handle_message();
    }
  }
  update_motors();
}

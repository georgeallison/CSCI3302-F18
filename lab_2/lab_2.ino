/* CSCI 3302 Introduction to Robotics
 * Lab 2 Odometry | September 2018
 * George Allison, Pierce Doogan
 */
#include <sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

//odometry variables
float pose_x = 0., pose_y = 0., pose_theta = 0.;
const float speed = 0.0273; //meters per second
const float wDiam = 0.0857; //diameter between wheels
const float pi = 3.1415;
const int rot = 50; //degrees sparki can rotate at 10Hz
int dir; //used for logic in UpdateOdometry()

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = sparki.ping();
}

void measure_30cm_speed() {
  //function prints values to the Arduino serial monitor
  Serial.print("Time:");
  time = millis();
  sparki.moveForward(30);//move 30 cm
  Serial.println(millis() - time);
  delay(1000);
}

void updateOdometry() {
  if (dir == 0){ //turning left
    pose_theta -= rot * 0.1;
  }
  if (dir == 1){ //turning right
    pose_theta += rot * 0.1;
  }
  if (dir == 2){ //going straight
    pose_y += .1 * speed * sin(pose_theta * (pi / 180));
    pose_x += .1 * speed * cos(theta * (pi / 180));
  }
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("x pos: "); sparki.println(pose_x);
  sparki.print("y pos: "); sparki.println(pose_y);
  sparki.print("theta: "); sparki.println(pose_theta);
  sparki.updateLCD();
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      if (lineLeft < threshold){ 
        sparki.moveLeft();
        dir = 0;
        updateOdometry();
        displayOdometry();
      }
      if (lineRight < threshold) {  
        sparki.moveRight();
        dir = 1;
        updateOdometry();
        displayOdometry();
      }
      if ((lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold)){
        sparki.moveForward();
        dir = 2;
        updateOdometry();
        displayOdometry();
      }
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }

  delay(1000*CYCLE_TIME);
}


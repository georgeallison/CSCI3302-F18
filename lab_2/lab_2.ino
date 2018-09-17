/* CSCI 3302 Introduction to Robotics
 * Lab 2 Odometry | September 2018
 * George Allison, Pierce Doogan
 */
#include <sparki.h>

#define CYCLE_TIME .100  // seconds

int current_state = 0; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

//odometry variables
float pose_x = 0., pose_y = 0., pose_theta = 0.;
const float speed = 0.0273; //meters per second
const float wDiam = 0.0857; //diameter between wheels
const float pi = 3.1415;
const int rot = 112; //degrees sparki can rotate at 10Hz
int dir; //used for logic in UpdateOdometry()
int distance;
int time;

void setup() {
  sparki.RGB(RGB_GREEN);
  sparki.servo(SERVO_CENTER);
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  delay(1000);
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
    pose_x += .1 * speed * cos(pose_theta * (pi / 180));
  }
  if (dir == 3){ //resetting values
    pose_y = 0;
    pose_x = 0;
    pose_theta = 0;
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
  
  readSensors();
  
  switch (current_state) {
    case 0:
      if (line_left < threshold){ 
        sparki.RGB(RGB_YELLOW);
        sparki.moveLeft();
        dir = 0;
      }
      if (line_right < threshold) {
        sparki.RGB(RGB_BLUE);
        sparki.moveRight();
        dir = 1;
      }
      if ((line_center < threshold) && (line_left > threshold) && (line_right > threshold)){
        sparki.RGB(RGB_PINK);
        sparki.moveForward();
        dir = 2;
      }
      if ((line_center < threshold) && (line_left < threshold) && (line_right < threshold)){
        sparki.RGB(RGB_GREEN);
        sparki.beep();
        sparki.moveForward();
        dir = 3;//end cycle
      }
      
      updateOdometry();
      displayOdometry();
      
    //case 1:
      //measure_30cm_speed();
  }

  delay(1000*CYCLE_TIME);
}


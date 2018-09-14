/* CSCI 3302 Introduction to Robotics
 * Lab 2 Odometry | September 2018
 * George Allison, Pierce Doogan
 */
 
#include <sparki.h>
#define CYCLE_TIME 1000 //milliseconds

//Program States:
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2

int current_state = CONTROLLER_FOLLOW_LINE; //used in switch statement
const int threshold = 700; //used with IR sensor
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
unsigned long time;

double leftWheel = 0;
double rightWheel = 0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup() {  
  Serial.begin(9600); //set the data rate to 9600 bits per second for serial data transmission
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.; //starting position at origin looking forward
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = sparki.ping();
}

void measure_30cm_speed() {
  Serial.print("Time:");
  time = millis();
  sparki.moveForward(30);//move 30 cm
  Serial.println(millis() - time);
  delay(1000);
}


void updateOdometry() {
  //integrate formula 3.40 ???
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("x pos: ");
  sparki.print(pose_x + "\n");
  sparki.print("y pos: ");
  sparki.print(pose_y + "\n");
  sparki.print("theta: ");
  sparki.print(pose_theta + "\n");
}

void loop() {
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      if (lineLeft < threshold){ 
        sparki.moveLeft();
        //left movement is 2.73 cm/s speed for 0.1 sec; 0.273 cm of movement for each wheel
        leftWheel -= 0.273;
        rightWheel += 0.273;
      }else if (lineRight < threshold) {  
        sparki.moveRight();
        //left movement is 2.73 cm/s speed for 0.1 sec; 0.273 cm of movement for each wheel
        leftWheel += 0.273;
        rightWheel -= 0.273;
      }else if ((lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold)){
        sparki.moveForward();
        leftWheel += 0.273;
        rightWheel += 0.273;
      }
      updateOdometry();
      displayOdometry();
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }
  delay(CYCLE_TIME); //10 
}

#include <sparki.h>

// Set up some global variables with default values to be replaced during operation
int current_state = 0;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int cm_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;


void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance = sparki.ping();
  line_left = 0; // Replace with code to read the left IR sensor
  line_right = 0; // Replace with code to read the right IR sensor
  line_center = 0; // Replace with code to read the center IR sensor
}

void rotate() {
  if(cm_distance != -1){
    if (cm_distance <= 30){
      sparki.moveStop();
      current_state = 1;
    }
  } 
   if(cm_distance > 30){
     sparki.moveLeft();
     sparki.moveRight();
     delay(500);
   }
}

void driveForward(){
  sparki.moveForward();
  delay(500);
  if (cm_distance < 5 {
    sparki.moveStop();
  }
}
/*
void capture(){
  //gripper should start as opened; there are no sensors to indicate if the gripper is open or closed
  sparki.gripperClose();
}

void turnAround(){

}

void followLine() {
  int threshold = 500;
  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor
  if ( lineLeft < threshold ) // if line is below left line sensor
  {  
    sparki.moveLeft(); // turn left
  }
  if ( lineRight < threshold ) // if line is below right line sensor
  {  
    sparki.moveRight(); // turn right
  }
  // if the center line sensor is the only one reading a line
  if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
  {
    sparki.moveForward(); // move forward
  }  
  if( (lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold) )
  {
    //reached start/end point of track (T junction)
    sparki.moveStop();
    current_state = 7;
  }
  sparki.clearLCD(); // wipe the screen
  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);
  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);
  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);
  sparki.updateLCD(); // display all of the information written to the screen
  delay(100); // wait 0.1 seconds
}

void stopBeep(){
  
}
 */
void loop() {
  readSensors();
  
  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);
  sparki.print("DIST: ");
  sparki.println(cm_distance);
  sparki.gripperOpen();
  
  switch (current_state){
  case 0:rotate();
  //case 1:driveForward();
  //case 2:capture();
  //case 3:turnAround();
  //case 4:driveForward();
  //case 5:followLine(); //finished - george
  //case 6:stopBeep();
  }
  
  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}

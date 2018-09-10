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
  line_left = sparki.lineLeft(); // Replace with code to read the left IR sensor
  line_right = sparki.lineRight(); // Replace with code to read the right IR sensor
  line_center = sparki.lineCenter(); // Replace with code to read the center IR sensor
}

void loop() {
  sparki.motorStop(MOTOR_RIGHT);
  sparki.motorStop(MOTOR_LEFT);
  readSensors();
  
  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);
  sparki.print("DIST: ");
  sparki.println(cm_distance);

  switch (current_state){
    case 0:
      rotate();
    case 1:
      driveForward();
    case 2:
      capture();
    case 3:
      driveToTrack();
    case 4:
      followLine();
    //case 5:stopBeep();
  }
  

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}

void rotate() {
  if(current_state == 0){
    if(cm_distance > 0 && cm_distance <= 30){
      sparki.println("Object Detected");
      sparki.moveStop();
      delay(200);
      current_state = 1;
    } else{
     sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
     sparki.motorRotate(MOTOR_LEFT, DIR_CW, 100);
     delay(200);
    }
  }
}

void driveForward(){
  if(current_state == 1){
    if(cm_distance > 7 || cm_distance < 0){
      sparki.moveForward();
      delay(200);
    }else {
      sparki.moveStop();
      delay(500);
      current_state = 2;
    }
  }
}


void capture(){
  if(current_state == 2){
    //gripper should start as opened; there are no sensors to indicate if the gripper is open or closed
    sparki.println("Closing grippers");
    sparki.clearLCD();
    sparki.updateLCD();
    sparki.gripperClose();
    delay(6000);
    
    //turn 180
    sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
    sparki.motorRotate(MOTOR_LEFT, DIR_CW, 100);
    delay(5000);
    current_state = 3;
  }
}

void driveToTrack(){
  if(current_state == 3){ // display all of the information written to the screen
  
    if(sparki.lineCenter() > 500){
       sparki.moveForward();
       delay(200);
    } else {
      //perpandicular case?
      sparki.RGB(RGB_BLUE);
      current_state = 4;
      delay(1000);
    }
  }
}


void followLine() {
  if (current_state == 4) {
    if ( line_left < threshold ) // if line is below left line sensor
    {  
      sparki.moveLeft(); // turn left
    }
    if ( line_right < threshold ) // if line is below right line sensor
    {  
      sparki.moveRight(); // turn right
    }
    // if the center line sensor is the only one reading a line
    if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
    {
      sparki.moveForward(); // move forward
    }  
    if( (line_center < threshold) && (line_left < threshold) && (line_right < threshold) )
    {
      //reached start/end point of track (T junction)
      sparki.moveStop();
      current_state = 7;
      sparki.gripperOpen();
      delay(4000);
      sparki.beep();
      delay(1000);
    }
    sparki.clearLCD(); // wipe the screen
    sparki.print("Line Left: "); // show left line sensor on screen
    sparki.println(line_left);
    sparki.print("Line Center: "); // show center line sensor on screen
    sparki.println(line_center);
    sparki.print("Line Right: "); // show right line sensor on screen
    sparki.println(line_right);
    sparki.updateLCD(); // display all of the information written to the screen
    delay(100); // wait 0.1 seconds
  }
}
/*
void stopBeep(){
  
}
 */

#include <sparki.h>

int current_state = 0;
const int threshold = 700;
int cm_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;


void setup() {
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
  line_left = sparki.lineLeft()
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
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
      followLine();
  }
  
  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
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

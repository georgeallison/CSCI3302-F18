#include <Sparki.h> 

unsigned long timeStart; //global varts for timing
unsigned long timeEnd;
unsigned long totalTime = 0;

long speed = 2.73 //cm per second

int threshold = 500; //global vars for IR sensor
int lineLeft;  
int lineCenter; 
int lineRight;

double xPos = 0; //global vars for odometry
double yPos = 0;
double theta = 0;
double leftWheel;
double rightWheel;
int lastDir;

void setup() 
{
  Serial.begin(9600); //set the data rate to 9600 bits per second for serial data transmission
  lastDir = 0;
}
 
void loop() {
  lineLeft   = sparki.lineLeft();  
  lineCenter = sparki.lineCenter(); 
  lineRight  = sparki.lineRight();
 
  if (lineLeft < threshold){ 
    sparki.moveLeft();
    //left movement is 2.73 cm/s speed for 0.1 sec; 0.273 cm of movement for each wheel
    leftWheel -= 0.273;
    rightWheel += 0.273;
    lastDir = 1;
  }else if (lineRight < threshold) {  
    sparki.moveRight();
    //left movement is 2.73 cm/s speed for 0.1 sec; 0.273 cm of movement for each wheel
    leftWheel += 0.273;
    rightWheel -= 0.273;
    lastDir = 2;
  }else if ((lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold)){
    sparki.moveForward();
    leftWheel += 0.273;
    rightWheel += 0.273;
    lastDir = 3;
  }

  if(lastDir == 1){
      //if last direction was forward
      xPos += 0.273;
      //y pos does not change
      //theta does not change
    } else if (lastDir == 2) {
      /*
        change theta and add distance of rotated movement
      */
    } else if (lastDir == 3) {
      /*
        change theta and add distance of rotated movement
      */
    } else {
      sparki.println("logic error 1");
  }
 
  sparki.clearLCD();
  sparki.print("Line Left: ");
  sparki.println(lineLeft);
 
  sparki.print("Line Center: ");
  sparki.println(lineCenter);
 
  sparki.print("Line Right: "); 
  sparki.println(lineRight);
 
  sparki.updateLCD();
 
  delay(100);
}

#include <Sparki.h> 
unsigned long time;
void setup() 
{
  Serial.begin(9600); //set the data rate to 9600 bits per second for serial data transmission
}
 
void loop() {
  Serial.print("Time: ");
  time = millis(); //milliseconds elapsed since program start
  Serial.println(time);
  
  int threshold = 500;
  int lineLeft   = sparki.lineLeft();  
  int lineCenter = sparki.lineCenter(); 
  int lineRight  = sparki.lineRight(); 
 
  if ( lineLeft < threshold ) 
  {  
    sparki.moveLeft();
  }
 
  if ( lineRight < threshold ) 
  {  
    sparki.moveRight();
  }
 
  
  if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
  {
    sparki.moveForward(); 
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

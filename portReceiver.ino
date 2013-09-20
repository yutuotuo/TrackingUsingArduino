#include <Servo.h>

Servo pan;
Servo tilt;
int panpos = 90;
int tiltpos = 50;

void setup() 
{
  pan.attach(8);
  tilt.attach(9);
  Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
  Serial.println("ready");
 
}

void loop() 
{
 
  if (Serial.available() > 0) 
  {
    int data = Serial.read();	 // read the incoming byte:
    switch(data)
    {
	case 'a' :  if(panpos > 0) {panpos -=2;};  break;
        case 'd' :  if(panpos < 180) {panpos += 2;}; break;
        case 'w' :  if(tiltpos > 0) {tiltpos -=1;}; break;
        case 's' :  if(tiltpos < 180) {tiltpos += 1;};  break;
        default  : break;
    }
    
    Serial.println(panpos);
  pan.write(panpos);
  tilt.write(tiltpos);
  delay(20);
  }
  
}  


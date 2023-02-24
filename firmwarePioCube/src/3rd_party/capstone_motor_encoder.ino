#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(2, 4);
Encoder knobRight(3, 7);


//Movement
bool forward = false;
bool moving = false;

//Encoders
long newLeft, newRight;
long positionLeft  = -999;
long positionRight = -999;

double revTar=0;
String incomingByte = ""; // for incoming serial data
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}



void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.readString();
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte);
  }

  if(incomingByte.substring(0,5).equals("reset")){
      Serial.println("Reset both knobs to zero");
      knobLeft.write(0);
      knobRight.write(0);
      analogWrite(5, 0);
      analogWrite(6, 0);
  }

  if(incomingByte.substring(0,4).equals("stop")){
      analogWrite(5, 0);
      analogWrite(6, 0);
      moving = false;
  }

  if(incomingByte.substring(0,3).equals("for")&&!moving){
      revTar=incomingByte.substring(3).toInt();
      Serial.println(revTar);
      analogWrite(5, 200);
      moving = true;
      forward = true;
  }

    if(incomingByte.substring(0,3).equals("bac")&&!moving){
      revTar=incomingByte.substring(3).toInt();
      Serial.println(revTar);
      analogWrite(6, 200);
      moving = true;
      forward = false;
  }


  
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }

  if(moving&&forward&&positionLeft>=revTar){
    analogWrite(5, 0);
    moving = false;
  }

  if(moving&&!forward&&positionLeft<=revTar){
    analogWrite(6, 0);
    moving = false;
  }

  incomingByte=""; 
}

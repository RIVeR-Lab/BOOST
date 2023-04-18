#include <stdio.h>


int cont0 = 6;
int cont1 = 3;
int cont2 = 4;
int wingcont = 5;

int mod0 = A2;
int mod1 = A3;
int mod2 = A4;
int hubbatt = A5;


void setup() {
Serial.begin(115200);
pinMode(cont0, INPUT_PULLUP);
//pinMode(dtwo, OUTPUT);

//digitalWrite(dtwo, HIGH);
pinMode(cont1, INPUT_PULLUP);
pinMode(cont2, INPUT_PULLUP);
pinMode(wingcont, INPUT_PULLUP);
pinMode(mod0, INPUT);
pinMode(mod1, INPUT);
pinMode(mod2, INPUT);
pinMode(hubbatt, INPUT);
}

void loop() {
 
double mod0V = analogRead(mod0);
double mod1V = analogRead(mod1);
double mod2V = analogRead(mod2);
double hubbatV = analogRead(hubbatt);

mod0V = int(mod0V / 1023.0 * 5.0 * 25.2 * 1000.0);
mod1V = int(mod1V / 1023.0 * 5.0 * 25.2 * 1000.0);
mod2V = int(mod2V / 1023.0 * 5.0 * 25.2 * 1000.0);
hubbatV = int(hubbatV / 1023.0 * 5.0 * 16.8 * 1000.0);

int batt0 = digitalRead(cont0);
int batt1 = digitalRead(cont1);
int batt2 = digitalRead(cont2);
int wing = digitalRead(wingcont);
delay(100);
//static constexpr int BUFFER_SIZE = 512;
//char b[BUFFER_SIZE];
//memset(b, 0, BUFFER_SIZE);

//sprintf(b, ">batt0Cont=%d,batt1Cont=%d,batt2Cont=%d,wingCont=%d,batt0Volt_mv=%d,batt1Volt_mv=%d,batt2Volt_mv=%d,hubBattVolt_mv=%d\r\n",
//batt0, batt1, batt2, wing, mod0V, mod1V, mod2V, hubbatV);
Serial.print(">batt0Cont=");
Serial.print(batt0);
Serial.print(",batt1Cont=");
Serial.print(batt1);
Serial.print(",batt2Cont=");
Serial.print(batt2);
Serial.print(",wingCont=");
Serial.print(wing);

Serial.print(",batt0Volt_mv=");
Serial.print(mod0V);
Serial.print(",batt1Volt_mv=");
Serial.print(mod1V);
Serial.print(",batt2Volt_mv=");
Serial.print(mod2V);
Serial.print(",hubBattVolt_mv=");
Serial.println(hubbatV);
Serial.print("\r\n");
  

  

  
}



 

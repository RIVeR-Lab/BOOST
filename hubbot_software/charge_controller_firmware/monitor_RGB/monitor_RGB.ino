#include <stdio.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN    10

#define LED_COUNT 60

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

long R;
long G;
long B;

int cont0 = 6;
int cont1 = 3;
int cont2 = 4;
int wingcont = 5;

int mod0 = A2;
int mod1 = A3;
int mod2 = A4;
int hubbatt = A5;


void setup() {
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
  strip.show();
  randomSeed(10);
  Serial.begin(9600);
  reset();
  
  pinMode(cont0, INPUT_PULLUP);
  pinMode(cont1, INPUT_PULLUP);
  pinMode(cont2, INPUT_PULLUP);
  pinMode(wingcont, INPUT_PULLUP);
  pinMode(mod0, INPUT);
  pinMode(mod1, INPUT);
  pinMode(mod2, INPUT);
  pinMode(hubbatt, INPUT);
  measure();
}


void measure(){
  double mod0V = analogRead(mod0);
  double mod1V = analogRead(mod1);
  double mod2V = analogRead(mod2);
  double hubbatV = analogRead(hubbatt);
  
  mod0V = int(mod0V / 1023.0 * 5.0 / 3.3 * 25.2 * 1000.0);
  mod1V = int(mod1V / 1023.0 * 5.0 / 3.3 * 25.2 * 1000.0);
  mod2V = int(mod2V / 1023.0 * 5.0 / 3.3 * 25.2 * 1000.0);
  hubbatV = int(hubbatV / 1023.0 * 5.0 / 3.3 * 16.8 * 1000.0);
  
  int batt0 = digitalRead(cont0);
  int batt1 = digitalRead(cont1);
  int batt2 = digitalRead(cont2);
  int wing = digitalRead(wingcont);
  
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

void setLEDState(char TMP, int VAL){
  if(TMP == 'L' || TMP == 'l'){    
    strip.clear();
    strip.setBrightness(255);
    strip.show();
    for (int i = -1; i <= 1; i++){
    strip.setPixelColor(int(VAL/100.0*LED_COUNT) + i, 255, 0, 0);
    strip.show();}
  }

  if(TMP == 'U' || TMP == 'u'){    
    strip.clear();
    strip.show();
    
    strip.setBrightness(255);
    strip.show();
    
    for (int i = 0; i <= 6; i++){
      for(int pix = 0; pix <LED_COUNT; pix++){
        strip.setPixelColor(pix, 0, 0, 255);
        }
        measure();
        strip.show();
        delay(200);
        measure();
        strip.clear();
        strip.show();
        delay(200);}
        measure();
        strip.clear();
        strip.show();
  }

  if(TMP == 'W' || TMP == 'w'){
    strip.clear();
    strip.show();
    strip.setBrightness(150);
    strip.show();
    for(int pix = 0; pix <LED_COUNT; pix++){
        strip.setPixelColor(pix, 0, 255, 120);
        strip.show();
        delay(20);
        measure();
        }
    for(int pix = 0; pix <LED_COUNT; pix++){
      strip.setPixelColor(pix, 0, 0, 0);
      strip.show();
      delay(20);
      measure();
      }
      strip.clear();
      strip.show();
      measure();
      }

  if(TMP == 'P' || TMP == 'p'){    
    strip.clear();
    strip.show();
    
    strip.setBrightness(150);
    measure();
    for (int i = 0; i <= 3; i++){
      measure();
      for(int pix = 0; pix <LED_COUNT; pix++){
        strip.setPixelColor(pix, 0, 255, 0);
        }
        strip.show();
        delay(200);
        strip.clear();
        strip.show();
        delay(100);
        measure();}
      colorWipe(strip.Color(0,   255,   0), 25); 
      strip.clear();
      strip.show();
      delay(200);
      measure();
  }

  if(TMP == 'R' || TMP == 'r'){        
    for(int pix = 0; pix <LED_COUNT; pix++){
        strip.setPixelColor(pix, 0, 255, 0);
        }
    strip.show();
    measure();
    delay(800);
    colorWipe(strip.Color(0,   0,   0), 25); 
    strip.clear();
    strip.show();
    measure();
    delay(200);
      
    for (int i = 0; i <= 3; i++){
      for(int pix = 0; pix <LED_COUNT; pix++){
        strip.setPixelColor(pix, 0, 255, 0);
        }
        strip.show();
        delay(200);
        measure();
        strip.clear();
        strip.show();
        delay(100);
        measure();}
  }     
}

void reset(){
  strip.setBrightness(150);
  strip.show();
  measure();
  for(int pix = 0; pix <LED_COUNT; pix++){
    R = random(0,255);
    G = random(0,255);
    B = random(0,255);
    strip.setPixelColor(pix, 120, 255, B);
    strip.show();
    measure();
  }
}

void readserial(){
  if(Serial.available() > 0){
      char tmp = Serial.read();
      int val = Serial.parseInt();
      String statemessage = "";
      if(tmp == 'L' || tmp == 'l'){
        setLEDState(tmp, val);
        }
      if(tmp == 'Q' || tmp == 'q'){
        strip.clear();
        strip.show();
        measure();
        delay(200);
        measure();
        reset();
        }
      if(tmp == 'U' || tmp == 'u'){
        setLEDState(tmp, val);
        reset();
        }
      if(tmp == 'W' || tmp == 'w'){
        setLEDState(tmp, val);
        reset();
        }
      if(tmp == 'P' || tmp == 'p'){
        setLEDState(tmp, val);
        reset();
        }
      if(tmp == 'R' || tmp == 'r'){
        setLEDState(tmp, val);
        reset();
        }
  }
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();
    measure();//  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}


void loop() {
  readserial();
  measure();
  ;
}

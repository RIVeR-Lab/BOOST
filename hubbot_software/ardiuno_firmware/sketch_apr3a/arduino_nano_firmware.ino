#include <stdio.h>
#include "Arduino.h"

#define BATT0_CONTINUITY_PIN 2
#define BATT1_CONTINUITY_PIN 3
#define BATT2_CONTINUITY_PIN 4
#define WING_CONTINUITY_PIN 5
#define BATT0_VOLT_PIN A2
#define BATT1_VOLT_PIN A3
#define BATT2_VOLT_PIN A4
#define HUB_BATT_VOLT_PIN A5

void setup() {
  Serial.begin(115200);
  pinMode(BATT0_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT1_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT2_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(WING_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT0_VOLT_PIN, INPUT);
  pinMode(BATT1_VOLT_PIN, INPUT);
  pinMode(BATT2_VOLT_PIN, INPUT);
  pinMode(HUB_BATT_VOLT_PIN, INPUT);
}

float myMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int get_voltage_reading(int pin) {
  static constexpr float MAX_VOLTAGE_6S_V = 25.2;
  static constexpr float ADC_MAX_VOLTAGE = 5.0;
  int raw = analogRead(pin);
  raw = myMap(raw, 0.0, 1023.0 * 3.3 / 5.0, 0.0, MAX_VOLTAGE_6S_V);
  // raw = myMap(raw, 0, 3.3, 0, MAX_VOLTAGE_6S_V);
  return (int)(raw * 1000.0);
  // return (analogRead(pin) / 1023.0) * (MAX_VOLTAGE_6S_MV);
}

void loop() {
  // static uint32_t contDataCounter = 0;
  // if ((millis() - contDataCounter) > 100) {
  //   contDataCounter = millis();
    // Get and send all battery coninuity data
  int batt0 = digitalRead(BATT0_CONTINUITY_PIN);
  int batt1 = digitalRead(BATT1_CONTINUITY_PIN);
  int batt2 = digitalRead(BATT2_CONTINUITY_PIN);
  int wing = digitalRead(WING_CONTINUITY_PIN);
  int batt0Volt_mv = get_voltage_reading(BATT0_VOLT_PIN);
  int batt1Volt_mv = get_voltage_reading(BATT1_VOLT_PIN);
  int batt2Volt_mv = get_voltage_reading(BATT2_VOLT_PIN);
  int hubBattVolt_mv = get_voltage_reading(HUB_BATT_VOLT_PIN);

  static constexpr int BUFFER_SIZE = 512;
  char b[BUFFER_SIZE];
  memset(b, 0, BUFFER_SIZE);

  sprintf(b, ">batt0Cont=%d,batt1Cont=%d,batt2Cont=%d,wingCont=%d,batt0Volt_mv=%d,batt1Volt_mv=%d,batt2Volt_mv=%d,hubBattVolt_mv=%d\r\n",
  batt0, batt1, batt2, wing, batt0Volt_mv, batt1Volt_mv, batt2Volt_mv, hubBattVolt_mv);
  Serial.print(b);
  // }
  delay(100);
}

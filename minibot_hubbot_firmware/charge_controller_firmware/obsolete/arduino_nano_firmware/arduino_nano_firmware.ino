#include <stdio.h>
#include "Arduino.h"

#define BATT0_CONTINUITY_PIN 6
#define BATT1_CONTINUITY_PIN 3
#define BATT2_CONTINUITY_PIN 4
#define WING_CONTINUITY_PIN 5
#define BATT0_VOLT_PIN A2
#define BATT1_VOLT_PIN A3
#define BATT2_VOLT_PIN A4
#define HUB_BATT_VOLT_PIN A5

static constexpr float MAX_6S_LIPO_VOLTAGE = (4.2 * 6.0);
static constexpr int32_t VOLT_DIVIDER_R1 = 220000;
static constexpr int32_t VOLT_DIVIDER_R2 = 33000;
static constexpr int32_t ADC_MAX_RESOLUTION = 1024;
static int32_t get_volt_divider_vin_mv(int32_t vout) {
  return (1.0 * vout * (VOLT_DIVIDER_R1 + VOLT_DIVIDER_R2)) / (VOLT_DIVIDER_R2 * 1.0);
}

void setup() {
  Serial.begin(9600);
  pinMode(BATT0_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT1_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT2_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(WING_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT0_VOLT_PIN, INPUT);
  pinMode(BATT1_VOLT_PIN, INPUT);
  pinMode(BATT2_VOLT_PIN, INPUT);
  pinMode(HUB_BATT_VOLT_PIN, INPUT);
}

static int32_t get_voltage_reading_mv(int pin) {
  static constexpr float ADC_MAX_VOLTAGE = 5.0;
  int32_t raw_read = analogRead(pin);
  int32_t voltage_reading_mv = (1.0 * raw_read) / 1024.0 * 5.0 * 1000.0;
  Serial.println(voltage_reading_mv);
  return voltage_reading_mv;
}

void loop() {
  // static uint32_t contDataCounter = 0;
  // if ((millis() - contDataCounter) > 100) {
  //   contDataCounter = millis();
    // Get and send all battery coninuity data
  int8_t batt0Cont = digitalRead(BATT0_CONTINUITY_PIN);
  int8_t batt1Cont = digitalRead(BATT1_CONTINUITY_PIN);
  int8_t batt2Cont = digitalRead(BATT2_CONTINUITY_PIN);
  int8_t wingCont = digitalRead(WING_CONTINUITY_PIN);
  int32_t batt0Volt_mv = get_volt_divider_vin_mv(get_voltage_reading_mv(BATT0_VOLT_PIN));
  int32_t batt1Volt_mv = get_volt_divider_vin_mv(get_voltage_reading_mv(BATT1_VOLT_PIN));
  int32_t batt2Volt_mv = get_volt_divider_vin_mv(get_voltage_reading_mv(BATT2_VOLT_PIN));
  int32_t hubBattVolt_mv = get_volt_divider_vin_mv(get_voltage_reading_mv(HUB_BATT_VOLT_PIN));

  Serial.print(">batt0Cont=");
  Serial.print(batt0Cont);
  Serial.print(",batt1Cont=");
  Serial.print(batt1Cont);
  Serial.print(",batt2Cont=");
  Serial.print(batt2Cont);
  Serial.print(",wingCont=");
  Serial.print(wingCont);

  Serial.print(",batt0Volt_mv=");
  Serial.print(batt0Volt_mv);
  Serial.print(",batt1Volt_mv=");
  Serial.print(batt1Volt_mv);
  Serial.print(",batt2Volt_mv=");
  Serial.print(batt2Volt_mv);
  Serial.print(",hubBattVolt_mv=");
  Serial.println(hubBattVolt_mv);
  Serial.print("\r\n");


  delay(100);
}

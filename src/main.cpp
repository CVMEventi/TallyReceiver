#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define RF69_FREQ_1 865.0
#define RF69_FREQ_2 867.2

#define RF69_DEFAULT_FREQUENCY 865.0
#define RF69_CUSTOM_FREQUENCY 866.2
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4

#define TALLY_LED_RED 5
#define TALLY_LED_GREEN 6
#define BATTERY_PIN A9

#define SERVER_ADDRESS 120

#define TALLY_ON 1
#define TALLY_OFF 0
#define TALLY_TYPE_PROGRAM 0
#define TALLY_TYPE_PREVIEW 1
#define TALLY_NUMBER_PINS {A0, A1, A2, A3}
#define CHANNEL_PIN A3

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rfManager(rf69);

uint8_t dipSwitchPins[] = TALLY_NUMBER_PINS;
uint8_t tallyNumber = 1;
bool useCustomFrequency = false;
unsigned long time;
//unsigned long time2;
bool inProgram = false;
bool inPreview = false;
//bool previewStatus = false;

void readConfiguration() {
  tallyNumber = 1;
  bool value1 = !digitalRead(dipSwitchPins[2]);
  bool value2 = !digitalRead(dipSwitchPins[1]);
  bool value3 = !digitalRead(dipSwitchPins[0]);
  tallyNumber = tallyNumber + value1 * 1 + value2 * 2 + value3 * 4;
  rfManager.setThisAddress(tallyNumber);

  useCustomFrequency = !digitalRead(dipSwitchPins[3]);
}

void sendBatteryStatus() {
  double batteryValue = analogRead(BATTERY_PIN);
  batteryValue *= 2;    // we divided by 2, so multiply back
  batteryValue *= 3.3;  // Multiply by 3.3V, our reference voltage
  batteryValue /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(batteryValue);

  int8_t percentage = ((batteryValue - 3.20) * 100.0);
  uint8_t fixedPercentage = 0;
  if (percentage <= 100 && percentage >= 0) {
    fixedPercentage = percentage;
  } else if (percentage < 0) {
    fixedPercentage = 0;
  } else {
    fixedPercentage = 100;
  }

  Serial.println(percentage);

  Serial.println("Sending battery value");
  rfManager.sendto(&fixedPercentage, sizeof(&fixedPercentage), 120);
}

void ledBootSequence() {
  for (uint8_t index = 0; index < 3; index++) {
    digitalWrite(TALLY_LED_RED, HIGH);
    delay(100);
    digitalWrite(TALLY_LED_RED, LOW);
    delay(200);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(TALLY_LED_RED, OUTPUT);
  pinMode(TALLY_LED_GREEN, OUTPUT);
  ledBootSequence();

  for (uint8_t pin : dipSwitchPins) {
    pinMode(pin, INPUT_PULLUP);
  }
  pinMode(CHANNEL_PIN, INPUT_PULLUP);

  readConfiguration();

  digitalWrite(TALLY_LED_RED, LOW);
  digitalWrite(TALLY_LED_GREEN, LOW);

  // manual reset of RF
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  rfManager.init();

  if (!useCustomFrequency) {
      rf69.setFrequency(RF69_DEFAULT_FREQUENCY);
  } else {
      rf69.setFrequency(RF69_CUSTOM_FREQUENCY);
  }
  rf69.setTxPower(20, true);

  time = 0;

  uint8_t statusRequest = 120;
  sendBatteryStatus();
  rfManager.sendto(&statusRequest, sizeof(&statusRequest), 120);
}

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  int trimmer = analogRead(A10);

  if (rfManager.available()) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rfManager.recvfromAck(buf, &len, &from)) {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.print(buf[1]);
      Serial.print(" type: ");
      Serial.println(buf[0]);

      if (buf[0] == TALLY_TYPE_PROGRAM) {
        inProgram = buf[1];
      } else if (buf[0] == TALLY_TYPE_PREVIEW) {
        inPreview = buf[1];
      }
    }
  }



  analogWrite(TALLY_LED_RED, inProgram ? trimmer / 4 : 0);
  analogWrite(TALLY_LED_GREEN, inPreview ? trimmer / 4 : 0);

  /*if (inProgram) {
    digitalWrite(TALLY_LED_RED, inProgram);
  } else if (inPreview) {
    unsigned long currentTime = millis();
    if (currentTime - time2 > 300) {
      digitalWrite(TALLY_LED_RED, !previewStatus);
      previewStatus = !previewStatus;
      time2 = currentTime;
    }
  } else {
    digitalWrite(TALLY_LED_RED, false);
  }*/

  unsigned long currentTime = millis();

  if (currentTime - time >= 60000) {
    sendBatteryStatus();
    time = currentTime;
  }

  readConfiguration();
}
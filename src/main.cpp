#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#ifndef DEBUG
#define DEBUG 0
#endif

/* Radio Setup */
#define RF69_DEFAULT_FREQUENCY 865.0
#define RF69_CUSTOM_FREQUENCY 866.2
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4

// 16-byte AES key. Must be identical on the transmitter and every receiver.
#define RF69_ENCRYPTION_KEY {0x54, 0x61, 0x6C, 0x6C, 0x79, 0x2D, 0x4B, 0x65, \
                             0x79, 0x2D, 0x43, 0x68, 0x61, 0x6E, 0x67, 0x65}

#define SERVER_ADDRESS 120

/* Radio protocol (must match the transmitter) */
// Transmitter -> broadcast: {MSG_STATE, programMask, previewMask}; bit n = tally n + 1
#define MSG_STATE 'S'
// Receiver -> transmitter (acknowledged): {MSG_HEARTBEAT, batteryPercent}
#define MSG_HEARTBEAT 'B'

#define HEARTBEAT_INTERVAL_MS 5000
#define LINK_TIMEOUT_MS 2000         // transmitter broadcasts every 250 ms
#define CONFIG_POLL_INTERVAL_MS 500

/* Hardware */
#define TALLY_LED_RED 5
#define TALLY_LED_GREEN 6
#define BATTERY_PIN A9
#define BRIGHTNESS_PIN A10
#define TALLY_NUMBER_PINS {A0, A1, A2}
#define CHANNEL_PIN A3

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rfManager(rf69);

const uint8_t dipSwitchPins[] = TALLY_NUMBER_PINS;
uint8_t tallyNumber = 1;
bool useCustomFrequency = false;

uint8_t programMask = 0;
uint8_t previewMask = 0;
bool linkUp = false;
unsigned long lastStateTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastConfigPoll = 0;

int16_t currentRed = -1;
int16_t currentGreen = -1;

uint8_t readTallyNumber() {
  return 1 + !digitalRead(dipSwitchPins[2]) * 1
           + !digitalRead(dipSwitchPins[1]) * 2
           + !digitalRead(dipSwitchPins[0]) * 4;
}

void applyFrequency() {
  rf69.setFrequency(useCustomFrequency ? RF69_CUSTOM_FREQUENCY : RF69_DEFAULT_FREQUENCY);
}

void pollConfiguration() {
  unsigned long now = millis();
  if (now - lastConfigPoll < CONFIG_POLL_INTERVAL_MS) {
    return;
  }
  lastConfigPoll = now;

  uint8_t newTallyNumber = readTallyNumber();
  if (newTallyNumber != tallyNumber) {
    // The LEDs pick up the new tally's state from the cached masks immediately.
    tallyNumber = newTallyNumber;
    rfManager.setThisAddress(tallyNumber);
  }

  bool newUseCustomFrequency = digitalRead(CHANNEL_PIN) == LOW;
  if (newUseCustomFrequency != useCustomFrequency) {
    useCustomFrequency = newUseCustomFrequency;
    applyFrequency();
  }
}

uint8_t readBatteryPercentage() {
  float voltage = analogRead(BATTERY_PIN);
  voltage *= 2;     // we divided by 2, so multiply back
  voltage *= 3.3;   // Multiply by 3.3V, our reference voltage
  voltage /= 1024;  // convert to voltage

  int16_t percentage = (voltage - 3.20) * 100.0;

#if DEBUG
  Serial.print(F("VBat: "));
  Serial.print(voltage);
  Serial.print(F(" -> "));
  Serial.println(percentage);
#endif

  return constrain(percentage, 0, 100);
}

void sendHeartbeat() {
  uint8_t data[2] = {MSG_HEARTBEAT, readBatteryPercentage()};
  bool acked = rfManager.sendtoWait(data, sizeof(data), SERVER_ADDRESS);
  lastHeartbeatTime = millis();

#if DEBUG
  Serial.println(acked ? F("Heartbeat acked") : F("Heartbeat not acked"));
#else
  (void)acked;
#endif
}

void handleIncoming() {
  if (!rfManager.available()) {
    return;
  }

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from, to, id, flags;

  // Plain receive: state broadcasts are never acknowledged, and recvfromAck's duplicate
  // filter would drop the repeated broadcasts (they share a header id).
  if (!rfManager.recvfrom(buf, &len, &from, &to, &id, &flags)) {
    return;
  }

  if (flags & RH_FLAGS_ACK) {
    return;  // late ACK for one of our heartbeats
  }

  if (from != SERVER_ADDRESS || to != RH_BROADCAST_ADDRESS || len < 3 || buf[0] != MSG_STATE) {
    return;
  }

  programMask = buf[1];
  previewMask = buf[2];
  lastStateTime = millis();
  linkUp = true;

#if DEBUG
  Serial.print(F("State P:"));
  Serial.print(programMask, BIN);
  Serial.print(F(" W:"));
  Serial.println(previewMask, BIN);
#endif
}

void setLeds(uint8_t red, uint8_t green) {
  if (red != currentRed) {
    analogWrite(TALLY_LED_RED, red);
    currentRed = red;
  }
  if (green != currentGreen) {
    analogWrite(TALLY_LED_GREEN, green);
    currentGreen = green;
  }
}

void updateLeds() {
  unsigned long now = millis();
  uint8_t brightness = analogRead(BRIGHTNESS_PIN) / 4;

  if (linkUp && now - lastStateTime > LINK_TIMEOUT_MS) {
    linkUp = false;
  }

  if (!linkUp) {
    // No transmitter: short green blink once per second instead of a stale tally state.
    setLeds(0, (now % 1000) < 100 ? brightness : 0);
    return;
  }

  uint8_t bit = 1 << (tallyNumber - 1);
  setLeds(programMask & bit ? brightness : 0, previewMask & bit ? brightness : 0);
}

void ledBootSequence() {
  for (uint8_t index = 0; index < 3; index++) {
    digitalWrite(TALLY_LED_RED, HIGH);
    delay(100);
    digitalWrite(TALLY_LED_RED, LOW);
    delay(200);
  }
}

void radioFailure() {
  // Fast red blink forever: the radio did not initialise.
  while (true) {
    digitalWrite(TALLY_LED_RED, HIGH);
    delay(50);
    digitalWrite(TALLY_LED_RED, LOW);
    delay(50);
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

  tallyNumber = readTallyNumber();
  useCustomFrequency = digitalRead(CHANNEL_PIN) == LOW;
  rfManager.setThisAddress(tallyNumber);

  // Manual reset of the radio (RST is active high)
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rfManager.init()) {
    radioFailure();
  }

  applyFrequency();
  rf69.setTxPower(20, true);

  uint8_t key[] = RF69_ENCRYPTION_KEY;
  rf69.setEncryptionKey(key);

  // Keep the loop responsive if the transmitter is missing: worst case ~4 x 60 ms.
  rfManager.setTimeout(30);
  rfManager.setRetries(3);

  sendHeartbeat();
}

void loop() {
  handleIncoming();
  updateLeds();

  if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    sendHeartbeat();
  }

  pollConfiguration();
}

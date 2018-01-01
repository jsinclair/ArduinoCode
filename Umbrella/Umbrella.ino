
#include <EEPROM.h>

// Threshold struct to manage the thresholds which need to accound for variance on either side after triggering (AKA kind of hysteresis)
struct AnalogThreshold {
    float threshold; // The actual threshold value, in volts: nnV
    float play; // The possible variance on either side of the threshold
    bool inPlay; // True directly after the threshold is passed, until the difference exceeds the play in either direction.
    float lastVoltage; // The last check value. This will always be updated until a change is triggered, at which point it will only start updating again once it exceeds the play range.
};

void thresholdCheck(AnalogThreshold* analogThreshold, float analogVoltage);

// The various thresholds. The first number is the actual threshold number, the second is the play on either side, the last two parameters should always be false and -1.0;
AnalogThreshold solarVoltageThreshold = {2.5, 0.3, false, -1.0}; // Threshold for solar power
AnalogThreshold batteryVoltageLimit1 = {4.0, 0.3, false, -1.0}; // The first limit for the battery, disables USB charger when battery level is lower than this
AnalogThreshold batteryVoltageLimit2 = {2.5, 0.3, false, -1.0}; // The second limit for the battery, disables light and limits umbrella to closing when battery level is lower than this
AnalogThreshold batteryVoltageLimit3 = {1.0, 0.3, false, -1.0}; // The third limit for the battery, lights up D10 when battery level is lower than this
AnalogThreshold dayNightVoltageThreshold = {2.5, 0.3, false, -1.0}; // The third limit for the battery, lights up D10 when battery level is lower than this

const float openingAmpLimit = 10.0; // the nnA for the opening current limit
const float closingAmpLimit = 2.5; // the nnA for the closing current limit
unsigned long debounceDelay = 200; // the button debounce time, in milliseconds
unsigned long currentMonitorDelay = 500; // Current monitor delay for opening and closing, in milliseconds

// Solar constants
const int solarAnalogInPin = A6;  // Analog input pin that the potentiometer is attached to
const int solarLedOutPin = 9; // Digital output pin that the LED is attached to

// Battery and USB constants
const int batteryAnalogInPin = A5;  // Analog input pin that the potentiometer is attached to
const int batteryLedOutPin = 10; // Digital output pin that the LED is attached to
const int usbChargerLedOutPin = 11; // Digital output pin that the USB LED is attached to

// Opening and Closing constants and variables
const int upDownButtonPin = 2;     // the number of the upDownButton pin
const int currentAnalogInPin = A4;  // Analog input pin that the potentiometer is attached to
const int openingLedOutPin = 6; // Digital output pin that the LED is attached to
const int closingLedOutPin = 7; // Digital output pin that the LED is attached to
const float openingVoltThreshold = (0.5 + (openingAmpLimit * 0.13)); // the voltage for the opening current limit
const float closingVoltThreshold = (0.5 + (closingAmpLimit * 0.13)); // the voltage for the closing current limit
// Opening and closing state constants
const int OPEN = 0;
const int OPENING = 1;
const int CLOSED = 2;
const int CLOSING = 3;
const int MANUAL_OPEN = 4;
// Opening and closing variables
int upDownButtonState;         // variable for reading the upDownButton status
int upDownState = CLOSED;
int lastUpDownButtonState = LOW;

// Lights Pins
const int dayNightAnalog = A3;     // the number of the Day/Night pin
const int lightButtonPin = 3;     // the number of the lightButton pin
const int lightsOutPin = 8;      // the LED pin for lights
int lightButtonState;         // variable for reading the upDownButton status
int lastLightButtonState = LOW;
int lightState = LOW;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastUmbrellaDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastLightDebounceTime = 0;  // the last time the light button was pressed
unsigned long currentMonitorDelayStartTime = 0; // The start time of a current monitor delay

int solarValue = 0;        // value read from the solar analog
int batteryValue = 0;        // value read from the battery analog
int currentValue = 0;        // value read from the current monitor analog

int stateAddress = 0;

void setup() {
  // initialize lights pins
  pinMode(lightButtonPin, INPUT);
  pinMode(lightsOutPin, OUTPUT);
  // initialize light states
  digitalWrite(lightsOutPin, lightState);
  
  // initialize the LED pin as an output:
  pinMode(openingLedOutPin, OUTPUT);
  pinMode(closingLedOutPin, OUTPUT);
  pinMode(upDownButtonPin, INPUT);
  // set opening and closing states
  digitalWrite(openingLedOutPin, LOW);
  digitalWrite(closingLedOutPin, LOW);

  // initialize the battery, solar and usb output pins:
  pinMode(solarLedOutPin, OUTPUT);
  pinMode(batteryLedOutPin, OUTPUT);
  pinMode(usbChargerLedOutPin, OUTPUT);

  Serial.begin(9600);

  // Configure the initial umbrella state, reading from the EEPROM and setting it to closed if an invalid value is read.
  upDownState = EEPROM.read(stateAddress);

  if (upDownState < 0 || upDownState > MANUAL_OPEN) {
    upDownState = CLOSED;
  }
}

void loop() {

  // SOLAR MONITOR
  // read the analog in value:
  solarValue = analogRead(solarAnalogInPin);
  // Convert the raw data value to voltage and do the solar voltage threshold check
  thresholdCheck(&solarVoltageThreshold, analogToVoltage(solarValue));
  // write to the solar led based on the voltage and threshold:
  digitalWrite(solarLedOutPin, solarVoltageThreshold.lastVoltage >= solarVoltageThreshold.threshold ? HIGH : LOW);

  // BATTERY MONITOR
  // read the analog in value:
  batteryValue = analogRead(batteryAnalogInPin);
  // Convert the raw data value to voltage:
  float batteryVoltage = analogToVoltage(batteryValue);
  // write to the battery led based on the voltage and threshold:
  thresholdCheck(&batteryVoltageLimit3, batteryVoltage);
  digitalWrite(batteryLedOutPin, batteryVoltageLimit3.lastVoltage < batteryVoltageLimit3.threshold ? HIGH : LOW);

  // DAY/NIGHT MONITOR
  // read the analog in value:
  int dayNightValue = analogRead(dayNightAnalog);
  // Convert the raw data value to voltage, and decide day or night based on the result
  Serial.println(analogToVoltage(dayNightValue));
  thresholdCheck(&dayNightVoltageThreshold, analogToVoltage(dayNightValue));
  bool isDay = dayNightVoltageThreshold.lastVoltage > dayNightVoltageThreshold.threshold;

  // Handle USB Charger
  thresholdCheck(&batteryVoltageLimit1, batteryVoltage);
  digitalWrite(usbChargerLedOutPin, batteryVoltageLimit1.lastVoltage >= batteryVoltageLimit1.threshold);

  // Handle Lights
  // read the state of the light switch into a local variable:
  int lightSwitchRead = digitalRead(lightButtonPin);

  if (lightSwitchRead != lastLightButtonState && (millis() - lastLightDebounceTime) > debounceDelay) {
    lightButtonState = lightSwitchRead;

    // LOW to trigger on release, HIGH to trigger on press
    if (lightButtonState == LOW) {
      lastLightDebounceTime = millis();
      
      lightState = !lightState;
    }
  }
  lastLightButtonState = lightSwitchRead;

  thresholdCheck(&batteryVoltageLimit2, batteryVoltage);
  digitalWrite(lightsOutPin, batteryVoltageLimit2.lastVoltage < batteryVoltageLimit2.threshold ? LOW : (isDay ? LOW : lightState));

  // OPENING AND CLOSING UMBRELLA
  int upDownReading = digitalRead(upDownButtonPin);

  if (upDownReading != lastUpDownButtonState && (millis() - lastUmbrellaDebounceTime) > debounceDelay) {
    upDownButtonState = upDownReading;

    // LOW to trigger on release, HIGH to trigger on press
    if (upDownButtonState == HIGH) {
      lastUmbrellaDebounceTime = millis();
      currentMonitorDelayStartTime = millis();

      switch (upDownState) {
        case OPEN:
        case MANUAL_OPEN:
          upDownState = CLOSING;
          break;
        case OPENING:
          upDownState = MANUAL_OPEN;
          break;
        case CLOSED:
          // This depends on the battery level. If its too low, it must only close.
          upDownState = batteryVoltageLimit2.lastVoltage < batteryVoltageLimit2.threshold ? CLOSING : OPENING;
          break;
        case CLOSING:
          upDownState = CLOSED;
          break;
      }
    }
  }

  lastUpDownButtonState = upDownReading;

  // Adjust state based on battery level
  if (batteryVoltageLimit2.lastVoltage < batteryVoltageLimit2.threshold && (upDownState == OPENING)) {
    // Setting the state to OPEN will make it only update to CLOSING on a button press, which is the limitation when the battery is low.
    upDownState = OPEN;
  }

  // Handle opening and closing based on state, after the current monitor delay period
  if (millis() > (currentMonitorDelayStartTime + currentMonitorDelay) && (upDownState == OPENING || upDownState == CLOSING)) {
    currentValue = analogRead(currentAnalogInPin);
    // Convert the raw data value (0 - 1023) to voltage (0.0V - 5.0V):
    float currentVoltage = currentValue * (5.0 / 1024.0);

    if (upDownState == OPENING) {
      if (currentVoltage >= openingVoltThreshold) {
        upDownState = OPEN;
      }
    } else if (upDownState == CLOSING) {
      if (currentVoltage >= closingVoltThreshold) {
        upDownState = CLOSED;
      }
    }
  }

  // set the LED:
  digitalWrite(openingLedOutPin, upDownState == OPENING);
  digitalWrite(closingLedOutPin, upDownState == CLOSING);

  // Save the umbrella state
  EEPROM.update(stateAddress, upDownState);
}

// Convert the raw data value (0 - 1023) to voltage (0.0V - 5.0V):
float analogToVoltage(int analogReadValue) {
  return analogReadValue * (5.0 / 1024.0);
}

// Checks whether or not the new voltage should trigger a change in state
void thresholdCheck(AnalogThreshold* analogThreshold, float analogVoltage) {
  if (analogThreshold->inPlay) {
    if (abs((analogVoltage - analogThreshold->threshold)) > analogThreshold->play) {
      analogThreshold->inPlay = false;
    }
  }

  if (analogThreshold->lastVoltage == -1) {
    analogThreshold->lastVoltage = analogVoltage;
  } else if (!analogThreshold->inPlay) {
    if ((analogThreshold->lastVoltage >= analogThreshold->threshold && analogVoltage < analogThreshold->threshold) ||
      (analogThreshold->lastVoltage < analogThreshold->threshold && analogVoltage >= analogThreshold->threshold)) {
      analogThreshold->inPlay = true;
    }
    
    analogThreshold->lastVoltage = analogVoltage;
  }
}


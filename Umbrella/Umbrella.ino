
#include <EEPROM.h>

// Float constants for battery thresholds. All values in volts.
const float usbOnValue = 3.15; // Value at which the usb turns on
const float usbOffValue = 2.8; // Value at which the usb turns off
const float batteryLimit2OffValue = 2.6; // Second battery threshold, turns off lights and only allows the umbrella to close when its off.
const float batteryLimit2OnValue = 3.0;
const float dayVoltage = 2.8; // If the day/night sensor reads this value or more, the system behaves for day time.
const float nightVoltage= 2.5; // If the day/night sensor reads this value or less, the system behaves for night time.
// Constants for opening and closing the umbrella
const float openingAmpLimit = 9.0; // the nnA for the opening current limit
const float closingAmpLimit = 3.0; // the nnA for the closing current limit
const long closedDebounceDuration = 1000; // The time in milliseconds that the motor reverses for after it finishes opening.
const long closedDebouncePauseDuration = 500; // The time in milliseconds that the system waits before reversing, after opening.
const long motorResistorDuration = 500; // Duration the resistor runs for while the motor is starting up.
const long remoteReceiverOnDuration = 1500; // How long in milliseconds the remote receiver stays on for in a cycle.
const long remoteReceiverOffDuration = 5000; // How long in milliseconds the remote receiver stays off for in a cycle.

// Hysteresis struct to manage the various thresholds
struct AnalogHysteresis {
    float offValue; // The value at which the hysteresis turns off, in volts: nnV
    float onValue; // The value at which the hysteresis turns on, in volts: nnV
    bool isOn; // Whether the value we are tracking is on or not
};

void hysteresisCheck(AnalogHysteresis* analogHysteresis, float analogVoltage);

// The various thresholds. The first number is the off value, the second is the on value. The third value is the default state of the check.
AnalogHysteresis batteryVoltageLimit1 = {usbOffValue, usbOnValue, true}; // The first limit for the battery, disables USB charger when battery level is lower than this
AnalogHysteresis batteryVoltageLimit2 = {batteryLimit2OffValue, batteryLimit2OnValue, true}; // The second limit for the battery, disables light and limits umbrella to closing when battery level is lower than this
AnalogHysteresis dayNightVoltageThreshold = {nightVoltage, dayVoltage, true}; // The Day/Night threshold. If the voltage from the light sensor is above this, it is considered day.

const float voltageLimit = 3.3; // change this for the different arduinos, 3.3 for mini, 5.0 for nano
const float motorVoltageMinimum = 1.9; // This is used as the minimum voltage when reading motor current, to compensate for the base amount of 1.9 volts.
const float motorVoltageVariance = voltageLimit - motorVoltageMinimum;
const float motorCurrentMaximum = 10.0; // The maximum amount of amps that the motor can exert.

// Battery and USB constants
const int batteryAnalogInPin = A2;  // Analog input pin that the potentiometer is attached to
const int usbChargerLedOutPin = 11; // Digital output pin that the USB LED is attached to

// Opening and Closing constants and variables
const int motorResistorPin = 9;
const int upDownButtonPin = 2;     // the number of the upDownButton pin
const int currentAnalogInPin = A1;  // Analog input pin that the potentiometer is attached to
const int openingLedOutPin = 6; // Digital output pin that the LED is attached to
const int closingLedOutPin = 7; // Digital output pin that the LED is attached to
const float openingVoltThreshold = ((openingAmpLimit / motorCurrentMaximum) * motorVoltageVariance); // the voltage for the opening current limit
const float closingVoltThreshold = ((closingAmpLimit / motorCurrentMaximum) * motorVoltageVariance); // the voltage for the closing current limit
// Opening and closing state constants
const int OPEN = 0;
const int OPENING = 1;
const int CLOSED = 2;
const int CLOSING = 3;
const int MANUAL_OPEN = 4;
const int CLOSED_DEBOUNCE = 5; // This is used to reverse the motor a little bit after the umbrella finishes opening
const int CLOSED_DEBOUNCE_PAUSE = 6; // This is used to give the motor a moment to stop before the debounce
// Opening and closing variables
int upDownButtonState;         // variable for reading the upDownButton status
int upDownState = CLOSED;
int lastUpDownButtonState = LOW;
// Closed debounce constants and variables
long closedDebounceRunTime = 0;
long closedDebouncePauseRunTime = 0;
unsigned long debounceDelay = 200; // the button debounce time, in milliseconds
unsigned long currentMonitorDelay = 500; // Current monitor delay for opening and closing, in milliseconds
long motorResistorRunTime = -motorResistorDuration;

// Lights Pins
const int dayNightAnalog = A0;     // the number of the Day/Night pin
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

// Remote receiver cycle variables.
int remoteReceiverState = HIGH;
long remoteReceiverStateDuration = 0;
const int remoteReceiverPin = 10;      // output pin for the remote receiver state

int batteryValue = 0;        // value read from the battery analog
int currentValue = 0;        // value read from the current monitor analog

int stateAddress = 0;

void setup() {
  // Get the millis time for the setup.
  const long setupMillis = millis();
  
  // initialize lights pins
  pinMode(lightButtonPin, INPUT);
  pinMode(lightsOutPin, OUTPUT);
  // initialize light states
  digitalWrite(lightsOutPin, lightState);
  
  // initialize the LED pin as an output:
  pinMode(openingLedOutPin, OUTPUT);
  pinMode(closingLedOutPin, OUTPUT);
  pinMode(upDownButtonPin, INPUT);
  pinMode(motorResistorPin, OUTPUT);
  pinMode(remoteReceiverPin, OUTPUT);
  // set opening and closing states
  digitalWrite(openingLedOutPin, LOW);
  digitalWrite(closingLedOutPin, LOW);
  digitalWrite(motorResistorPin, LOW);
  digitalWrite(remoteReceiverPin, remoteReceiverState);

  // initialize the usb output pin
  pinMode(usbChargerLedOutPin, OUTPUT);

  Serial.begin(9600);

  // Configure the initial umbrella state, reading from the EEPROM and setting it to closed if an invalid value is read.
  upDownState = EEPROM.read(stateAddress);

  if (upDownState < 0 || upDownState > CLOSED_DEBOUNCE_PAUSE) {
    upDownState = CLOSED;
  } else if (upDownState == CLOSED_DEBOUNCE || upDownState == CLOSED_DEBOUNCE_PAUSE) {
    upDownState = CLOSED_DEBOUNCE;
    // Run the closed debounce if the umbrella was busy with it when it turned off.
    closedDebounceRunTime = setupMillis;
  }

  remoteReceiverStateDuration = setupMillis;
}

void loop() {

  // Get the millis time for this loop.
  const long loopMillis = millis();

  // BATTERY MONITOR
  // read the analog in value:
  batteryValue = analogRead(batteryAnalogInPin);
  // Convert the raw data value to voltage:
  float batteryVoltage = analogToVoltage(batteryValue);
  
  // DAY/NIGHT MONITOR
  // read the analog in value:
  int dayNightValue = analogRead(dayNightAnalog);
  // Convert the raw data value to voltage, and decide day or night based on the result
  hysteresisCheck(&dayNightVoltageThreshold, analogToVoltage(dayNightValue));
  bool isDay = dayNightVoltageThreshold.isOn;

  if (isDay) {
    // When its day. set the light state to LOW, so they dont automatically turn on the next day.
    lightState = LOW;
  }

  // Handle USB Charger
  hysteresisCheck(&batteryVoltageLimit1, batteryVoltage);
  digitalWrite(usbChargerLedOutPin, batteryVoltageLimit1.isOn);

  // Handle Lights
  // read the state of the light switch into a local variable:
  int lightSwitchRead = digitalRead(lightButtonPin);

  if (lightSwitchRead != lastLightButtonState && (loopMillis - lastLightDebounceTime) > debounceDelay) {
    lightButtonState = lightSwitchRead;

    // LOW to trigger on release, HIGH to trigger on press
    if (lightButtonState == LOW) {
      lastLightDebounceTime = loopMillis;
      
      lightState = !lightState;
    }
  }
  lastLightButtonState = lightSwitchRead;

  // Check the battery level
  hysteresisCheck(&batteryVoltageLimit2, batteryVoltage);

  // OPENING AND CLOSING UMBRELLA
  int upDownReading = digitalRead(upDownButtonPin);

  if ((upDownState != CLOSED_DEBOUNCE && upDownState != CLOSED_DEBOUNCE_PAUSE) && upDownReading != lastUpDownButtonState && (loopMillis - lastUmbrellaDebounceTime) > debounceDelay) {
    upDownButtonState = upDownReading;

    // LOW to trigger on release, HIGH to trigger on press
    if (upDownButtonState == HIGH) {
      lastUmbrellaDebounceTime = loopMillis;
      currentMonitorDelayStartTime = loopMillis;

      switch (upDownState) {
        case OPEN:
        case MANUAL_OPEN:
          upDownState = CLOSING;
          motorResistorRunTime = loopMillis;
          break;
        case OPENING:
          upDownState = MANUAL_OPEN;
          break;
        case CLOSED:
          // This depends on the battery level. If its too low, it must only close.
          upDownState = !batteryVoltageLimit2.isOn ? CLOSING : OPENING;
          motorResistorRunTime = loopMillis;
          break;
        case CLOSING:
          upDownState = CLOSED;
          break;
      }
    }
  }

  lastUpDownButtonState = upDownReading;

  // Adjust state based on battery level
  if (!batteryVoltageLimit2.isOn && (upDownState == OPENING)) {
    // Setting the state to OPEN will make it only update to CLOSING on a button press, which is the limitation when the battery is low.
    upDownState = OPEN;
  }

  // Handle opening and closing based on state, after the current monitor delay period
  if (loopMillis > (currentMonitorDelayStartTime + currentMonitorDelay) && (upDownState == OPENING || upDownState == CLOSING)) {
    currentValue = analogRead(currentAnalogInPin);
    // Convert the raw data value (0 - 1023) to voltage (0.0V - voltageLimitV):
    float currentVoltage = (currentValue * (voltageLimit / 1024.0)) - motorVoltageMinimum;
    
    if (upDownState == OPENING) {
      if (currentVoltage >= openingVoltThreshold) {
        upDownState = OPEN;
      }
    } else if (upDownState == CLOSING) {
      if (currentVoltage >= closingVoltThreshold) {
        // Initialise the debounce to relieve stress on the closing spring.
        upDownState = CLOSED_DEBOUNCE_PAUSE;
        closedDebouncePauseRunTime = loopMillis;
      }
    }
  } else if (upDownState == CLOSED_DEBOUNCE && loopMillis - closedDebounceRunTime > closedDebounceDuration) {
    // If the debounce has run its course, set the state to closed
    upDownState = CLOSED;
  } else if (upDownState == CLOSED_DEBOUNCE_PAUSE && loopMillis - closedDebouncePauseRunTime > closedDebouncePauseDuration) {
    // If the debounce has run its course, set the state to closed
    upDownState = CLOSED_DEBOUNCE;
    closedDebounceRunTime = loopMillis;
  }
  
  // If the upbrella state isnt open, turn off the lights
  if (upDownState != OPEN) {
    lightState = LOW;
  }

  // Work out the remote receiver state
  if (remoteReceiverState && (remoteReceiverStateDuration + remoteReceiverOnDuration < loopMillis)) {
    remoteReceiverState = LOW;
    remoteReceiverStateDuration = loopMillis;
  } else if (!remoteReceiverState && (remoteReceiverStateDuration + remoteReceiverOffDuration < loopMillis)) {
    remoteReceiverState = HIGH;
    remoteReceiverStateDuration = loopMillis;
  }

  // Set the resistor state
  digitalWrite(motorResistorPin, loopMillis - motorResistorRunTime < motorResistorDuration);

  // Set the light state
  digitalWrite(lightsOutPin, !batteryVoltageLimit2.isOn ? LOW : (isDay ? LOW : lightState));

  // set opening and closing the LEDs:
  digitalWrite(openingLedOutPin, (upDownState == OPENING || upDownState == CLOSED_DEBOUNCE));
  digitalWrite(closingLedOutPin, upDownState == CLOSING);

  // set the remote receiver state
  digitalWrite(remoteReceiverPin, remoteReceiverState);

  // Save the umbrella state
  EEPROM.update(stateAddress, upDownState);
}

// Convert the raw data value (0 - 1023) to voltage (0.0V - voltageLimitV):
float analogToVoltage(int analogReadValue) {
  return analogReadValue * (voltageLimit / 1024.0);
}

// Checks whether or not the new voltage should trigger a change in state
void hysteresisCheck(AnalogHysteresis* analogHysteresis, float analogVoltage) {
  if (analogHysteresis->offValue >= analogVoltage) {
    analogHysteresis->isOn = false;
  } else if (analogHysteresis->onValue <= analogVoltage) {
    analogHysteresis->isOn = true;
  }
}


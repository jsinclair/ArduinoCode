#include <EEPROM.h>

// Float constants for battery thresholds. All values in volts.
const float batteryMotorLimitOffValue = 2.8; // Battery threshold for the motor, the umbrella can only close when its off.
const float batteryMotorLimitOnValue = 3.0;
const float batteryLightLimitOffValue = 2.75; // Battery threshold for the lights.
const float batteryLightLimitOnValue = 2.9;
// Constants for opening and closing the umbrella
float openingAmpLimit = 7.0; // the nnA for the opening current limit. Allowable range: 0.0 - maxCurrentAmps
const float voltageLimit = 3.3; // change this for the different arduinos, 3.3 for mini, 5.0 for nano
const float motorVoltageMinimum = 1.25; // This is used as the minimum voltage when reading motor current, to compensate for the base amount of 2.5 volts.
const float maxCurrentAmps = 15;
const unsigned long currentMonitorDelay = 600; // Current monitor delay for opening and closing, in milliseconds
const unsigned long startPulseDetectorDelay = 300; // Delay before starting the motor, when we power on the pulse detector out
const unsigned long stopPulseDetectorDelay = 1000; // Delay after stopping the motor, when we power off the pulse detector out
const int pulsesToListenForClosed = 10; // When closing and the pulse count reaches this, listen to the 

float openingVoltLimit; // Calculated in setup: (openingAmpLimit * 0.05) + motorVoltageMinimum

// Hysteresis struct to manage the various thresholds
struct AnalogHysteresis {
    float offValue; // The value at which the hysteresis turns off, in volts: nnV
    float onValue; // The value at which the hysteresis turns on, in volts: nnV
    bool isOn; // Whether the value we are tracking is on or not
};

void hysteresisCheck(AnalogHysteresis* analogHysteresis, float analogVoltage);

// The various thresholds. The first number is the off value, the second is the on value. The third value is the default state of the check.
AnalogHysteresis motorVoltageLimit = {batteryMotorLimitOffValue, batteryMotorLimitOnValue, true}; // The limit for the battery, limits umbrella to closing when battery level is lower than this
AnalogHysteresis lightsVoltageLimit = {batteryLightLimitOffValue, batteryLightLimitOnValue, true}; // The limit for the battery, disables lights when battery level is lower than this

// Motor consts and vars
const int motorButtonPin = 2;		// the number of the motorButton pin
const int motorUpPin = 6;			// Output pin for up motor
const int motorDownPin = 7;		// Output pin for down motor

// Up/Down states
const int OPENING = 0;
const int OPEN = 1;
const int CLOSING = 2;
const int CLOSED = 3;
const int OPEN_PARTIAL = 4;
const int CLOSED_PARTIAL = 5;

int lastMotorButtonState = LOW;
int motorButtonState;
int motorState = CLOSED;
unsigned long lastMotorDebounceTime = 0;  // the last time the output pin was toggled
unsigned long currentMonitorDelayStartTime = 0; // The start time of a current monitor delay

// Light consts and vars
const int lightButtonPin = 3;     	// Light button in pin
const int lightOutPin = 8;			// Light out pin

int lastLightButtonState = LOW;
int lightButtonState;
int lightState = LOW;
unsigned long lastLightDebounceTime = 0;  // the last time the output pin was toggled

// Battery consts and vars
const int batteryInPin = A2;     	// Battery in in pin

// Motor current consts and vars
const int motorCurrentPin = A1;

// Pulse detector consts and vars
unsigned long pulseDetectorStartTime = 0;
unsigned long pulseDetectorDelay = 0;
const int pulseDetectorPin = 10;

// Pulse consts and vars
const int pulsePin = 9;			// The pin we listen for pulses on
int pulseCount = 0;
unsigned long lastPulseDebounceTime = 0;  // the last time the output pin was toggled
int pulseState;
int lastPulseState;

const int closedPulsePin = 11;			// When closing, listen to this when pulses from below pulsesToListenForClosed
unsigned long lastClosedPulseDebounceTime = 0;  // the last time the output pin was toggled
int closedPulseState;
int lastClosedPulseState;

// Reset consts and vars
const int pulseResetPin = 5;	// The pin which can reset pulses
int lastResetButtonState = LOW;
int resetButtonState;
unsigned long lastResetDebounceTime = 0;  // the last time the output pin was toggled

// Activity indicator consts and vars
const int activityPin = 4;

// Other consts and vars
const unsigned long debounceDelay = 50;    // the standard button debounce time
const unsigned long resetDebounceDelay = 1000;    // the reset button debounce time
const unsigned long pulseDebounceDelay = 10;    // the debounce time between pulse registers
const unsigned long powerDebounceDelay = 10;  // delay on powerPin before saving

bool firstLoop = true;

// EEPROM Stuff
const int powerPin = 12;			// The pin we listen on for powering off. We must only save to EEPROM when the power goes down.
unsigned long lastPowerDebounceTime = 0;
int lastPowerPinState = HIGH;
int powerPinState = HIGH;
const int motorStateAddress = 0;
int firstRunAddress;
int pulseCountAddress;

void setup() {
  setupPins();
  
  //Serial.begin(9600);
  
  setInitialMotorState();
  
  setInitialPulseCount();

  setOpeningLimits();
}

void loop() {
  // Get the millis time for this loop.
  const unsigned long loopMillis = millis();

  // BATTERY MONITOR
  updateBatteryLevel();
  
  // Handle lights input
  handleLights(loopMillis);
  
  // Handle pulse input
  handlePulses(loopMillis);

  // Read in the state of the closedPulsPin
  readClosedPulse(loopMillis);
  
  // Handle motor input
  handleMotorButtonInput(loopMillis);
  
  // Update up/down state based on motor state and input readings
  handleMotorLogic(loopMillis);

  // Handle reset input
  handleResetButtonInput(loopMillis);
  
  // OUTPUTS
  
  // set the light output
  digitalWrite(lightOutPin, lightState);

  // set pulse sensor out
  writePulseDetectorOut(loopMillis);
  
  // set the motor outputs
  writeMotorStateOut(loopMillis);
  
  // Save the motor state and pulse count
  handleSaving(loopMillis);

  // Write to the activity pin
  digitalWrite(activityPin, getActivity(loopMillis));

  if (firstLoop) {
    firstLoop = false;
  }
}

// --- Setup functions
void setupPins() {
  pinMode(lightButtonPin, INPUT);
  pinMode(lightOutPin, OUTPUT);
  
  pinMode(motorButtonPin, INPUT);
  pinMode(motorUpPin, OUTPUT);
  pinMode(motorDownPin, OUTPUT);
  
  pinMode(batteryInPin, INPUT);
  pinMode(motorCurrentPin, INPUT);
  
  pinMode(pulsePin, INPUT);
  pinMode(closedPulsePin, INPUT);
  pinMode(pulseResetPin, INPUT);

  pinMode(pulseDetectorPin, OUTPUT);

  pinMode(activityPin, OUTPUT);

  pinMode(powerPin, INPUT);
  
  // Set initial out states
  digitalWrite(lightOutPin, lightState);
  digitalWrite(motorUpPin, LOW);
  digitalWrite(motorDownPin, LOW);
  digitalWrite(pulseDetectorPin, LOW);
  digitalWrite(activityPin, HIGH);
}

void setInitialMotorState() {
  // Configure the initial umbrella state, reading from the EEPROM.
  motorState = readIntFromEEPROM(motorStateAddress);
  switch(motorState) {
    case OPEN:
    case CLOSED:
    case OPEN_PARTIAL:
    case CLOSED_PARTIAL:
      break;
    case CLOSING:
      motorState = OPEN_PARTIAL;
      break;
    default:
      motorState = CLOSED;
      break;
  }
}

void setInitialPulseCount() {
  // Check if the byte after the motor state in eeprom is 0
  // If it is, then read the read the following bytes to the pulse count.
  // If it isnt, this is out first run. Set it and the following bytes to 0 and pulseCount to 0
  firstRunAddress = sizeof(int);
  pulseCountAddress = firstRunAddress + 1;
  if (EEPROM.read(firstRunAddress) != 0) {
    EEPROM.write(firstRunAddress, 0);
    saveIntIntoEEPROM(pulseCountAddress, 0);
    pulseCount = 0;
  } else {
    pulseCount = readIntFromEEPROM(pulseCountAddress);
  }
}

void setOpeningLimits() {
  // Check that the opening amps fall within the allowable ranges.
  openingAmpLimit = openingAmpLimit < 0.0 ? 0.0 : openingAmpLimit > maxCurrentAmps ? maxCurrentAmps : openingAmpLimit;
  // Calculate opening volt limit
  openingVoltLimit = (openingAmpLimit * 0.05) + motorVoltageMinimum;
}

// --- Loop functions
void updateBatteryLevel() {
  // Read battery voltage
  float batteryVoltage = analogToVoltage(analogRead(batteryInPin));
  
  // Check the battery level
  hysteresisCheck(&lightsVoltageLimit, batteryVoltage);
  hysteresisCheck(&motorVoltageLimit, batteryVoltage);
  //Serial.print(lightsVoltageLimit.isOn);
  //Serial.println(motorVoltageLimit.isOn);
}

void handleLights(unsigned long loopMillis) {
  int lightReading = digitalRead(lightButtonPin);
  
  if (lightReading != lastLightButtonState) {
    // reset the debouncing timer
    lastLightDebounceTime = loopMillis;
  }
  
  if ((loopMillis - lastLightDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (lightReading != lightButtonState) {
      lightButtonState = lightReading;

      // only toggle the LED if the new button state is HIGH
      if (lightButtonState == HIGH) {
        lightState = !lightState;
      }
    }
  }
  
  // If the battery is too low, just always turn the lights off
  if (!lightsVoltageLimit.isOn) {
    lightState = LOW;
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastLightButtonState = lightReading;
}

void handlePulses(unsigned long loopMillis) {
  int pulseReading = digitalRead(pulsePin);

  if (firstLoop) {
    pulseState = pulseReading;
    lastPulseState = pulseReading;
  }
  
  if (pulseReading != lastPulseState) {
    lastPulseDebounceTime = loopMillis;
  }

  if ((loopMillis - lastPulseDebounceTime) >= pulseDebounceDelay) {
    // if the input state has changed:
    if (pulseReading != pulseState) {
      pulseState = pulseReading;

      // If we are in any of the open or opending states, increment the pulses
      // If closing, decrement
      // CLOSED will now only be set when we get a pulse on the closedPulsePin, and the count should not change in this state
      switch (motorState) {
        case OPENING:
        case OPEN:
        case OPEN_PARTIAL:
          pulseCount += 1;
          break;
        case CLOSING:
        case CLOSED_PARTIAL:
          pulseCount -= 1;
          break;
        default:
          break;
      }
      //Serial.println(pulseCount);
    }
  }
  lastPulseState = pulseReading;
}

void readClosedPulse(unsigned long loopMillis) {
  int closedPulseReading = digitalRead(closedPulsePin);

  if (firstLoop) {
    closedPulseState = closedPulseReading;
    lastClosedPulseState = closedPulseReading;
  }
  
  if (closedPulseReading != lastClosedPulseState) {
    lastClosedPulseDebounceTime = loopMillis;
  }

  if ((loopMillis - lastClosedPulseDebounceTime) >= pulseDebounceDelay) {
    // if the input state has changed:
    if (closedPulseReading != closedPulseState) {
      closedPulseState = closedPulseReading;
    }
  }
  lastClosedPulseState = closedPulseReading;
}

void handleMotorButtonInput(unsigned long loopMillis) {
  int motorReading = digitalRead(motorButtonPin);
  
  if (motorReading != lastMotorButtonState) {
    // reset the debouncing timer
    lastMotorDebounceTime = loopMillis;
  }
  
  if ((loopMillis - lastMotorDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (motorReading != motorButtonState) {
      motorButtonState = motorReading;

      // only update the motor state if the new button state is HIGH
      if (motorButtonState == HIGH) {    
        pulseDetectorStartTime = loopMillis;
		    switch (motorState) {
          case OPENING:
          motorState = OPEN_PARTIAL;
          pulseDetectorDelay = stopPulseDetectorDelay;
          break;
          case OPEN:
          case OPEN_PARTIAL:
          motorState = CLOSING;
          pulseDetectorDelay = startPulseDetectorDelay;
          break;
          case CLOSED:
          case CLOSED_PARTIAL:
          motorState = motorVoltageLimit.isOn ? OPENING : CLOSING;
          pulseDetectorDelay = startPulseDetectorDelay;
          currentMonitorDelayStartTime = loopMillis;
          break;
          case CLOSING:
          motorState = CLOSED_PARTIAL;
          pulseDetectorDelay = stopPulseDetectorDelay;
          break;
        }
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastMotorButtonState = motorReading;
}

void handleMotorLogic(unsigned long loopMillis) {
  if (motorState == OPENING && (loopMillis - currentMonitorDelayStartTime > currentMonitorDelay + startPulseDetectorDelay)) {
    // When opeing, check against the current
    float currentVoltage = analogToVoltage(analogRead(motorCurrentPin));
    
    if (currentVoltage >= openingVoltLimit) {
      motorState = OPEN;
      pulseDetectorStartTime = loopMillis;
      pulseDetectorDelay = stopPulseDetectorDelay;
    }
  }

  // When closing, check against the pulse count and closedPulseState
  if (motorState == CLOSING) {
    if (pulseCount <= pulsesToListenForClosed && closedPulseState == HIGH) {
      motorState = CLOSED;
      pulseCount = 0;
      pulseDetectorStartTime = loopMillis;
      pulseDetectorDelay = stopPulseDetectorDelay;
    }
  }
  
  if (motorState != OPEN) {
    lightState = LOW;
  }
}

void handleResetButtonInput(unsigned long loopMillis) {
  int resetReading = digitalRead(pulseResetPin);
  
  if (resetReading != lastResetButtonState) {
    // reset the debouncing timer
    lastResetDebounceTime = loopMillis;
  }
  
  if ((loopMillis - lastResetDebounceTime) > resetDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (resetReading != resetButtonState) {
      resetButtonState = resetReading;

      if (resetButtonState == HIGH) {
        pulseCount = 0;
        motorState = CLOSED;
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastResetButtonState = resetReading;
}

void writePulseDetectorOut(unsigned long loopMillis) {
  if ((motorState == OPENING || motorState == CLOSING) || (loopMillis - pulseDetectorStartTime < pulseDetectorDelay)) {
    digitalWrite(pulseDetectorPin, HIGH);
  } else {
    digitalWrite(pulseDetectorPin, LOW);
  }
}

void writeMotorStateOut(unsigned long loopMillis) {
  switch (motorState) {
    case OPENING:
      digitalWrite(motorUpPin, (loopMillis - pulseDetectorStartTime >= pulseDetectorDelay) ? HIGH : LOW);
      digitalWrite(motorDownPin, LOW);
      break;
    case CLOSING:
      digitalWrite(motorUpPin, LOW);
      digitalWrite(motorDownPin, (loopMillis - pulseDetectorStartTime >= pulseDetectorDelay) ? HIGH : LOW);
      break;
    case OPEN:
    case OPEN_PARTIAL:
    case CLOSED:
    case CLOSED_PARTIAL:
      digitalWrite(motorUpPin, LOW);
      digitalWrite(motorDownPin, LOW);
      break;
  }
}

// Checks for all possible forms of activity and returns HIGH if any of them are true, otherwise LOW.
int getActivity(unsigned long loopMillis) {
  // Check light state
  if (lightState == HIGH) {
    return HIGH;
  }

  // check motor state
  if (motorState == OPENING || motorState == CLOSING) {
    return HIGH;
  }

  // check button debounces
  if (
    ((loopMillis - lastResetDebounceTime) <= resetDebounceDelay && lastResetButtonState == HIGH) ||
    ((loopMillis - lastMotorDebounceTime) <= debounceDelay && lastMotorButtonState == HIGH) ||
    ((loopMillis - lastLightDebounceTime) <= debounceDelay && lastLightButtonState == HIGH)
  ) {
    return HIGH;
  }

  // check sensor timer
  if (loopMillis - pulseDetectorStartTime < pulseDetectorDelay) {
    return HIGH;
  }

  return LOW;
}

// Check the powerPin, only save when it is low.
void handleSaving(unsigned long loopMillis) {
  int powerPinReading = digitalRead(powerPin);
  
  if (powerPinReading != lastPowerPinState) {
    // reset the debouncing timer
    lastPowerDebounceTime = loopMillis;
  }
  
  if ((loopMillis - lastPowerDebounceTime) > powerDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (powerPinState != powerPinReading) {
      powerPinState = powerPinReading;
    }
  }

  // Only save to the EEPROM when the power drops, to extend the like of the EEPROM
  // As per the documentation, it is only reliable for 100 000 writes. This function should make it last more than long enough for this project.
  if (powerPinState == LOW) {
    saveMotorState();
    saveIntIntoEEPROM(pulseCountAddress, pulseCount);
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastPowerPinState = powerPinReading;
}

// Write the motor state to the EEPROM. Optimisted to reduce writes, improving longevity of EEPROM
void saveMotorState() {
  int writeState;
  switch(motorState) {
    case OPEN:
      writeState = OPEN;
      break;
    case CLOSING:
    case OPEN_PARTIAL:
      writeState = OPEN_PARTIAL;
      break;
    case CLOSED:
    case OPENING:
    case CLOSED_PARTIAL:
    default:
      writeState = CLOSED;
      break;
  }
  saveIntIntoEEPROM(motorStateAddress, writeState);
}

// --- Utility functions

// Convert the raw data value (0 - 1023) to voltage (0.0V - voltageLimitV):
float analogToVoltage(int analogReadValue) {
  return (analogReadValue / 1023.0) * voltageLimit;
}

// Checks whether or not the new voltage should trigger a change in state
void hysteresisCheck(AnalogHysteresis* analogHysteresis, float analogVoltage) {
  if (analogHysteresis->offValue >= analogVoltage) {
    analogHysteresis->isOn = false;
  } else if (analogHysteresis->onValue <= analogVoltage) {
    analogHysteresis->isOn = true;
  }
}

// For EEPROM notes: https://roboticsbackend.com/arduino-store-int-into-eeprom/
// Used to write a multi-byte int to EEPROM
void saveIntIntoEEPROM(int address, int number) { 
  EEPROM.update(address, number >> 8);
  EEPROM.update(address + 1, number & 0xFF);
}

// Used to write a multi-byte int to EEPROM
int readIntFromEEPROM(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

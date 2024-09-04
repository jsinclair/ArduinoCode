#include <EEPROM.h>

// Float constants for battery thresholds. All values in volts.
const float batteryMotorLimitOffValue = 2.8; // Battery threshold for the motor, the umbrella can only close when its off.
const float batteryMotorLimitOnValue = 3.0;
const float batteryLightLimitOffValue = 2.75; // Battery threshold for the lights.
const float batteryLightLimitOnValue = 2.9;
// Constants for opening and closing the umbrella
float openingAmpLimit = 7.0; // the nnA for the opening current limit. Allowable range: 0.0 - maxCurrentAmps
float closingAmpLimit = 7.0; // the nnA for the opening current limit. Allowable range: 0.0 - maxCurrentAmps
float closingMinimumAmpLimit = 1.0; // the nnA for the opening current limit. Allowable range: 0.0 - maxCurrentAmps
const float voltageLimit = 3.3; // change this for the different arduinos, 3.3 for mini, 5.0 for nano
const float motorVoltageMinimum = 2.5; // This is used as the minimum voltage when reading motor current, to compensate for the base amount of 2.5 volts.
const float maxCurrentAmps = 8;
const float voltsPerAmp = 0.1; // Used with openingAmpLimit to work out the opening volt limit
const unsigned long currentMonitorDelay = 600; // Current monitor delay for opening and closing, in milliseconds
// Remote Cycling Constants
const unsigned long remoteOnDuration = 500; // How long the remote output should stay high in a remote cycle
const unsigned long remoteOffDuration = 2500; // How long the remote output should stay low in a remote cycle

const float savingVoltageThreshold = 3.0;
const bool bypassRemoteCylcling = false;

float openingVoltLimit; // Calculated in setup: (openingAmpLimit * voltsPerAmp) + motorVoltageMinimum
float closingVoltLimit; // Calculated in setup: (closingAmpLimit * voltsPerAmp) + motorVoltageMinimum
float closingMinimumVoltage; // If current voltage drops below this while closing, stop closing

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

// I/O Pins
const int remotePin = 2; // Output pin for the remote state
const int lightButtonPin = 3;     	// Light button in pin
const int motorDownInPin = 4;		// Motor down button pin
const int motorStopInPin = 5;		// Motor stop button pin
const int motorUpInPin = 6;		// Motor up button pin
const int lightOutPin = 10;			// Light out pin
const int motorUpOutPin = 11;			// Output pin for up motor
const int motorDownOutPin = 12;		// Output pin for down motor
const int currentSensorPin = 13; // Output pin for the inrush detector

const int batteryInPin = A1;     	// Battery in pin
const int motorCurrentPin = A2;   // Motor current in pin

// Up/Down states
const int OPENING = 0;
const int OPEN = 1;
const int CLOSING = 2;
const int SCHRODINGER = 3;

// Motor consts and vars
int motorState = SCHRODINGER;
unsigned long currentMonitorDelayStartTime = 0; // The start time of a current monitor delay

// Down Button
int lastDownButtonState = LOW;
int downButtonState;
unsigned long lastDownButtonDebounceTime = 0;  // the last time the output pin was toggled

// Stop Button
int lastStopButtonState = LOW;
int stopButtonState;
unsigned long lastStopButtonDebounceTime = 0;  // the last time the output pin was toggled

// Up Button
int lastUpButtonState = LOW;
int upButtonState;
unsigned long lastUpButtonDebounceTime = 0;  // the last time the output pin was toggled

// Light consts and vars
int lastLightButtonState = LOW;
int lightButtonState;
int lightState = LOW;
unsigned long lastLightDebounceTime = 0;  // the last time the output pin was toggled

// Remote Cycling
unsigned long currentRemoteCycleDuration = remoteOnDuration;
unsigned long lastRemoteCycleTime = 0;  // the last time the output pin was toggled
bool remoteState = HIGH;

// Other consts and vars
const unsigned long debounceDelay = 50;    // the standard button debounce time

// EEPROM Stuff
const int motorStateAddress = 0;
int lastWriteState = SCHRODINGER;

void setup() {
  setupPins();
  
  //Serial.begin(9600);
  
  setInitialMotorState();

  setLimits();
}

void loop() {
  // BATTERY MONITOR
  updateBatteryLevel();
  // Handle lights input
  handleLights();
  // Handle down button input
  handleDownButtonInput();
  // Handle up button input
  handleUpButtonInput();
  // Handle stop button input
  handleStopButtonInput();
  // Update up/down state based on motor state and input readings
  handleMotorLogic();
  // Handle the remote state change
  handleRemoteCycle();

  // OUTPUTS
  // set the light output
  digitalWrite(lightOutPin, lightState);
  // set current sensor out
  writeCurrentSensorOut();
  // set the motor outputs
  writeMotorStateOut();
  // Save the motor state and pulse count
  handleSaving();
  // Write to the remote pin
  digitalWrite(remotePin, remoteState);
}

// --- Setup functions
void setupPins() {
  pinMode(remotePin, OUTPUT);

  pinMode(lightButtonPin, INPUT);

  pinMode(motorDownInPin, INPUT);
  pinMode(motorStopInPin, INPUT);
  pinMode(motorUpInPin, INPUT);
  
  pinMode(lightOutPin, OUTPUT);

  pinMode(motorUpOutPin, OUTPUT);
  pinMode(motorDownOutPin, OUTPUT);
  
  pinMode(currentSensorPin, OUTPUT);

  pinMode(batteryInPin, INPUT);
  pinMode(motorCurrentPin, INPUT);
  
  // Set initial out states
  digitalWrite(lightOutPin, lightState);
  digitalWrite(motorUpOutPin, LOW);
  digitalWrite(motorDownOutPin, LOW);
  digitalWrite(motorStopInPin, LOW);
  digitalWrite(currentSensorPin, LOW);
  digitalWrite(remotePin, remoteState);
}

void setInitialMotorState() {
  // Configure the initial umbrella state, reading from the EEPROM.
  motorState = readIntFromEEPROM(motorStateAddress);
  switch(motorState) {
    case OPEN:
      break;
    default:
      motorState = SCHRODINGER;
      break;
  }
}

void setLimits() {
  // Check that the opening amps fall within the allowable ranges.
  openingAmpLimit = openingAmpLimit < 0.0 ? 0.0 : openingAmpLimit > maxCurrentAmps ? maxCurrentAmps : openingAmpLimit;
  // Calculate opening volt limit
  openingVoltLimit = (openingAmpLimit * voltsPerAmp) + motorVoltageMinimum;

  // Check that the closing amps fall within the allowable ranges.
  closingAmpLimit = closingAmpLimit < 0.0 ? 0.0 : closingAmpLimit > maxCurrentAmps ? maxCurrentAmps : closingAmpLimit;
  // Calculate closing volt limit
  closingVoltLimit = (closingAmpLimit * voltsPerAmp) + motorVoltageMinimum;
  // Calculate minimum closing voltage
  closingMinimumVoltage = closingMinimumAmpLimit * voltsPerAmp;
}

// --- Loop functions
void updateBatteryLevel() {
  // Read battery voltage
  float batteryVoltage = analogToVoltage(analogRead(batteryInPin));
  
  // Check the battery level
  hysteresisCheck(&lightsVoltageLimit, batteryVoltage);
  hysteresisCheck(&motorVoltageLimit, batteryVoltage);
}

void handleLights() {
  int lightReading = digitalRead(lightButtonPin);
  
  if (lightReading != lastLightButtonState) {
    // reset the debouncing timer
    lastLightDebounceTime = millis();
  }
  
  if ((millis() - lastLightDebounceTime) > debounceDelay) {
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

void handleDownButtonInput() {
  int downReading = digitalRead(motorDownInPin);
  
  if (downReading != lastDownButtonState) {
    // reset the debouncing timer
    lastDownButtonDebounceTime = millis();
  }

  if ((millis() - lastDownButtonDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (downReading != downButtonState) {
      downButtonState = downReading;

      if (downButtonState == HIGH) {    
		    switch (motorState) {
          case SCHRODINGER:
          case OPEN:
            currentMonitorDelayStartTime = millis();
            motorState = CLOSING;
            break;
          default:
            motorState = SCHRODINGER;
            break;
        }
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastDownButtonState = downReading;
}

void handleUpButtonInput() {
  int upReading = digitalRead(motorUpInPin);
  
  if (upReading != lastUpButtonState) {
    // reset the debouncing timer
    lastUpButtonDebounceTime = millis();
  }

  if ((millis() - lastUpButtonDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (upReading != upButtonState) {
      upButtonState = upReading;

      if (upButtonState == HIGH) {    
        switch (motorState) {
          case SCHRODINGER:
            if (!motorVoltageLimit.isOn) {
              break;
            }
            currentMonitorDelayStartTime = millis();
            motorState = OPENING;
            break;
          case CLOSING:
          case OPENING:
            motorState = SCHRODINGER;
            break;
          case OPEN:
            break;
        }
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastUpButtonState = upReading;
}

void handleStopButtonInput() {
  int stopReading = digitalRead(motorStopInPin);
  
  if (stopReading != lastStopButtonState) {
    // reset the debouncing timer
    lastStopButtonDebounceTime = millis();
  }

  if ((millis() - lastStopButtonDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (stopReading != stopButtonState) {
      stopButtonState = stopReading;

      if (stopButtonState == HIGH) {    
        switch (motorState) {
          case OPEN:
            break;
          default:
            motorState = SCHRODINGER;
            break;
        }
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastStopButtonState = stopReading;
}

void handleMotorLogic() {
  if (motorState == OPENING && (millis() - currentMonitorDelayStartTime > currentMonitorDelay)) {
    // When opeing, check against the current
    float currentVoltage = analogToVoltage(analogRead(motorCurrentPin));
    if (currentVoltage >= openingVoltLimit) {
      motorState = OPEN;
    }
  }

  // When closing, check against the pulse count and closedPulseState
  if (motorState == CLOSING && (millis() - currentMonitorDelayStartTime > currentMonitorDelay)) { 
    float currentVoltage = analogToVoltage(analogRead(motorCurrentPin));
    if (currentVoltage >= closingVoltLimit || currentVoltage <= closingMinimumVoltage) {
      motorState = SCHRODINGER;
    }
  }
  
  if (motorState != OPEN) {
    lightState = LOW;
  }
}

void handleRemoteCycle() {
  // If bypassRemoteCylcling then skip all the remote cycling logic
  if (bypassRemoteCylcling) {
    remoteState = HIGH;
    return;
  }

  if (millis() - lastRemoteCycleTime > currentRemoteCycleDuration) {
    lastRemoteCycleTime = millis();
    if (remoteState == HIGH) {
      remoteState = LOW;
      currentRemoteCycleDuration = remoteOffDuration;
    } else {
      remoteState = HIGH;
      currentRemoteCycleDuration = remoteOnDuration;
    }
  }
}

void writeCurrentSensorOut() {
  if (motorState == OPENING || motorState == CLOSING) {
    digitalWrite(currentSensorPin, HIGH);
  } else {
    digitalWrite(currentSensorPin, LOW);
  }
}

void writeMotorStateOut() {
  switch (motorState) {
    case OPENING:
      digitalWrite(motorUpOutPin, HIGH);
      digitalWrite(motorDownOutPin, LOW);
      break;
    case CLOSING:
      digitalWrite(motorUpOutPin, LOW);
      digitalWrite(motorDownOutPin, HIGH);
      break;
    default:
      digitalWrite(motorUpOutPin, LOW);
      digitalWrite(motorDownOutPin, LOW);
      break;
  }
}

// Check the powerPin, only save when it is low.
void handleSaving() {
  // Read battery voltage
  float batteryVoltage = analogToVoltage(analogRead(batteryInPin));
  
  // Only save to the EEPROM when the power drops, to extend the life of the EEPROM
  // As per the documentation, it is only reliable for 100 000 writes. This function should make it last more than long enough for this project.
  if (batteryVoltage <= savingVoltageThreshold) {
    saveMotorState();
  }
}

// Write the motor state to the EEPROM. Optimisted to reduce writes, improving longevity of EEPROM
void saveMotorState() {
  int writeState;
  switch(motorState) {
    case OPEN:
      writeState = OPEN;
      break;
    default:
      writeState = SCHRODINGER;
      break;
  }

  if (lastWriteState != writeState) {
    lastWriteState = writeState;
    saveIntIntoEEPROM(motorStateAddress, writeState);
  }
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

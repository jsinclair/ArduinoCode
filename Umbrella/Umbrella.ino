#include <EEPROM.h>

// Float constants for battery thresholds. All values in volts.
const float batteryLimitOffValue = 2.6; // Battery threshold, turns off lights and only allows the umbrella to close when its off.
const float batteryLimitOnValue = 3.0;
// Constants for opening and closing the umbrella
float openingAmpLimit = 4.0; // the nnA for the opening current limit. Allowable range: 0.0 - 7.0
const float voltageLimit = 3.3; // change this for the different arduinos, 3.3 for mini, 5.0 for nano
const float motorVoltageMinimum = 2.5; // This is used as the minimum voltage when reading motor current, to compensate for the base amount of 2.5 volts.
const float motorAmpsCoefficient = 10;
const long remoteReceiverOnDuration = 1500; // How long in milliseconds the remote receiver stays on for in a cycle.
const long remoteReceiverOffDuration = 5000; // How long in milliseconds the remote receiver stays off for in a cycle.
unsigned long currentMonitorDelay = 500; // Current monitor delay for opening and closing, in milliseconds

// Hysteresis struct to manage the various thresholds
struct AnalogHysteresis {
    float offValue; // The value at which the hysteresis turns off, in volts: nnV
    float onValue; // The value at which the hysteresis turns on, in volts: nnV
    bool isOn; // Whether the value we are tracking is on or not
};

void hysteresisCheck(AnalogHysteresis* analogHysteresis, float analogVoltage);

// The various thresholds. The first number is the off value, the second is the on value. The third value is the default state of the check.
AnalogHysteresis batteryVoltageLimit = {batteryLimitOffValue, batteryLimitOnValue, true}; // The limit for the battery, disables light and limits umbrella to closing when battery level is lower than this

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

int lastMotorButtonState = LOW;
int motorButtonState;
int motorState = CLOSED;
const int motorStateAddress = 0;
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

// Pulse consts and vars
const int pulsePin = 9;			// The pin we listen for pulses on
const int pulseResetPin = 5;	// The pin which can reset pulses
int pulseCount = 0;
const int pulseCountAddress = 1;

int lastResetButtonState = LOW;
int resetButtonState;
int resetState = LOW;
unsigned long lastResetDebounceTime = 0;  // the last time the output pin was toggled

int pulseState;
bool firstLoop = true;

// Other consts and vars
unsigned long debounceDelay = 50;    // the debounce time
unsigned long resetDebounceDelay = 1000;    // the debounce time


void setup() {
  pinMode(lightButtonPin, INPUT);
  pinMode(lightOutPin, OUTPUT);
  
  pinMode(motorButtonPin, INPUT);
  pinMode(motorUpPin, OUTPUT);
  pinMode(motorDownPin, OUTPUT);
  
  pinMode(batteryInPin, INPUT);
  pinMode(motorCurrentPin, INPUT);
  
  pinMode(pulsePin, INPUT);
  pinMode(pulseResetPin, INPUT);
  
  // Set initial out states
  digitalWrite(lightOutPin, lightState);
  digitalWrite(motorUpPin, LOW);
  digitalWrite(motorDownPin, LOW);
  
  //Serial.begin(9600);
  
  // Configure the initial umbrella state, reading from the EEPROM.
  motorState = EEPROM.read(motorStateAddress);
  if (motorState == OPENING) {
    motorState = CLOSED;
  } else if (motorState == CLOSING) {
    motorState = OPEN_PARTIAL;
  } else if (motorState < 0) {
    motorState = CLOSED;
  }
  
  // Load pulse count
  pulseCount = EEPROM.read(pulseCountAddress);
  if (pulseCount < 0) {
    pulseCount = 0;
  }
  
  // Check that the opening and closing amps are within the allowable ranges.
  openingAmpLimit = openingAmpLimit < 0.0 ? 0.0 : openingAmpLimit > 7.0 ? 7.0 : openingAmpLimit;
}

void loop() {
  // Get the millis time for this loop.
  const long loopMillis = millis();
  
  // BATTERY MONITOR
  // Convert the raw data value to voltage:
  float batteryVoltage = analogToVoltage(analogRead(batteryInPin));
  
  // Check the battery level
  hysteresisCheck(&batteryVoltageLimit, batteryVoltage);
//  Serial.println(batteryVoltage);
  
  // Read battery voltage
  //Serial.println(analogRead(batteryInPin));
  
  // Handle lights input
  int lightReading = digitalRead(lightButtonPin);
  
  if (lightReading != lastLightButtonState) {
    // reset the debouncing timer
    lastLightDebounceTime = loopMillis;
  }
  
  if ((loopMillis - lastLightDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

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
  if (!batteryVoltageLimit.isOn) {
    lightState = LOW;
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastLightButtonState = lightReading;
  
  // Handle pulse input
  int pulseReading = digitalRead(pulsePin);

  if (firstLoop) {
    firstLoop = false;
    pulseState = pulseReading;
  }
  
  // if the input state has changed:
  if (pulseReading != pulseState) {
    pulseState = pulseReading;

    // If we are in any of the open or opending states, increment the pulses
    // If closing, decrement
    switch (motorState) {
      case OPENING:
      case OPEN:
      case OPEN_PARTIAL:
      pulseCount += 1;
      break;
      case CLOSING:
      case CLOSED:
      pulseCount -= 1;
      break;
    }
  }
  //Serial.println(pulseCount);
  
  // Handle motor input
  int motorReading = digitalRead(motorButtonPin);
  
  if (motorReading != lastMotorButtonState) {
    // reset the debouncing timer
    lastMotorDebounceTime = loopMillis;
  }
  
  if ((loopMillis - lastMotorDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (motorReading != motorButtonState) {
      motorButtonState = motorReading;

      // only toggle the LED if the new button state is HIGH
      if (motorButtonState == HIGH) {
        currentMonitorDelayStartTime = loopMillis;
        
		switch (motorState) {
          case OPENING:
          motorState = OPEN_PARTIAL;
          break;
          case OPEN:
          case OPEN_PARTIAL:
          motorState = CLOSING;
          break;
          case CLOSED:
          motorState = !batteryVoltageLimit.isOn ? CLOSING : OPENING;
          break;
          case CLOSING:
          motorState = CLOSED;
          break;
        }
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastMotorButtonState = motorReading;
  
  // Update up/down state based on motor state and input readings
  if (loopMillis > (currentMonitorDelayStartTime + currentMonitorDelay) && (motorState == OPENING || motorState == CLOSING)) {
    
    // When opeing, check against the current
    if (motorState == OPENING) {
      float currentVoltage = analogToVoltage(analogRead(motorCurrentPin));
      float currentAmps = (currentVoltage - motorVoltageMinimum) * motorAmpsCoefficient;
      if (currentAmps >= openingAmpLimit) {
        motorState = OPEN;
      }
    }
    
    // When closing, check against the pulse count
    if (motorState == CLOSING) {
      if (pulseCount <= 0) {
        motorState = CLOSED;
      }
    }
  }
  
  if (motorState != OPEN) {
    lightState = LOW;
  }

  // Handle reset input
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
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastResetButtonState = resetReading;
  
  // OUTPUTS
  
  // set the light output
  digitalWrite(lightOutPin, lightState);
  
  // set the motor outputs
  switch (motorState) {
    case OPENING:
    digitalWrite(motorUpPin, HIGH);
    digitalWrite(motorDownPin, LOW);
    break;
    case CLOSING:
    digitalWrite(motorUpPin, LOW);
    digitalWrite(motorDownPin, HIGH);
    break;
    case OPEN:
    case OPEN_PARTIAL:
    case CLOSED:
    digitalWrite(motorUpPin, LOW);
    digitalWrite(motorDownPin, LOW);
    break;
  }
  
  // Save the umbrella state and pulse count
  EEPROM.update(motorStateAddress, motorState);
  EEPROM.update(pulseCountAddress, pulseCount);
}

// Convert the raw data value (0 - 1023) to voltage (0.0V - voltageLimitV):
float analogToVoltage(int analogReadValue) {
  return analogReadValue * (voltageLimit / 1023.0);
}

// Checks whether or not the new voltage should trigger a change in state
void hysteresisCheck(AnalogHysteresis* analogHysteresis, float analogVoltage) {
  if (analogHysteresis->offValue >= analogVoltage) {
    analogHysteresis->isOn = false;
  } else if (analogHysteresis->onValue <= analogVoltage) {
    analogHysteresis->isOn = true;
  }
}

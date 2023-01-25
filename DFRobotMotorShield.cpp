#include "DFRobotMotorShield.h"

// Constructors
DFRobotMotorShield::DFRobotMotorShield() : m1DirPin(M1_DIR_PIN), m1PWMPin(M1_PWM_PIN), m2DirPin(M2_DIR_PIN), m2PWMPin(M2_PWM_PIN) {
}

DFRobotMotorShield::DFRobotMotorShield(byte m1Dir, byte m1PWM, byte m2Dir, byte m2PWM) :
  m1DirPin(m1Dir), m1PWMPin(m1PWM), m2DirPin(m2Dir), m2PWMPin(m2PWM) {
}


void DFRobotMotorShield::initialize() {
  if (!initialized) {
    initializePins();
    initialized = true;
  }
}

void DFRobotMotorShield::initializePins() {
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high,
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  digitalWrite(m1PWMPin, LOW);
  pinMode(m1PWMPin, OUTPUT);
  digitalWrite(m1PWMPin, LOW);

  digitalWrite(m2PWMPin, LOW);
  pinMode(m2PWMPin, OUTPUT);
  digitalWrite(m2PWMPin, LOW);

  digitalWrite(m1DirPin, LOW);
  pinMode(m1DirPin, OUTPUT);
  digitalWrite(m1DirPin, HIGH); // define high as forward direction

  digitalWrite(m2DirPin, LOW);
  pinMode(m2DirPin, OUTPUT);
  digitalWrite(m2DirPin, HIGH); // define high as forward direction
}

// set speed and direction for motor 1
void DFRobotMotorShield::setM1Speed(int speed) {
  initialize(); // initialize if necessary

  boolean reverse = false;

  if (speed < 0) {
    speed = -speed; // make speed a positive quantity
    reverse = true;    // preserve the direction
  }
  if (speed > MAX_SPEED) {
    speed = MAX_SPEED;
  }

  // set the speed
  analogWrite(m1PWMPin, speed);

  // set the direction
  // reverse if speed was negative or if _flipM1 setting is active, but not both
  if (reverse || _flipM1) {
    digitalWrite(m1DirPin, LOW); // reverse direction
  }
  else {
    digitalWrite(m1DirPin, HIGH); // forward direction
  }
}

// set speed and direction for motor 2
void DFRobotMotorShield::setM2Speed(int speed) {
  initialize(); // initialize if necessary

  boolean reverse = false;

  if (speed < 0) {
    speed = -speed;  // make speed a positive quantity
    reverse = true;  // preserve the direction
  }
  if (speed > MAX_SPEED)  {
    speed = MAX_SPEED;
  }

  // set the speed
  analogWrite(m2PWMPin, speed);

  // set the direction
  // reverse if speed was negative or if _flipM1 setting is active, but not both
  if (reverse || _flipM2)
    digitalWrite(m2DirPin, LOW);
  else
    digitalWrite(m2DirPin, HIGH);
}

// set speed for both motors
void DFRobotMotorShield::setSpeeds(int m1Speed, int m2Speed) {
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

void DFRobotMotorShield::flipM1(boolean flip) {
  _flipM1 = flip;
}

void DFRobotMotorShield::flipM2(boolean flip) {
  _flipM2 = flip;
}

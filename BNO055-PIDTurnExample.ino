/*
  Motor Assignments
  M2 -> left motor
  M1 -> right motor
  Note: the default setting for the motor driver pins are 4, 5, 6, 7. If you want to use these pins for some other purpose,
  you'll need to reassign the motor driver pins via the DFRobotMotorShield constructor as follows

  DFRobotMotorShield motors(M1_DIR_PIN, M1_PWM_PIN, M2_DIR_PIN, M2_PWM_PIN);

  M1_DIR_PIN -- motor 1 direction control
  M1_PWM_PIN -- motor 1 pwm control
  M2_DIR_PIN -- motor 2 direction control
  M2_PWM_PIN -- motor 2 pwm control

*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h> // use the servo library
#include "DFRobotMotorShield.h"
DFRobotMotorShield motors;

// pin numbers
#define DISTANCE_SERVO_PIN 46  // distance servo pin
#define TRIG_PIN 40 // sensor trigger pin
#define ECHO_PIN 18 // sensor echo pin, pin 18

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// wheel speeds
int leftSpeed, rightSpeed;
const int maxTurnSpeed = 100;

// PID constants
// You'll need to adjust "tune" this values for optimal performance
const float KP_Left = 10.0;//80  10
const float KI_Left = .2;//50
const float KD_Left = .5;//20
const float KP_Right = 10.0;//80
const float KI_Right = .2;//50
const float KD_Right = .5;//20
const float DELTA_TIME = 1; //50 milliseconds

// PID variables
float error = 0.0;
float previousError = 0.0;
float leftIntegral = 0.0;
float leftDerivative = 0.0;
float rightIntegral = 0.0;
float rightDerivative = 0.0;

// define time variables/conversions
long currentMillis = 0;
long previousMillis = 0;
#define MILLISEC_TO_SEC 1/1000

// distance sensor servo
//#define DISTANCE_SERVO_PIN 46  // distance sensor servo pin
Servo myservo;
int servoPosition = 90;

// yaw angle variables
float yawAngle, remappedYawAngle; // deg
float angleSetpoint = -90; // deg
float angleTolerance = 1; // deg used to be 1
float setpointLowerLimit = angleSetpoint - angleTolerance;
float setpointUpperLimit = angleSetpoint + angleTolerance;

int prev_state, state = 0;
const int right_turn = 1;
const int left_turn = 5;
const int straight = 2;
const int right_look = 3;
const int straight_look = 4;

const byte leftChannelA = 3;
const byte leftChannelB = 32;
const byte rightChannelA = 2;
const byte rightChannelB = 34;

// define encoder count variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// define channel A previous and current variables
volatile byte prevLeftAVal = 0;
volatile byte currentLeftAVal = 0;
volatile byte prevRightAVal = 0;
volatile byte currentRightAVal = 0;

float distance = 3000;

long timeOfTravel;
float distanceCm, distanceIn;

void setup() {
  Serial.begin(115200);

  // setup encoder pins
  pinMode(leftChannelA, INPUT);
  pinMode(leftChannelB, INPUT);
  pinMode(rightChannelA, INPUT);
  pinMode(rightChannelB, INPUT);

  pinMode(TRIG_PIN, OUTPUT); // set the trigger pin as an output
  pinMode(ECHO_PIN, INPUT); // set the echo pin as an input

  // read initial states of encoder A channels
  prevLeftAVal = digitalRead(leftChannelA);
  prevRightAVal = digitalRead(rightChannelA);

  // attach channel A pins to ISRs
  attachInterrupt(digitalPinToInterrupt(leftChannelA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightChannelA), rightEncoderISR, CHANGE);

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  // get the yaw angle and wait 1 second before running your PID turning algorithm. 
  // this wait period seems to be necessary for the BNO055 to stabilize
  yawAngle = event.orientation.x; // deg
  remappedYawAngle = remapAngle(yawAngle);
  delay(1000);

  myservo.attach(DISTANCE_SERVO_PIN);  // attach the servo to the servo pin
  //myservo.write(servoPosition);

  state = right_look;
}

void loop() {
  //if(turn){
  //  rightTurn();
  //}
  //else{
  //  motors.setM1Speed(0);
  //  motors.setM2Speed(0);
  //}
  //if (leftSpeed < 5 && rightSpeed < 5)  turn = false;
  Serial.println(state);
  switch (state) {
    case right_turn:
      Serial.println("right state");
      angleSetpoint = 90;
      rightTurn();
      //delay(3000);
      //state = right_look;
      break;
    case left_turn:
      Serial.println("left state");
      angleSetpoint = -90;
      rightTurn();
      //state = straight;
      break;
    
    case straight:
      Serial.println("straight state");
      angleSetpoint = 0;
      leftSpeed = 130;//100
      rightSpeed = 130;//100
      motors.setM1Speed(rightSpeed);
      motors.setM2Speed(leftSpeed);
      delay(500);
      
//      leftEncoderCount = 0;
//      rightEncoderCount = 0;
//      while (leftEncoderCount < distance && rightEncoderCount < distance){
//        motors.setM1Speed(60);
//        motors.setM2Speed(60);
//        Serial.print("left encoder value = ");
//        Serial.print(leftEncoderCount);
//        Serial.print(", right encoder value = ");
//        Serial.println(rightEncoderCount);
//        } 
//      leftEncoderCount = 0;
//      rightEncoderCount = 0;
      state = right_look;
      break;
     
     case right_look:
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      myservo.write(0); 
      delay(1000);
      Serial.println("right looking");
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(3);
    
      // set the TRIG_PIN high for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
    
      // reads the ECHO_PIN and return the time of travel in microseconds
      timeOfTravel = pulseIn(ECHO_PIN, HIGH);
    
      // calculate the distance
      distanceCm = 0.0343 * timeOfTravel  / 2.0;
      distanceIn = 0.0133 * timeOfTravel  / 2.0;
    
      // print the distance to the serial monitor
      Serial.print("Distance: ");
      Serial.print(distanceCm);
      Serial.print(" cm, ");
      Serial.print(distanceIn);
      Serial.println(" in");

      if (distanceCm > 40){
        Serial.println("distance is big so i will turn");
        delay(2000);
        state = right_turn;
      }else{
        state = straight_look;
      }
      
      
      break;

    case straight_look:
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      myservo.write(82); 
      delay(1000);
      Serial.println("straight looking");
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(3);
    
      // set the TRIG_PIN high for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
    
      // reads the ECHO_PIN and return the time of travel in microseconds
      timeOfTravel = pulseIn(ECHO_PIN, HIGH);
    
      // calculate the distance
      distanceCm = 0.0343 * timeOfTravel  / 2.0;
      distanceIn = 0.0133 * timeOfTravel  / 2.0;
    
      // print the distance to the serial monitor
      Serial.print("Distance: ");
      Serial.print(distanceCm);
      Serial.print(" cm, ");
      Serial.print(distanceIn);
      Serial.println(" in");

      if (distanceCm > 20){
        state = straight;
      } else {
        state = right_turn;
      }
      
      break;
      
  
  
  }

  
  //if(turn){
  //  rightTurn();
  //  delay(3000);
  //  turn = false;
  //}
  //motors.setM1Speed(0);
  //motors.setM2Speed(0);
  
  //while (!rightTurn()) {};
  //delay(1000);
  prev_state = state;

}

void rightTurn(){
  currentMillis = millis();

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  // get the yaw angle
  yawAngle = event.orientation.x; // deg
  remappedYawAngle = remapAngle(yawAngle);

  if ((remappedYawAngle < setpointLowerLimit) || (remappedYawAngle > setpointUpperLimit)) {
    if ((currentMillis - previousMillis >= DELTA_TIME) ) {
      PIDTurner(remappedYawAngle, angleSetpoint, DELTA_TIME);
      motors.setM1Speed(rightSpeed);
      motors.setM2Speed(-leftSpeed);

      previousMillis = currentMillis;
    }
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.println(rightSpeed);
    if ((abs(leftSpeed) < 3 && abs(rightSpeed) < 3)){
      Serial.println("turn is complete");
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      state = straight;
      //turn = false;
    }

  }
  else {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
  }
}

void PIDTurner(float actualValue, float setpoint, float delta_time) {

  // compute the error
  error = setpoint - actualValue;

  // convert time from milliseconds to seconds
  delta_time = delta_time * MILLISEC_TO_SEC;

  // PID computation for left wheel
  leftIntegral = leftIntegral + error * delta_time;
  leftDerivative = (error - previousError) / delta_time;
  leftSpeed = int (KP_Left * error + KI_Left * leftIntegral + KD_Left * leftDerivative);

  // contrain the wheel speed to lie between -maxTurnSpeed and maxTurnSpeed
  if (leftSpeed > maxTurnSpeed) {
    leftSpeed = maxTurnSpeed;
  }
  else if (leftSpeed < -maxTurnSpeed) {
    leftSpeed = -maxTurnSpeed;
  }


  // PID computation for right wheel
  rightIntegral = rightIntegral + error * delta_time;
  rightDerivative = (error - previousError) / delta_time;
  rightSpeed = int(KP_Right * error + KI_Right * rightIntegral + KD_Right * rightDerivative);

  // contrain the wheel speed to lie between -maxTurnSpeed and maxTurnSpeed
  if (rightSpeed > maxTurnSpeed) {
    rightSpeed = maxTurnSpeed;
  }
  else if (rightSpeed < -maxTurnSpeed) {
    rightSpeed = -maxTurnSpeed;
  }

  previousError = error;
}

// remaps the output angle from the BNO055 IMU
// this remapping makes left turns positive (up to +180) and right turns negative (up to -180)
// if you want to turn greater than +/- 180 deg, you'll need create additional code
double remapAngle(double angle) {

  double xyCoordinate[2];

  angle = angle * DEG_TO_RAD;
  xyCoordinate[0] = cos(angle);
  xyCoordinate[1] = -sin(angle);

  return RAD_TO_DEG * (atan2 (xyCoordinate[1], xyCoordinate[0]));
}
void leftEncoderISR() {
  currentLeftAVal = digitalRead(leftChannelA);

  if (currentLeftAVal != prevLeftAVal) {

    if (currentLeftAVal != digitalRead(leftChannelB)) {
      // swap these for counting in opposite direction
      // leftEncoderCount++;
      leftEncoderCount--;
    }
    else {
      // swap these for counting in opposite direction
      // leftEncoderCount--;
      leftEncoderCount++;
    }
  }

  prevLeftAVal = currentLeftAVal;
}


void rightEncoderISR() {
  currentRightAVal = digitalRead(rightChannelA);

  if (currentRightAVal != prevRightAVal) {

    if (currentRightAVal != digitalRead(rightChannelB)) {
      // swap these for counting in opposite direction
      // rightEncoderCount--;
      rightEncoderCount++;
    }
    else {
      // swap these for counting in opposite direction
      // rightEncoderCount++;
      rightEncoderCount--;
    }
  }

  prevRightAVal = currentRightAVal;
}

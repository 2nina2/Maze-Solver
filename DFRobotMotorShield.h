#ifndef DFRobotMotorShield_h
#include <Arduino.h>

class DFRobotMotorShield {
  public:
    DFRobotMotorShield();
    DFRobotMotorShield(byte m1DirPin, byte m1PWMPin, byte m2DirPin, byte m2PWMPin);

    void setM1Speed(int speed);
    void setM2Speed(int speed);
    void setSpeeds(int m1Speed, int m2Speed);
    void flipM1(boolean flip);
    void flipM2(boolean flip);

  private:
    byte m1DirPin;
    byte m2DirPin;
    byte m1PWMPin;
    byte m2PWMPin;

    bool _flipM1 = false;
    bool _flipM2 = false;

    bool initialized = false;

    void initialize();
    void initializePins();

    static const byte M1_PWM_PIN = 5; // PWM pin for M1
    static const byte M1_DIR_PIN  = 4; // Direction pin for M1 (HIGH and LOW), (forward and reverse)
    static const byte M2_PWM_PIN = 6; // PWM pin for M2
    static const byte M2_DIR_PIN = 7; // Direction pin for M2 (HIGH and LOW), (forward and reverse)
    static const byte MAX_SPEED = 255;

};

#endif

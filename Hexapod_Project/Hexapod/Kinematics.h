#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


// Constants
#define SERVO_MIN 102
#define SERVO_MAX 434

extern const float link_1;
extern const float link_2;
extern const int max_len;
extern const float d;

extern int global_x;
extern int global_y;
extern int arc;


// Leg structure definition
struct Leg {
  int servo0;   // Shoulder yaw
  int servo1;   // Shoulder pitch
  int servo2;   // Elbow

  Adafruit_PWMServoDriver* pwm;  // Pointer to correct board

  int x, y, z;
  double c, L;

  int theta1_deg, theta2_deg, theta3_deg;
  int theta1_pwm, theta2_pwm, theta3_pwm;
};

// PWM board declarations
extern Adafruit_PWMServoDriver pwm1;
extern Adafruit_PWMServoDriver pwm2;

// Leg declarations
extern Leg leg1;
extern Leg leg2;
extern Leg leg3;
extern Leg leg4;
extern Leg leg5;
extern Leg leg6;

extern Leg* allLegs[6];

// Function declarations
void computeIK(Leg &leg, int x, int y, int z);
void moveLeg(Leg &leg);

#endif
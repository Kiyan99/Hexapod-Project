#include "Kinematics.h"

// Constants
const float link_1 = 78.8;
const float link_2 = 150;
const int max_len = link_1 + link_2;
const float d = 38.5;

// Global gait values
int global_x = 120;
int global_y = -130;
int arc = 30;

// PWM boards
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// Define legs
Leg leg1 = {12, 13, 14, &pwm2};  // front left
Leg leg2 = {12, 13, 14, &pwm1};     // right middle
Leg leg3 = {0, 1, 2, &pwm1};     // rear left
Leg leg4 = {8, 9, 10, &pwm2};    // front right
Leg leg5 = {0, 1, 2, &pwm2};     // left middle
Leg leg6 = {4, 5, 6, &pwm1};     // rear right

Leg* allLegs[6] = { &leg1, &leg2, &leg3, &leg4, &leg5, &leg6 };

// Inverse kinematics
void computeIK(Leg &leg, int x, int y, int z) {
  leg.x = x;
  leg.y = y;
  leg.z = z;

  leg.L = sqrt(x * x + z * z);
  double theta1_rad = atan2(x, z);

  int x_p = leg.L - d;
  leg.c = sqrt(x_p * x_p + y * y);

  double theta2_rad =
      acos((pow(link_1, 2) + pow(leg.c, 2) - pow(link_2, 2)) / (2 * link_1 * leg.c))
      + atan2(y, leg.L);

  double theta3_rad =
      acos((pow(link_1, 2) + pow(link_2, 2) - pow(leg.c, 2)) / (2 * link_1 * link_2));

  leg.theta1_deg = theta1_rad * (180.0 / M_PI);
  leg.theta2_deg = theta2_rad * (180.0 / M_PI);
  leg.theta3_deg = theta3_rad * (180.0 / M_PI);
}

// Move one leg
void moveLeg(Leg &leg) {
  leg.theta1_pwm = map(leg.theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta2_pwm = map(leg.theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta3_pwm = map(leg.theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);

  leg.pwm->setPWM(leg.servo0, 0, leg.theta1_pwm);
  delay(5);
  leg.pwm->setPWM(leg.servo1, 0, leg.theta2_pwm);
  delay(5);
  leg.pwm->setPWM(leg.servo2, 0, leg.theta3_pwm);
  delay(5);
}





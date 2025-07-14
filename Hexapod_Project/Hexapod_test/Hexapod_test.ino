#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVO_MIN 150
#define SERVO_MAX 600

const float link_1 = 70.0;  // mm
const float link_2 = 137.0; // mm

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second board

// Leg structure definition
struct Leg {
  int servo0; // Shoulder yaw (theta1)
  int servo1; // Shoulder pitch (theta2)
  int servo2; // Elbow (theta3)

  Adafruit_PWMServoDriver* pwm; // Pointer to the correct board

  int x, y, z;
  double c, L;

  int theta1_deg, theta2_deg, theta3_deg;
  int theta1_pwm, theta2_pwm, theta3_pwm;
};

// Define four legs and assign their PWM channels
Leg leg1 = {12, 13, 14, &pwm2};  // front left
Leg leg2 = {4, 5, 6, &pwm2}; // right middle
Leg leg3 = {0, 1, 2, &pwm1}; // rear left
Leg leg4 = {8, 9, 10, &pwm2}; // front right
Leg leg5 = {0, 1, 2, &pwm2}; // left middle
Leg leg6 = {8, 9, 10, &pwm1}; // rear right

Leg* allLegs[6] = { &leg1, &leg2, &leg3, &leg4, &leg5, &leg6 };

// Inverse Kinematics for one leg
void computeIK(Leg &leg, int x, int y, int z) {
  leg.x = x;
  leg.y = y;
  leg.z = z;

  leg.L = sqrt(x * x + z * z);
  leg.c = sqrt(leg.L * leg.L + y * y);

  double theta1_rad = atan2(x, z);
  double theta2_rad = acos((pow(link_1, 2) + pow(leg.c, 2) - pow(link_2, 2)) / (2 * link_1 * leg.c)) + atan2(y, leg.L);
  double theta3_rad = acos((pow(link_1, 2) + pow(link_2, 2) - pow(leg.c, 2)) / (2 * link_1 * link_2));

  leg.theta1_deg = theta1_rad * (180.0 / M_PI);
  leg.theta2_deg = theta2_rad * (180.0 / M_PI);
  leg.theta3_deg = 180 - (theta3_rad * (180.0 / M_PI));
}

// Move servos for one leg
void moveLeg(Leg &leg) {
  leg.theta1_pwm = map(leg.theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta2_pwm = map(leg.theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta3_pwm = map(leg.theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  // first board
  leg.pwm->setPWM(leg.servo0, 0, leg.theta1_pwm);
  delay(20);
  leg.pwm->setPWM(leg.servo1, 0, leg.theta2_pwm);
  delay(20);
  leg.pwm->setPWM(leg.servo2, 0, leg.theta3_pwm);
  delay(20);

}

void setup() {
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  delay(10);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  delay(10);

  // Initialize all legs to neutral position
  for (int i = 0; i < 6; i++) {
    computeIK(*allLegs[i], 150, -90, 0);
    moveLeg(*allLegs[i]);
  }
  delay(2000);
}

void loop() {

    
  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -20 + t * 20;              // Forward swing
    int y = -90 + 20 * sin(M_PI * t);           // Lift arc
    computeIK(leg1, 150, y, z);
    moveLeg(leg1);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -20 + t * 20;              // Forward swing
    int y = -90 + 20 * sin(M_PI * t);           // Lift arc
    computeIK(leg2, 150, y, -z);
    moveLeg(leg2);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -20 + t * 20;              // Forward swing
    int y = -90 + 20 * sin(M_PI * t);           // Lift arc
    computeIK(leg3, 150, y, z);
    moveLeg(leg3);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -20 + t * 20;              // Forward swing
    int y = -90 + 20 * sin(M_PI * t);           // Lift arc
    computeIK(leg4, 150, y, -z);
    moveLeg(leg4);
    delay(10);
  }


  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -20 + t * 20;              // Forward swing
    int y = -90 + 20 * sin(M_PI * t);           // Lift arc
    computeIK(leg5, 150, y, z);
    moveLeg(leg5);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -20 + t * 20;              // Forward swing
    int y = -90 + 20 * sin(M_PI * t);           // Lift arc
    computeIK(leg6, 150, y, -z);
    moveLeg(leg6);
    delay(10);
  }


  // Pull phase: all legs push together
    computeIK(leg1, 150, -90, -20);
    moveLeg(leg1);
    delay(10);
    computeIK(leg2, 150, -90, 20);
    moveLeg(leg2);
    delay(10);
    computeIK(leg3, 150, -90, -20);
    moveLeg(leg3);
    delay(10);
    computeIK(leg4, 150, -90, 20);
    moveLeg(leg4);
    delay(10);
    computeIK(leg5, 150, -90, -20);
    moveLeg(leg5);
    delay(10);
    computeIK(leg6, 150, -90, 20);
    moveLeg(leg6);
    delay(10);


  delay(300); // Pause before next cycle
}


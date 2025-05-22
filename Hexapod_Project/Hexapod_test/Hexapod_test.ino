#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVO_MIN 150
#define SERVO_MAX 600

const float link_1 = 70.0;  // mm
const float link_2 = 137.0; // mm

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Leg structure definition
struct Leg {
  int servo0; // Shoulder yaw (theta1)
  int servo1; // Shoulder pitch (theta2)
  int servo2; // Elbow (theta3)

  int x, y, z;
  double c, L;

  int theta1_deg, theta2_deg, theta3_deg;
  int theta1_pwm, theta2_pwm, theta3_pwm;
};

// Define four legs and assign their PWM channels
Leg legs[4] = {
  {0, 1, 2},   // Front Left
  {3, 4, 5},   // Front Right
  {6, 7, 8},   // Back Left
  {9, 10, 11}  // Back Right
};

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

  pwm.setPWM(leg.servo0, 0, leg.theta1_pwm);
  delay(20);
  pwm.setPWM(leg.servo1, 0, leg.theta2_pwm);
  delay(20);
  pwm.setPWM(leg.servo2, 0, leg.theta3_pwm);
  delay(20);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Initialize all legs to neutral position
  for (int i = 0; i < 4; i++) {
    int z_direction = (i < 2) ? 1 : -1;
    computeIK(legs[i], 70.0, -130, 0 * z_direction);
    moveLeg(legs[i]);
  }
  delay(2000);
}

void loop() {
  // Example movement: move each leg forward individually
  for (int i = 0; i < 4; i++) {
    int z_direction = (i < 2) ? 1 : -1; // Front legs use +Z, back legs use -Z
    for (float t = 0; t <= 1.0; t += 0.05) {
      int z = (1 - t) * -50 + t * 50;              // Forward swing
      int y = -130 + 40 * sin(M_PI * t);           // Lift arc
      computeIK(legs[i], 70.0, y, z * z_direction);
      moveLeg(legs[i]);
      delay(20);
    }
  }

  // Pull phase: all legs push together
  for (int i = 0; i < 4; i++) {
    int z_direction = (i < 2) ? 1 : -1;
    computeIK(legs[i], 70.0, -130, -50 * z_direction);
    moveLeg(legs[i]);
  }

  delay(300); // Pause before next cycle
}



#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 150
#define SERVO_MAX 600

const float link_1 = 90;
const float link_2 = 70;

int x = 0;
int y = 0;
int z = 0;
double c = 0;
double L = 0;

double theta1_rad, theta2_rad, theta3_rad;
int theta1_deg, theta2_deg, theta3_deg;
int theta1, theta2, theta3;

// Define your joystick analog pins
const int joyXPin = 15;  // example GPIO pin for X-axis
const int joyYPin = 4;  // example GPIO pin for Y-axis

// IK function
void IK(int x, int y, int z) {
  // calculating theta 1
  theta1_rad = atan2(x, z);

  // calculating the distance from x-z origin
  L = sqrt(x * x + z * z);

  //calculation for hypotenuse
  c = sqrt(L * L + y * y);
  
  // calculations for theta 2 and theta 3
  theta2_rad = acos( ( (link_1 * link_1)  + (c * c) - (link_2 * link_2) ) / (2 * link_1 * c) ) + atan2(y, L);
  
  theta3_rad = acos( ( (link_1 * link_1) + (link_2 * link_2) - (c * c) ) / (2 * link_1 * link_2) );

  // Converting theta 1 & 2 to degrees from rads
  theta1_deg = theta1_rad * (180/M_PI);

  theta2_deg = theta2_rad * (180/M_PI);

  theta3_deg = theta3_rad * (180/M_PI);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Joystick Control Mode");

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  pwm.setPWM(0, 0, 375);
  delay(1000);
  pwm.setPWM(1, 0, 375);
  delay(1000);
  pwm.setPWM(2, 0, 375);
  delay(1000);
}

void loop() {
  int rawX = analogRead(joyXPin);
  int rawY = analogRead(joyYPin);

  // Map raw joystick values (0–4095) to desired motion range
  x = map(rawX, 0, 4095, -158, 158);  // Left-Right
  y = map(rawY, 0, 4095, -68, 158);   // Up-Down (adjust for your leg)

  z = 0;  // Keep Z constant for now (straight forward)

  IK(x, y, z);

  theta1 = map(theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  theta2 = map(theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  theta3 = map( 180 - theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);

  Serial.print("Theta 1: ");
  Serial.print(theta1_deg);
  Serial.print(" | Theta 2: ");
  Serial.print(theta2_deg);
  Serial.print(" | Theta 3: ");
  Serial.print(theta3_deg);
  Serial.print(" | C: ");
  Serial.print(c);
  // Debug output
  Serial.print(" X = "); Serial.print(x);
  Serial.print(" | Y = "); Serial.print(y);
  Serial.print(" | Z = "); Serial.print(z);
  Serial.print(" | C = "); Serial.println(c);

  if (c < (link_1 + link_2) && c > abs(link_1 - link_2)) {
    pwm.setPWM(0, 0, theta1);
    delay(50);
    pwm.setPWM(1, 0, theta2);
    delay(50);
    pwm.setPWM(2, 0, theta3);
    delay(50);
  } else {
    Serial.println("Point out of reach. Moving to center.");
    pwm.setPWM(0, 0, 375);
    pwm.setPWM(1, 0, 375);
    pwm.setPWM(2, 0, 375);
  }


  delay(50);  // Control loop speed
}

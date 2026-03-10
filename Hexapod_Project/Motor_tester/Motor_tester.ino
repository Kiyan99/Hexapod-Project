#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second board


struct Leg {
  int servo0; // Shoulder yaw (theta1)
  int servo1; // Shoulder pitch (theta2)
  int servo2; // Elbow (theta3)
  Adafruit_PWMServoDriver* pwm; // Pointer to the correct board
};


// Define four legs and assign their PWM channels
Leg leg1 = {12, 13, 14, &pwm2};  // front left
Leg leg2 = {4, 5, 6, &pwm2}; // right middle
Leg leg3 = {0, 1, 2, &pwm1}; // rear left
Leg leg4 = {8, 9, 10, &pwm2}; // front right
Leg leg5 = {0, 1, 2, &pwm2}; // left middle
Leg leg6 = {8, 9, 10, &pwm1}; // rear right

Leg* allLegs[6] = { &leg1, &leg2, &leg3, &leg4, &leg5, &leg6 };

void setup() {
  // put your setup code here, to run once:
    // Start the pwm function
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  delay(10);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  delay(10);


}

void loop() {
  // put your main code here, to run repeatedly:
  // DS3218 motor go from 100 to 600 (270 degrees)

  pwm1.setPWM(0, 0, 268);
  delay(100);
  pwm1.setPWM(1, 0, 268);
  delay(100);
  pwm1.setPWM(2, 0, 268);
  delay(100);
  pwm1.setPWM(3, 0, 268);
  delay(100);
  pwm1.setPWM(4, 0, 268);
  delay(100);
  pwm1.setPWM(5, 0, 268);
  delay(100);
  pwm1.setPWM(6, 0, 268);
  delay(100);
  pwm1.setPWM(7, 0, 268);
  delay(100);
  pwm1.setPWM(8, 0, 268);
  delay(100);
  pwm1.setPWM(9, 0, 268);
  delay(100);
  pwm1.setPWM(10, 0, 268);
  delay(100);
  pwm1.setPWM(11, 0, 268);
  delay(100);
  pwm1.setPWM(12, 0, 268);
  delay(100);
  pwm1.setPWM(13, 0, 268);
  delay(100);
  pwm1.setPWM(14, 0, 268);
  delay(100);
  pwm1.setPWM(15, 0, 268);
  delay(100);

  pwm2.setPWM(0, 0, 268);
  delay(100);
  pwm2.setPWM(1, 0, 268);
  delay(100);
  pwm2.setPWM(2, 0, 268);
  delay(100);
  pwm2.setPWM(3, 0, 268);
  delay(100);
  pwm2.setPWM(4, 0, 268);
  delay(100);
  pwm2.setPWM(5, 0, 268);
  delay(100);
  pwm2.setPWM(6, 0, 268);
  delay(100);
  pwm2.setPWM(7, 0, 268);
  delay(100);
  pwm2.setPWM(8, 0, 268);
  delay(100);
  pwm2.setPWM(9, 0, 268);
  delay(100);
  pwm2.setPWM(10, 0, 268);
  delay(100);
  pwm2.setPWM(11, 0, 268);
  delay(100);
  pwm2.setPWM(12, 0, 268);
  delay(100);
  pwm2.setPWM(13, 0, 268);
  delay(100);
  pwm2.setPWM(14, 0, 268);
  delay(100);
  pwm2.setPWM(15, 0, 268);
  delay(100);

}

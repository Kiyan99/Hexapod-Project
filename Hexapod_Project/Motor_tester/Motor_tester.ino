#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() {
  // put your setup code here, to run once:
    // Start the pwm function
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo frequency: 50 Hz
  delay(10); // 10ms delay


}

void loop() {
  // put your main code here, to run repeatedly:

  // Leg 1 set to 90 degrees
  pwm.setPWM(0, 0, 375);
  delay(50);
  pwm.setPWM(1, 0, 150);
  delay(50);
  pwm.setPWM(2, 0, 375);
  delay(50);


  pwm.setPWM(3, 0, 375);
  delay(50);
  pwm.setPWM(4, 0, 150);
  delay(50);
  pwm.setPWM(5, 0, 375);
  delay(50);


  pwm.setPWM(6, 0, 375);
  delay(50);
  pwm.setPWM(7, 0, 150);
  delay(50);
  pwm.setPWM(8, 0, 375);
  delay(50);


  pwm.setPWM(9, 0, 375);
  delay(50);
  pwm.setPWM(10, 0, 150);
  delay(50);
  pwm.setPWM(11, 0, 375);
  delay(50);


}

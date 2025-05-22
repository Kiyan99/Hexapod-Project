#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 150
#define SERVO_MAX 600

const float link_1 = 70;
const float link_2 = 137;

int x = 0;
int y = 0;
int z = 0;
double c = 0;
double L = 0;

double theta1_rad, theta2_rad, theta3_rad;
int theta1_deg, theta2_deg, theta3_deg;
int theta1, theta2, theta3;

// IK function
void IK(int x, int y, int z) {
  theta1_rad = atan2(x, z);  // Rotation around shoulder

  L = sqrt(x * x + z * z);   // Distance in X-Z plane
  c = sqrt(L * L + y * y);   // Hypotenuse to foot tip

  theta2_rad = acos((pow(link_1, 2) + pow(c, 2) - pow(link_2, 2)) / (2 * link_1 * c)) + atan2(y, L);
  theta3_rad = acos((pow(link_1, 2) + pow(link_2, 2) - pow(c, 2)) / (2 * link_1 * link_2));

  theta1_deg = theta1_rad * (180.0 / M_PI);
  theta2_deg = theta2_rad * (180.0 / M_PI);
  theta3_deg = theta3_rad * (180.0 / M_PI);
}

void setup() {
  Serial.begin(9600);
  Serial.println("PCA9685 Servo Test - Manual X Y Z Input");

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);


}

void loop() {

  //   // Leg 1 set to 90 degrees
  // pwm.setPWM(0, 0, 375);
  // delay(50);
  // pwm.setPWM(1, 0, 150);
  // delay(50);
  // pwm.setPWM(2, 0, 375);
  // delay(50);


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



  if (Serial.available()) {
    Serial.println("Enter coordinates (X Y Z separated by spaces):");

    while (Serial.available() == 0) {}

    x = Serial.parseInt();
    y = Serial.parseInt();
    z = Serial.parseInt();

    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.print("Received coordinates: X = ");
    Serial.print(x);
    Serial.print(" Y = ");
    Serial.print(y);
    Serial.print(" Z = ");
    Serial.println(z);

    IK(x, y, z);

    theta1 = map(theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
    theta2 = map(theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
    theta3 = map(theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);

    Serial.print("Theta 1: ");
    Serial.print(theta1_deg);
    Serial.print(" | Theta 2: ");
    Serial.print(theta2_deg);
    Serial.print(" | Theta 3: ");
    Serial.print(theta3_deg);
    Serial.print(" | C: ");
    Serial.println(c);

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
  }
}


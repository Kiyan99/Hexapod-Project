#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN  150 // Pulse length for 0 degrees
#define SERVO_MAX  600 // Pulse length for 180 degrees

const float link_1 = 60; // Length of first link in cm
const float link_2 = 82; // Length of second link in cm


int x = 0;  // x axis value
int y = 0;  // y axis value 
int z = 0;  // z axis value
double c = 0; // Hypotenuse
double L = 0; // distance from x-z origin


double theta1_rad, theta2_rad, theta3_rad;  // theta 1 & 2 & 3 in radians set as doubles for more accuracy

int  theta1_deg, theta2_deg, theta3_deg;  // theta 1 & 2 & 3 in degrees

int theta1, theta2, theta3; // theta 1 & 2 & 3 which will be in PMW


// IK function declaration and implementation
void IK(int x,int y, int z) {

  // calculating theta 1
  theta1_rad = atan2(x, z);

  // calculating the distance from x-z origin
  L = sqrt(x * x + z * z);

  //calculation for hypotenuse
  c = sqrt(L * L + y * y);
  
  // calculations for theta 2 and theta 3
  theta2_rad = acos( ( pow(link_1, 2) + pow(c, 2) - pow(link_2, 2) ) / (2 * link_1 * c) ) + atan2(y, L);
  
  theta3_rad = acos( ( pow(link_1, 2) + pow(link_2, 2) - pow(c, 2) ) / (2 * link_1 * link_2) );

  // Converting theta 1 & 2 to degrees from rads
  theta1_deg = theta1_rad * (180/M_PI);

  theta2_deg = theta2_rad * (180/M_PI);

  theta3_deg = theta3_rad * (180/M_PI);

}


float stepcycle[4][3] = {
  {100, -70, -100},
  {100, 0, -100},
  {100, 0, 100},
  {100, -70, 100}
};



void setup() {
  // start the Serial port
  Serial.begin(9600);
  Serial.println("PCA9685 Servo Test"); // Print text

  // Start the pwm function
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo frequency: 50 Hz
  delay(10); // 10ms delay

    
  pwm.setPWM(0, 0, 375);
  delay(1000);
  pwm.setPWM(1, 0, 375);
  delay(1000);
  pwm.setPWM(2, 0, 375);
  delay(1000);
}

void loop() {


  for ( int i = 0; i < 4; i++) {

    x = stepcycle[i][0];
    y = stepcycle[i][1];
    z = stepcycle[i][2];

    IK(x, y, z); // call the function IK

    // map theta 1 & 2 & 3 from degrees to pwm
    theta1 = map(theta1_deg, 0, 180, 150, 600);
    theta2 = map(theta2_deg, 0, 180, 150, 600);
    theta3 = map(theta3_deg, 0, 180, 150, 600);

    pwm.setPWM(0, 0, theta1);
    delay(100);
    pwm.setPWM(1, 0, theta2);
    delay(100);
    pwm.setPWM(2, 0, theta3);
    delay(100);


    Serial.print("X = ");
    Serial.print(x);
    Serial.print(" Y = ");
    Serial.print(y);
    Serial.print(" Z = ");
    Serial.print(z);
    delay(100);
    // print theta 1 & 2 and Hypotenuse
    Serial.print("  C = ");
    Serial.println(c);
    Serial.print("θ1: ");
    Serial.print(theta1_deg);
    Serial.print("  θ2: ");
    Serial.print(theta2_deg);
    Serial.print("  θ3: ");
    Serial.println(theta3_deg);

  }
  

}





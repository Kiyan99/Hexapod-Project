#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVO_MIN 150  // min value for the servos 0 degrees 
#define SERVO_MAX 600 // max value for the servos 180 degrees

const float link_1 = 70.0;  // length of link 1 in mm
const float link_2 = 137.0; // length of link 2 in mm

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First PCA9685 board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second PCA9685 board

int global_X = 100;  // This replaces all the individual global_X values
int global_y = -120; // replace all the y values
int arc = 40; // this is the value that determines the height of the stepping arc


// Leg structure definition
struct Leg {
  int servo0; // Shoulder yaw (theta1)
  int servo1; // Shoulder pitch (theta2)
  int servo2; // Elbow (theta3)

  Adafruit_PWMServoDriver* pwm; // Pointer to the correct board

  int x, y, z;  // variables for 3 axis
  double c, L; // variables for c & L

  int theta1_deg, theta2_deg, theta3_deg; // theta angles in degrees
  int theta1_pwm, theta2_pwm, theta3_pwm; // theta angles in pwm
};

// Define 6 legs and assign their PWM channels
Leg leg1 = {12, 13, 14, &pwm2};  // front left
Leg leg2 = {4, 5, 6, &pwm2}; // right middle
Leg leg3 = {0, 1, 2, &pwm1}; // rear left
Leg leg4 = {8, 9, 10, &pwm2}; // front right
Leg leg5 = {0, 1, 2, &pwm2}; // left middle
Leg leg6 = {8, 9, 10, &pwm1}; // rear right

Leg* allLegs[6] = { &leg1, &leg2, &leg3, &leg4, &leg5, &leg6 }; // one array for all 6 legs to controle all together

// Inverse Kinematics for one leg
void computeIK(Leg &leg, int x, int y, int z) {
  leg.x = x; // x value from leg structure
  leg.y = y; // y value from leg structure
  leg.z = z; // z value from leg structure

  leg.L = sqrt(x * x + z * z); // calculating L
  leg.c = sqrt(leg.L * leg.L + y * y); // calculating C

  // calculating theta 1, 2, 3 in radians
  double theta1_rad = atan2(x, z);
  double theta2_rad = acos((pow(link_1, 2) + pow(leg.c, 2) - pow(link_2, 2)) / (2 * link_1 * leg.c)) + atan2(y, leg.L); 
  double theta3_rad = acos((pow(link_1, 2) + pow(link_2, 2) - pow(leg.c, 2)) / (2 * link_1 * link_2)); 

  // converting theta 1, 2, 3 to degrees
  leg.theta1_deg = theta1_rad * (180.0 / M_PI); 
  leg.theta2_deg = theta2_rad * (180.0 / M_PI);
  leg.theta3_deg = 180 - (theta3_rad * (180.0 / M_PI));
}

// Move servos for one leg
void moveLeg(Leg &leg) {
  // mapping the theta 1, 2, 3 from degrees to pwm
  leg.theta1_pwm = map(leg.theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta2_pwm = map(leg.theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta3_pwm = map(leg.theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  // snding the pwm to each board and motor
  leg.pwm->setPWM(leg.servo0, 0, leg.theta1_pwm);
  delay(20);
  leg.pwm->setPWM(leg.servo1, 0, leg.theta2_pwm);
  delay(20);
  leg.pwm->setPWM(leg.servo2, 0, leg.theta3_pwm);
  delay(20);

}

void setup() {
  //start serial terminal
  Serial.begin(9600);
  pwm1.begin(); // start board 1
  pwm1.setPWMFreq(50); // set frequency to 50Hz for board 1
  delay(10); // wait 10ms
  pwm2.begin(); //start board 2
  pwm2.setPWMFreq(50); // set frequency to 50Hz for board 1
  delay(10);


  // Initialize all legs to neutral position
  for (int i = 0; i < 6; i++) {
    computeIK(*allLegs[i], global_X, global_y, 0);
    moveLeg(*allLegs[i]);
  }

  // print all the pwm values for theta 1, 2, 3
  Serial.print("Motor 1: ");
  Serial.print(leg1.theta1_pwm);
  Serial.print(" Motor 2: ");
  Serial.print(leg1.theta2_pwm);
  Serial.print(" Motor 3: ");
  Serial.println(leg1.theta3_pwm);


  delay(2000); // wait 2 seconds
}

void loop() {

  // repeat the for loop 20 times  
  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -50 + t * 50;              // Forward swing
    int y = global_y + 40 * sin(M_PI * t);       // Lift arc
    computeIK(leg1, global_X, y, z);             // call computeIK to compute the angles 
    moveLeg(leg1);                               // call moveleg function to move legs
    delay(10);                                   // wait 10ms
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -50 + t * 50;              // Forward swing
    int y = global_y + 40 * sin(M_PI * t);       // Lift arc
    computeIK(leg2, global_X, y, -z);
    moveLeg(leg2);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -50 + t * 50;              // Forward swing
    int y = global_y + 40 * sin(M_PI * t);           // Lift arc
    computeIK(leg3, global_X, y, z);
    moveLeg(leg3);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -50 + t * 50;              // Forward swing
    int y = global_y + 40 * sin(M_PI * t);           // Lift arc
    computeIK(leg4, global_X, y, -z);
    moveLeg(leg4);
    delay(10);
  }


  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -50 + t * 50;              // Forward swing
    int y = global_y + 40 * sin(M_PI * t);           // Lift arc
    computeIK(leg5, global_X, y, z);
    moveLeg(leg5);
    delay(10);
  }

  for (float t = 0; t <= 1.0; t += 0.05) {
    int z = (1 - t) * -50 + t * 50;              // Forward swing
    int y = global_y + 40 * sin(M_PI * t);           // Lift arc
    computeIK(leg6, global_X, y, -z);
    moveLeg(leg6);
    delay(10);
  }


  // Pull phase: all legs pull back together to move the body forward
    computeIK(leg1, global_X, global_y, -20);
    moveLeg(leg1);
    delay(10);
    computeIK(leg2, global_X, global_y, 20);
    moveLeg(leg2);
    delay(10);
    computeIK(leg3, global_X, global_y, -20);
    moveLeg(leg3);
    delay(10);
    computeIK(leg4, global_X, global_y, 20);
    moveLeg(leg4);
    delay(10);
    computeIK(leg5, global_X, global_y, -20);
    moveLeg(leg5);
    delay(10);
    computeIK(leg6, global_X, global_y, 20);
    moveLeg(leg6);
    delay(10);


  delay(300); // Pause for 300ms before next cycle
}





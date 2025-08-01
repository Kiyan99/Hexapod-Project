#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVO_MIN 150
#define SERVO_MAX 600

const float link_1 = 70.0;  // mm
const float link_2 = 137.0; // mm

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second board

int global_x = 100;  // This replaces all the individual global_x values
int global_y = -120; // replace all the y values
int arc = 40; // this is the value that determines the height of the stepping arc


// Leg structure definition
struct Leg {
  int servo0; // Shoulder yaw (theta1)
  int servo1; // Shoulder pitch (theta2)
  int servo2; // Elbow (theta3)

  Adafruit_PWMServoDriver* pwm; // Pointer to the correct board

  int x, y, z;  // x, y, z axis values
  double c, L;  // c, L values as double to store more accurate number

  int theta1_deg, theta2_deg, theta3_deg;  // theta angles in degrees
  int theta1_pwm, theta2_pwm, theta3_pwm;  // theta angles in PWM
};

// Define four legs and assign their PWM channels
Leg leg1 = {12, 13, 14, &pwm2};  // front left
Leg leg2 = {4, 5, 6, &pwm2}; // right middle
Leg leg3 = {0, 1, 2, &pwm1}; // rear left
Leg leg4 = {8, 9, 10, &pwm2}; // front right
Leg leg5 = {0, 1, 2, &pwm2}; // left middle
Leg leg6 = {8, 9, 10, &pwm1}; // rear right

// Storing all legs in one array
Leg* allLegs[6] = { &leg1, &leg2, &leg3, &leg4, &leg5, &leg6 };

// Inverse Kinematics for one leg
void computeIK(Leg &leg, int x, int y, int z) {
  leg.x = x;
  leg.y = y;
  leg.z = z;

  leg.L = sqrt(x * x + z * z);
  leg.c = sqrt(leg.L * leg.L + y * y);
  
  // Calculating theta angles in radians
  double theta1_rad = atan2(x, z);
  double theta2_rad = acos((pow(link_1, 2) + pow(leg.c, 2) - pow(link_2, 2)) / (2 * link_1 * leg.c)) + atan2(y, leg.L);
  double theta3_rad = acos((pow(link_1, 2) + pow(link_2, 2) - pow(leg.c, 2)) / (2 * link_1 * link_2));

  // Converting theta angles to degrees
  leg.theta1_deg = theta1_rad * (180.0 / M_PI);
  leg.theta2_deg = theta2_rad * (180.0 / M_PI);
  leg.theta3_deg = 180 - (theta3_rad * (180.0 / M_PI));
}

// Move servos for one leg
void moveLeg(Leg &leg) {
  // Maping theta angles from degrees to PWM
  leg.theta1_pwm = map(leg.theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta2_pwm = map(leg.theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  leg.theta3_pwm = map(leg.theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  
  // Sending PWM signals to each baord and legs 
  leg.pwm->setPWM(leg.servo0, 0, leg.theta1_pwm);
  delay(5);
  leg.pwm->setPWM(leg.servo1, 0, leg.theta2_pwm);
  delay(5);
  leg.pwm->setPWM(leg.servo2, 0, leg.theta3_pwm);
  delay(5);

}



void tripod_gait() {
   // for-loop will run 10 timmes
  for (float t = 0; t <= 1.0; t += 0.1) {
    
    // Moving forward
    int z = (1 - t) * -30 + t * 30;              // Forward swing
    int y = global_y + 30 * sin(M_PI * t);           // Lift arc


    // Front legs
    // Pulling and pushing values used by front and read legs
    int z_push = (1 - t) * 0 + t * 60;
    int z_pull = (1 - t) * 60;  
    int x_push = (1 - t) * 50 + t * 100;
    int x_pull = (1 - t) * 100 + t * 50;  
    computeIK(leg1, x_push, y, z_push);
    computeIK(leg4, x_pull, global_y, -z_pull);
    moveLeg(leg1);
    moveLeg(leg4);

    // Middle legs
    // Pulling back for middle legs
    int z_mid_pull = (1 - t) * 30 + t * -30;
    computeIK(leg2, global_x, y, -z);
    computeIK(leg5, global_x, global_y, z_mid_pull);  
    moveLeg(leg2); 
    moveLeg(leg5);     


    // Rear legs    
    computeIK(leg3, x_pull, y, -z_pull);
    computeIK(leg6, x_push, global_y, z_push);
    moveLeg(leg3);
    moveLeg(leg6);    


  }



  for (float t = 0; t <= 1.0; t += 0.1) {
    
    // Moving forward
    int z = (1 - t) * -30 + t * 30;              // Forward swing
    int y = global_y + 30 * sin(M_PI * t);           // Lift arc


    //front Legs
    int z_push = (1 - t) * 0 + t * 60;
    int z_pull = (1 - t) * 60;  
    int x_push = (1 - t) * 50 + t * 100;
    int x_pull = (1 - t) * 100 + t * 50;  
    computeIK(leg4, x_push, y, -z_push);
    computeIK(leg1, x_pull, global_y, z_pull);
    moveLeg(leg4);
    moveLeg(leg1);



    // Midlle Legs
    // Pulling back
    int z_mid_pull = (1 - t) * 30 + t * -30;
    computeIK(leg5, global_x, y, z);
    computeIK(leg2, global_x, global_y, -z_mid_pull);
    moveLeg(leg5);
    moveLeg(leg2);



    // Rear legs
    computeIK(leg6, x_pull, y, z_pull);
    computeIK(leg3, x_push, global_y, -z_push);
    moveLeg(leg6);
    moveLeg(leg3);

  }
}






void turn_right(){


  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * -30 + t * 30;              // Forward swing
    int y = global_y + 30 * sin(M_PI * t);           // Lift arc      

    computeIK(leg1, global_x, y, z);
    moveLeg(leg1);
    computeIK(leg2, global_x, y, z);
    moveLeg(leg2);
    computeIK(leg3, global_x, y, z);
    moveLeg(leg3);


    int z_pull = (1 - t) * 30 + t * -30;
    computeIK(leg4, global_x, global_y, z_pull);
    moveLeg(leg4);
    computeIK(leg5, global_x, global_y, z_pull);
    moveLeg(leg5);
    computeIK(leg6, global_x, global_y, z_pull);
    moveLeg(leg6);


  }



  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * -30 + t * 30;              // Forward swing
    int y = global_y + 30 * sin(M_PI * t);           // Lift arc      

    computeIK(leg4, global_x, y, z);
    moveLeg(leg4);
    computeIK(leg5, global_x, y, z);
    moveLeg(leg5);
    computeIK(leg6, global_x, y, z);
    moveLeg(leg6);


    int z_pull = (1 - t) * 30 + t * -30;
    computeIK(leg1, global_x, global_y, z_pull);
    moveLeg(leg1);
    computeIK(leg2, global_x, global_y, z_pull);
    moveLeg(leg2);
    computeIK(leg3, global_x, global_y, z_pull);
    moveLeg(leg3);


  }
}






void turn_left(){

  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * 30 + t * -30;              // Forward swing
    int y = global_y + 30 * sin(M_PI * t);           // Lift arc      

    computeIK(leg1, global_x, y, z);
    moveLeg(leg1);
    computeIK(leg2, global_x, y, z);
    moveLeg(leg2);
    computeIK(leg3, global_x, y, z);
    moveLeg(leg3);


    int z_pull = (1 - t) * -30 + t * 30;
    computeIK(leg4, global_x, global_y, z_pull);
    moveLeg(leg4);
    computeIK(leg5, global_x, global_y, z_pull);
    moveLeg(leg5);
    computeIK(leg6, global_x, global_y, z_pull);
    moveLeg(leg6);


  }



  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * 30 + t * -30;              // Forward swing
    int y = global_y + 30 * sin(M_PI * t);           // Lift arc      

    computeIK(leg4, global_x, y, z);
    moveLeg(leg4);
    computeIK(leg5, global_x, y, z);
    moveLeg(leg5);
    computeIK(leg6, global_x, y, z);
    moveLeg(leg6);


    int z_pull = (1 - t) * -30 + t * 30;
    computeIK(leg1, global_x, global_y, z_pull);
    moveLeg(leg1);
    computeIK(leg2, global_x, global_y, z_pull);
    moveLeg(leg2);
    computeIK(leg3, global_x, global_y, z_pull);
    moveLeg(leg3);


  }

}



void setup() {
  // initialise boards and serial monitor
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  delay(10);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  delay(10);


  // Initialize all legs to neutral position
  for (int i = 0; i < 6; i++) {
    computeIK(*allLegs[i], global_x, global_y, 0);
    moveLeg(*allLegs[i]);
  }

  delay(4000);
}

void loop() {

  //unsigned long t0;

  // // Walk forward for 5 seconds
  // t0 = millis();
  // while (millis() - t0 < 5000) {
  //   tripod_gait();
  // }
  // delay(500);

  turn_left();

}


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVO_MIN 150
#define SERVO_MAX 600

const float link_1 = 61.0;  // mm
const float link_2 = 145.0; // mm
int max_len = link_1 + link_2; // This calculates the maximum length of the leg
const float d = 61.0; // offset in X direction

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second board

int global_x = 100;  // This replaces all the individual global_x values
int global_y = -140; // replace all the y values
int arc = 30; // this is the value that determines the height of the stepping arc


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
Leg leg6 = {4, 5, 6, &pwm1}; // rear right

// Storing all legs in one array
Leg* allLegs[6] = { &leg1, &leg2, &leg3, &leg4, &leg5, &leg6 };

// Inverse Kinematics for one leg
void computeIK(Leg &leg, int x, int y, int z) {
  leg.x = x;
  leg.y = y;
  leg.z = z;

  leg.L = sqrt(x * x + z * z);
  double theta1_rad = atan2(x, z);

  int x_p = leg.L - d;
  leg.c = sqrt(x_p * x_p + y * y);
  
  // Calculating theta angles in radians
  double theta2_rad = acos((pow(link_1, 2) + pow(leg.c, 2) - pow(link_2, 2)) / (2 * link_1 * leg.c)) + atan2(y, leg.L);
  double theta3_rad = acos((pow(link_1, 2) + pow(link_2, 2) - pow(leg.c, 2)) / (2 * link_1 * link_2));

  // Converting theta angles to degrees
  leg.theta1_deg = theta1_rad * (180.0 / M_PI);
  leg.theta2_deg = theta2_rad * (180.0 / M_PI);
  leg.theta3_deg = theta3_rad * (180.0 / M_PI);
  //leg.theta3_deg = 180 - (theta3_rad * (180.0 / M_PI));
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



void tripod_forward() {

  // for-loop will run 10 timmes
  // Phase 1
  for (float t = 0; t <= 1.0; t += 0.1) {
    
    // Moving forward
    int z = (1 - t) * -15 + t * 15;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);       // Lift arc


    // Front legs
    // Pulling and pushing values used by front and read legs
    int z_push = (1 - t) * 0 + t * 30;
    int z_pull = (1 - t) * 30;  
    int x_push = (1 - t) * 50 + t * global_x;
    int x_pull = (1 - t) * global_x + t * 50;  
    computeIK(leg1, x_push, y, z_push);
    moveLeg(leg1);

    computeIK(leg4, x_pull, global_y, -z_pull);
    moveLeg(leg4);

    // Middle legs
    // Pulling back for middle legs
    int z_mid_pull = (1 - t) * 15 + t * -15;
    computeIK(leg2, global_x, y, -z);
    moveLeg(leg2); 

    computeIK(leg5, global_x, global_y, z_mid_pull);  
    moveLeg(leg5);     


    // Rear legs    
    computeIK(leg3, x_pull, y, -z_pull);
    moveLeg(leg3);

    computeIK(leg6, x_push, global_y, z_push);
    moveLeg(leg6);    


  }


  // Phase 2
  for (float t = 0; t <= 1.0; t += 0.1) {
    
    // Moving forward
    int z = (1 - t) * -15 + t * 15;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc


    //front Legs
    int z_push = (1 - t) * 0 + t * 30;
    int z_pull = (1 - t) * 30;  
    int x_push = (1 - t) * 50 + t * global_x;
    int x_pull = (1 - t) * global_x + t * 50;  
    computeIK(leg4, x_push, y, -z_push);
    moveLeg(leg4);

    computeIK(leg1, x_pull, global_y, z_pull);
    moveLeg(leg1);



    // Midlle Legs
    // Pulling back
    int z_mid_pull = (1 - t) * 15 + t * -15;
    computeIK(leg5, global_x, y, z);
    moveLeg(leg5);

    computeIK(leg2, global_x, global_y, -z_mid_pull);
    moveLeg(leg2);



    // Rear legs
    computeIK(leg6, x_pull, y, z_pull);
    moveLeg(leg6);

    computeIK(leg3, x_push, global_y, -z_push);
    moveLeg(leg3);

  }
}


void tripod_revers(){
  // In this function legs are reversed, it is bassically walking forward with
  // front and rear legs being swapped
  // Phase 1
   // for-loop will run 10 timmes
  for (float t = 0; t <= 1.0; t += 0.1) {
    
    
    int y = global_y + arc * sin(M_PI * t);           // Lift arc


    // Front legs
    // Pulling and pushing values used by front legs
    int z_push = (1 - t) * 0 + t * 30;
    int z_pull = (1 - t) * 30;  
    int x_push = (1 - t) * 50 + t * global_x;
    int x_pull = (1 - t) * global_x + t * 50;  
    computeIK(leg6, x_push, y, z_push);
    moveLeg(leg6);

    computeIK(leg3, x_pull, global_y, -z_pull);
    moveLeg(leg3);

    // Middle legs
    // Pulling back for middle legs
    int z = (1 - t) * -15 + t * 15; 
    computeIK(leg5, global_x, y, -z);
    moveLeg(leg5); 

    computeIK(leg2, global_x, global_y, -z);  
    moveLeg(leg2);     


    // Rear legs    
    computeIK(leg4, x_pull, y, -z_pull);
    moveLeg(leg4);

    computeIK(leg1, x_push, global_y, z_push);
    moveLeg(leg1);    


  }


  // Phase 2
  for (float t = 0; t <= 1.0; t += 0.1) {
    
    // Moving forward
                  // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc


    //front Legs
    int z_push = (1 - t) * 0 + t * 30;
    int z_pull = (1 - t) * 30;  
    int x_push = (1 - t) * 50 + t * global_x;
    int x_pull = (1 - t) * global_x + t * 50;  
    computeIK(leg6, x_pull, global_y, z_pull);
    moveLeg(leg6);

    computeIK(leg3, x_push, y, -z_push);
    moveLeg(leg3);



    // Midlle Legs
    // Pulling back
    int z = (1 - t) * -15 + t * 15;
    computeIK(leg5, global_x, global_y, z);
    moveLeg(leg5);

    computeIK(leg2, global_x, y, z);
    moveLeg(leg2);



    // Rear legs
    computeIK(leg4, x_push, global_y, -z_push);
    moveLeg(leg4);

    computeIK(leg1, x_pull, y, z_pull);
    moveLeg(leg1);
    

  }

}




void turn_right(){

  // Phase 1
  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * -30 + t * 30;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc      

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


  // Phase 2
  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * -30 + t * 30;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc      

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

  // Phase 1
  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * 30 + t * -30;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc      

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


  // Phase 2
  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * 30 + t * -30;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc      

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


void crab_walk_right(){

  // Phase 1
  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * 0 + t * 20;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc      
    int x_push = (1 - t) * 70 + t * global_x; 
    int x_pull = (1 - t) * global_x + t * 70;


    computeIK(leg4, x_push, y, z);
    moveLeg(leg4);

    computeIK(leg5, x_pull, y, 0);
    moveLeg(leg5);

    computeIK(leg6, x_push, y, -z);
    moveLeg(leg6);


    computeIK(leg1, x_push, global_y, -z);
    moveLeg(leg1);

    computeIK(leg2, x_pull, global_y, 0);
    moveLeg(leg2);

    computeIK(leg3, x_push, global_y, z);
    moveLeg(leg3);

  }

  // Phase 2
  for (float t = 0; t <= 1.0; t += 0.1){


    int y = global_y + arc * sin(M_PI * t);           // Lift arc      
    int x_push = (1 - t) * 70 + t * global_x; 
    int x_pull = (1 - t) * global_x + t * 70;     
    int z_pull = (1 - t) * 20 + t * 0;

    computeIK(leg4, x_pull, global_y, z_pull);
    moveLeg(leg4);

    computeIK(leg5, x_push, global_y, 0);
    moveLeg(leg5);

    computeIK(leg6, x_pull, global_y, -z_pull);
    moveLeg(leg6);


    computeIK(leg1, x_pull, y, -z_pull);
    moveLeg(leg1);
    
    computeIK(leg2, x_push, y, 0);
    moveLeg(leg2);

    computeIK(leg3, x_pull, y, z_pull);
    moveLeg(leg3);
    

  }


}


void crab_walk_left(){

  // Phase 1
  for (float t = 0; t <= 1.0; t += 0.1){

    int z = (1 - t) * 0 + t * 20;              // Forward swing
    int y = global_y + arc * sin(M_PI * t);           // Lift arc      
    int x_push = (1 - t) * 70 + t * global_x; 
    int x_pull = (1 - t) * global_x + t * 70;

    computeIK(leg4, x_push, global_y, z);
    moveLeg(leg4);

    computeIK(leg5, x_pull, global_y, 0);
    moveLeg(leg5);

    computeIK(leg6, x_push, global_y, -z);
    moveLeg(leg6);



    computeIK(leg1, x_push, y, -z);
    moveLeg(leg1);

    computeIK(leg2, x_pull, y, 0);
    moveLeg(leg2);

    computeIK(leg3, x_push, y, z);
    moveLeg(leg3);


  }

  // Phase 2
  for (float t = 0; t <= 1.0; t += 0.1){

    int y = global_y + arc * sin(M_PI * t);           // Lift arc      
    int x_push = (1 - t) * 70 + t * global_x; 
    int x_pull = (1 - t) * global_x + t * 70;     
    int z_pull = (1 - t) * 20 + t * 0;

    computeIK(leg4, x_pull, y, z_pull);
    moveLeg(leg4);

    computeIK(leg5, x_push, y, 0);
    moveLeg(leg5);

    computeIK(leg6, x_pull, y, -z_pull);
    moveLeg(leg6);



    computeIK(leg1, x_pull, global_y, -z_pull);
    moveLeg(leg1);

    computeIK(leg2, x_push, global_y, 0);
    moveLeg(leg2);

    computeIK(leg3, x_pull, global_y, z_pull);
    moveLeg(leg3);


  }



}

// Global char variable that saves the last key entered in the Serial Monitor
char currentCommand = '\0';

void setup() {
  // initialise boards and serial monitor
  Serial.begin(115200);
    while (!Serial) {
    ; // Wait for serial connection
  }
  Serial.println("ESP32 ready. Type a key and press Enter:");

  // Initializes the two PWM boards (servo drivers) and set them to 50Hz.
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

// Checks if a character is one of your valid commands (w, s, a, d, q, e, v, x).
static inline bool isValidKey(char c) {
  return c=='w'|| c=='s'|| c=='a'|| c=='d'|| c=='q'|| c=='e'|| c=='v'|| c=='x' || c=='z';
}

void loop() {


  // Read any incoming byte(s)
  while (Serial.available() > 0) {
    char input = Serial.read();

    // Ignore line endings and any junk
    if (input == '\r' || input == '\n') continue;

    // Only update when it's a recognized command
    if (isValidKey(input)) {
      currentCommand = input;
      Serial.print("Command: "); Serial.println(currentCommand);
    } else {
      // Unknown -> ignore, keep previous command running
      Serial.print("Ignoring: "); Serial.println((int)input);
    }
  }

  // Execute current command continuously
  switch (currentCommand) {
    case 'w': tripod_forward(); break;

    case 's': tripod_revers();  break;

    case 'a': turn_left();      break;
    
    case 'd': turn_right();     break;

    case 'q': crab_walk_left(); break;

    case 'e': crab_walk_right();break;

    case 'x': 
      global_y = global_y + 5;
      global_x = global_x + 5;
      Serial.print("Decreasing height by 5mm, Y = ");
      Serial.println(global_y);

      for (int i = 0; i < 6; i++) {
        computeIK(*allLegs[i], global_x, global_y, 0);
        moveLeg(*allLegs[i]);
      }
      currentCommand = '\0';
      break;

    case 'z':
      global_y = global_y - 5;
      global_x = global_x - 5;
      Serial.print("Increasing Height by 5mm, Y = ");
      Serial.println(global_y);

      for (int i = 0; i < 6; i++) {
        computeIK(*allLegs[i], global_x, global_y, 0);
        moveLeg(*allLegs[i]);
      }
      currentCommand = '\0';
      break;

    case 'v': // neutralize all legs
      for (int i = 0; i < 6; i++) {
        computeIK(*allLegs[i], global_x, global_y, 0);
        moveLeg(*allLegs[i]);
      }
      break;

  } 

}


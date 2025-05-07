#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Create display object (for ESP32 you can define Wire pins if needed)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN  150 // Pulse length for 0 degrees
#define SERVO_MAX  600 // Pulse length for 180 degrees

const float link_1 = 90; // Length of first link in cm
const float link_2 = 82; // Length of second link in cm


int x = 100;  // x axis value
int y = 0;  // y axis value 
int z = 0;  // z axis value
double c = 0; // Hypotenuse
double L = 0; // distance from x-z origin


double theta1_rad, theta2_rad, theta3_rad;  // theta 1 & 2 & 3 in radians set as doubles for more accuracy

int  theta1_deg, theta2_deg, theta3_deg;  // theta 1 & 2 & 3 in degrees

int theta1, theta2, theta3; // theta 1 & 2 & 3 which will be in PMW


int z_start = -50;
int z_end = 50;
int y_ground = -100;
int h = 50;


// IK function declaration and implementation
void IK(int x,int y, int z) {

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
  // start the Serial port
  Serial.begin(9600);
  Serial.println("PCA9685 Servo Test"); // Print text

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Loop forever if failed
  }

  display.clearDisplay();

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

  for (float t = 0; t <= 1.0; t += 0.05){

    z = ( 1 - t ) * z_start + ( t * z_end );

    y = y_ground + ( h * sin(M_PI * t) );


    IK(x, y, z); // call the function IK

    // map theta 1 & 2 & 3 from degrees to pwm
    theta1 = map(theta1_deg, 0, 180, SERVO_MIN, SERVO_MAX);
    theta2 = map(theta2_deg, 0, 180, SERVO_MIN, SERVO_MAX);
    theta3 = map(180 - theta3_deg, 0, 180, SERVO_MIN, SERVO_MAX);

    // Clear the display first
    display.clearDisplay();

    if ( c < 160 ){
      

      pwm.setPWM(0, 0, theta1);
      delay(25);
      pwm.setPWM(1, 0, theta2);
      delay(25);
      pwm.setPWM(2, 0, theta3);
      delay(25);


      // Show the position
      display.setTextSize(1.5);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 2);
      display.print(F("X:"));
      display.print(x);
      display.print(F(" Y:"));
      display.print(y);
      display.print(F(" Z:"));
      display.print(z);

    }
    else{
      // Show out of reach message
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.println(F("Out of"));
      display.setCursor(0, 40);
      display.println(F("Reach!"));
      
    }

    // Finally update the display after drawing everything
    display.display();

  }


}





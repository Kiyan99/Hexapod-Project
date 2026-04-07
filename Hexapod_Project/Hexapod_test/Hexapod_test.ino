#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <RF24.h>
#include "Kinematics.h"
#include "Gaits.h"

Gaits gaits;

RF24 radio(16, 17); // CE & CSN
byte address[][6] = {"Node1", "Node2"};

// Global char variable that saves the last key entered in the Serial Monitor
char currentcommand = '\0';

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

  // NRF24
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.startListening(); // RX mode


  // Initialize all legs to neutral position
  for (int i = 0; i < 6; i++) {
    computeIK(*allLegs[i], global_x, global_y, 0);
    moveLeg(*allLegs[i]);
  }

  delay(4000);

}




char last_command = '0';
char command = '0';

unsigned long t_start = 0;
const unsigned long timeout = 250;

void loop() {


  if (radio.available()) {
      radio.read(&command, sizeof(command));
      Serial.print("command: ");
      Serial.println(command);
      t_start = millis();
  }

  if ( millis() - t_start > timeout){
    command = '0';
  }

  if(last_command != command){
    gaits.reset_gaits();
    last_command = command;
  }


  // Execute current command continuously
  switch (command) {
    case 'w': gaits.tripod_forward();break;

    case 's': gaits.tripod_revers();break;

    case 'a': gaits.turn_left();break;
    
    case 'd': gaits.turn_right();break;

    case 'q': gaits.crab_walk_left();break;

    case 'e': gaits.crab_walk_right();break;

    case 'x': gaits.down_up(); break;

    case 'z':
      global_y = global_y - 5;
      global_x = global_x - 5;
      Serial.print("Increasing Height by 5mm, Y = ");
      Serial.println(global_y);

      for (int i = 0; i < 6; i++) {
        computeIK(*allLegs[i], global_x, global_y, 0);
        moveLeg(*allLegs[i]);
      }
      currentcommand = '\0';
      break;

    case 'v': // neutralize all legs
      for (int i = 0; i < 6; i++) {
        computeIK(*allLegs[i], global_x, global_y, 0);
        moveLeg(*allLegs[i]);
      }
      break;

  } 

}


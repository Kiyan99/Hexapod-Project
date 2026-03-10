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

  // NRF24
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
  radio.startListening(); // RX mode


  // Initialize all legs to neutral position
  for (int i = 0; i < 6; i++) {
    computeIK(*allLegs[i], global_x, global_y, 0);
    moveLeg(*allLegs[i]);
  }

  delay(4000);

}

//Checks if a character is one of your valid commands (w, s, a, d, q, e, v, x).
static inline bool isValidKey(char c) {
  return c=='w'|| c=='s'|| c=='a'|| c=='d'|| c=='q'|| c=='e'|| c=='v'|| c=='x' || c=='z';
}

void loop() {

  // if (radio.available()) {
  //     int command;
  //     radio.read(&command, sizeof(command));
  //     Serial.print("Command: ");
  //     Serial.println(command);
  // }




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
    case 'w': gaits.tripod_forward();break;

    case 's': gaits.tripod_revers();break;

    case 'a': gaits.turn_left();break;
    
    case 'd': gaits.turn_right();break;

    case 'q': gaits.crab_walk_left();break;

    case 'e': gaits.crab_walk_right();break;

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


#include <SPI.h>
#include <RF24.h>

// Joysticks  
int J1_x = 33;
int J1_y = 34;
int J1_sw = 26;

int J2_x = 32;
int J2_y = 35;
int J2_sw = 27;

const float ADC_REF = 3.3;
const float ADC_MAX = 4095.0;
// End


// RF24
RF24 radio(16, 17); // CE & CSN

byte address[][6] = {"Node1", "Node2"};
// End

bool bodyTiltMode = false;
bool lastSw2State = HIGH;
bool sendExitOnce = false;

struct Packet {
  char command;
  float roll_voltage;
  float pitch_voltage;
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);

  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
  radio.startListening(); // RX mode

  pinMode(J1_sw, INPUT_PULLUP);
  pinMode(J2_sw, INPUT_PULLUP);

}

void loop() {
  
  radio.startListening(); // RX mode

  float x1_raw = analogRead(J1_x);
  float y1_raw = analogRead(J1_y);
  int sw1 = digitalRead(J1_sw);

  float x2_raw = analogRead(J2_x);
  float y2_raw = analogRead(J2_y);
  int sw2 = digitalRead(J2_sw);

  float x1_voltage = (x1_raw / ADC_MAX) * ADC_REF;
  float y1_voltage = (y1_raw / ADC_MAX) * ADC_REF;

  float x2_voltage = (x2_raw / ADC_MAX) * ADC_REF;
  float y2_voltage = (y2_raw / ADC_MAX) * ADC_REF;



  // if (radio.available()) {
  //   float roll = 0.0f;
  //   radio.read(&roll, sizeof(roll));
  //   Serial.print("Roll: ");
  //   Serial.println(roll);
  // }


  Packet data;
  data.command = '0';
  data.roll_voltage = y2_voltage;
  data.pitch_voltage = x2_voltage;

  // Toggle tilt mode only once per press
  if (lastSw2State == HIGH && sw2 == LOW) {
    bodyTiltMode = !bodyTiltMode;
    delay(200); // simple debounce
  }

  lastSw2State = sw2;

  if (bodyTiltMode == true) {
    // In tilt mode, only send tilt packets
    data.command = 'z';
    data.roll_voltage = y2_voltage; Serial.print("J2_Y voltage: "); Serial.print(y2_voltage);
    data.pitch_voltage = x2_voltage; Serial.print("   J2_X Pitch votage: "); Serial.println(x2_voltage);

  }
  else { 
      // Normal mode
      if (x1_voltage == 0.0f) {
        data.command = 's';
        Serial.println("Reverse command");
      }
      else if (x1_voltage > 3.0f) {
        data.command = 'w';
        Serial.println("Forward command");
      }
      else if (y1_voltage == 0.0f) {
        data.command = 'a';
        Serial.println("Left command");
      }
      else if (y1_voltage > 3.0f) {
        data.command = 'd';
        Serial.println("Right command");
      }
      else if (y2_voltage == 0.0f) {
        data.command = 'q';
        Serial.println("Crab walk left");
      }
      else if (y2_voltage > 3.0f) {
        data.command = 'e';
        Serial.println("Crab walk right");
      }
      else if (sw1 == 0) {
        data.command = 'x';
        Serial.println("Sit down");
      }
  }

  radio.stopListening();
  radio.write(&data, sizeof(data));
  delay(150);

}

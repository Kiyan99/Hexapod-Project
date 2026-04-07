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
  // put your main code here, to run repeatedly:

  int x1_raw = analogRead(J1_x);
  int y1_raw = analogRead(J1_y);
  int sw1 = digitalRead(J1_sw);

  int x2_raw = analogRead(J2_x);
  int y2_raw = analogRead(J2_y);
  int sw2 = digitalRead(J2_sw);

  int x1_voltage = (x1_raw / ADC_MAX) * ADC_REF;
  int y1_voltage = (y1_raw / ADC_MAX) * ADC_REF;

  int x2_voltage = (x2_raw / ADC_MAX) * ADC_REF;
  int y2_voltage = (y2_raw / ADC_MAX) * ADC_REF;

  radio.stopListening(); // TX mode

  if(x1_voltage == 0){
    
    char command = 's';
    radio.write(&command, sizeof(command));
    Serial.println("Revers command");
  }
  else if (x1_voltage == 3) {
    
    char command = 'w';
    radio.write(&command, sizeof(command));
    Serial.println("forward command");
  }
  else if (y1_voltage == 0) {
    
    char command = 'a';
    radio.write(&command, sizeof(command));
    Serial.println("Left command");
  }
  else if (y1_voltage == 3) {
    
    char command = 'd';
    radio.write(&command, sizeof(command));
    Serial.println("Right command");
  }
  else if (y2_voltage == 0){

    char command = 'q';
    radio.write(&command, sizeof(command));
    Serial.println("Crab walk left");
  }
  else if (y2_voltage == 3){

    char command = 'e';
    radio.write(&command, sizeof(command));
    Serial.println("Crab walk right");    
  }

  else if (sw1 == 0){

    char command = 'x';
    radio.write(&command, sizeof(command));
    Serial.println("Sit down");       
  }
  
  delay(200);

}

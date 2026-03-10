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

  if(x1_voltage != 1){
    int command = x1_voltage;
    radio.write(&command, sizeof(command));
    Serial.println("Command sent");
  }


  // Serial.print("x1: "); Serial.print(x1_voltage); Serial.print("   y1: "); Serial.print(y1_voltage); Serial.print("    sw1: "); Serial.println(sw1);

  // Serial.print("x2: "); Serial.print(x2_voltage); Serial.print("   y2: "); Serial.print(y2_voltage); Serial.print("    sw2: "); Serial.println(sw2);
  // Serial.println();
  
  delay(100);

}

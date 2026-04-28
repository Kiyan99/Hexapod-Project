#include <SPI.h>
#include <RF24.h>
#include <TFT_eSPI.h>


// LCD Display
TFT_eSPI tft = TFT_eSPI();

float rollDisplay = 0.0f;
float pitchDisplay = 0.0f;

// Joysticks  
int J1_x = 33;
int J1_y = 34;
int J1_sw = 26;

int J2_x = 32;
int J2_y = 35;
int J2_sw = 27;

const float ADC_REF = 3.3;
const float ADC_MAX = 4095.0;


float roll_voltage_smooth = 0.0f;
float pitch_voltage_smooth = 0.0f;
// End


// RF24
RF24 radio(16, 17); // CE & CSN

byte address[][6] = {"Node1", "Node2"};
// End

bool bodyTiltMode = false;
bool lastSw2State = HIGH;

struct Packet_out {
  char command;
  float roll_voltage;
  float pitch_voltage;
};


struct Packet_in {
  float roll;
  float pitch;
};

bool sit = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);

  radio.openWritingPipe(address[1]);   // send command to robot
  radio.openReadingPipe(1, address[0]); // receive IMU from robot
  radio.startListening(); // RX mode

  pinMode(J1_sw, INPUT_PULLUP);
  pinMode(J2_sw, INPUT_PULLUP);

  // Display setup

  int rollCx = 80;
  int rollCy = 120;
  int pitchCx = 240;
  int pitchCy = 120;
  int radius = 50;

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setTextSize(2);

  tft.setTextDatum(TC_DATUM);
  tft.drawString("Robot Status", 120,  20);

  tft.setTextDatum(TL_DATUM);
  tft.drawString("Roll", 40, 60);

  tft.setTextDatum(TR_DATUM);
  tft.drawString("Pitch", 200, 60);

  tft.setTextDatum(ML_DATUM);
  tft.drawString("CMD:", 10, 150);

  tft.setTextDatum(ML_DATUM);
  tft.drawString("Target Roll:", 10, 190);

  tft.setTextDatum(ML_DATUM);
  tft.drawString("Target Pitch:", 10, 260);

}

void loop() {
  

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


  radio.startListening();
  if (radio.available()) {
    Packet_in imuData;
    radio.read(&imuData, sizeof(imuData));

    rollDisplay = imuData.roll;
    pitchDisplay = imuData.pitch;
  }
  delay(150);
  radio.stopListening();


  // Toggle tilt mode only once per press
  if (lastSw2State == HIGH && sw2 == LOW) {
    bodyTiltMode = !bodyTiltMode;
    delay(100); // simple debounce
    

  }

  lastSw2State = sw2;

  Packet_out data;
  data.roll_voltage = y2_voltage;
  data.pitch_voltage = x2_voltage;



  tft.fillRect(10, 220, 240, 30, TFT_BLACK); // Clear target roll
  tft.fillRect(10, 290, 240, 30, TFT_BLACK); // Clear target pitch
  tft.fillRect(0, 90, 240, 30, TFT_BLACK); // Clear area roll and pitch
  tft.fillRect(60, 140, 200, 30, TFT_BLACK); // Clear command

  // Display Roll and Pitch values from robot
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawFloat(rollDisplay, 1, 40, 90); // Roll

  tft.setTextDatum(TR_DATUM);
  tft.drawFloat(pitchDisplay, 1, 200, 90); // Pitch

  if (bodyTiltMode == true) {
    // In tilt mode, only send tilt packets
    data.command = 'z';
    radio.write(&data, sizeof(data));

    // Smooth joystick voltages
    float alpha_v = 0.3f;

    roll_voltage_smooth  += alpha_v * (y2_voltage - roll_voltage_smooth);
    pitch_voltage_smooth += alpha_v * (x2_voltage - pitch_voltage_smooth);

    // Map voltages to target angles
    float target_roll  = (roll_voltage_smooth  / 3.3f) * 20.0f - 10.0f;
    float target_pitch = (pitch_voltage_smooth / 3.3f) * 20.0f - 10.0f;

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawFloat(target_roll, 1, 10, 220);

    tft.setTextDatum(TL_DATUM);
    tft.drawFloat(target_pitch, 1, 10, 290);

    tft.setTextDatum(MC_DATUM);
    tft.drawString("Tilt Mode", 120, 150);


  }

  

  else {

    // Normal mode
    if (x1_voltage == 0.0f) {
      data.command = 's';

      tft.setTextDatum(MC_DATUM);
      tft.drawString("Backward", 120, 150);
      radio.write(&data, sizeof(data));
    }
    else if (x1_voltage > 3.0f) {
      data.command = 'w';

      tft.setTextDatum(MC_DATUM);
      tft.drawString("Forward", 120, 150);
      radio.write(&data, sizeof(data));

    }
    else if (y1_voltage == 0.0f) {
      data.command = 'a';

      tft.setTextDatum(MC_DATUM);
      tft.drawString("Turn Left", 120, 150);
      radio.write(&data, sizeof(data));
    }
    else if (y1_voltage > 3.0f) {
      data.command = 'd';

      tft.setTextDatum(MC_DATUM);
      tft.drawString("Turn Right", 120, 150);
      radio.write(&data, sizeof(data));

    }
    else if (y2_voltage == 0.0f) {
      data.command = 'q';

      tft.setTextDatum(MC_DATUM);
      tft.drawString("C-walk left", 140, 150);
      radio.write(&data, sizeof(data));

    }
    else if (y2_voltage > 3.0f) {
      data.command = 'e';

      tft.setTextDatum(MC_DATUM);
      tft.drawString("C-walk right", 140, 150);
      radio.write(&data, sizeof(data));

    }
    else if (sw1 == 0) {
      data.command = 'x';
      sit = !sit;
      if (sit == true){

        tft.setCursor(90, 140); // Print at this postion
        tft.print("Sitting down");
        radio.write(&data, sizeof(data));
        delay(3000);

      }
      else if (sit != true){

        tft.setCursor(90, 140); // Print at this postion
        tft.print("Standing");
        radio.write(&data, sizeof(data));
        delay(3000);

      }

    }
  }


  

}

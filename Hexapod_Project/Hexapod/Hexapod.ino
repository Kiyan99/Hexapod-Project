#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <RF24.h>
#include "Kinematics.h"
#include "Gaits.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>


Adafruit_MPU6050 mpu;

// Madgwick filter state
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Filter gain
float beta = 0.08f;
// Timing
unsigned long lastMicros = 0;


// ===============================
// 6-axis Madgwick update
// gx, gy, gz in rad/s
// ax, ay, az in any accel unit
// dt in seconds
// ===============================
void MadgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float norm;
  float f1, f2, f3;
  float s0, s1, s2, s3;
  float qDot0, qDot1, qDot2, qDot3;

  // --------------------------------
  // Step 1: Normalize accelerometer
  // --------------------------------
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm <= 0.0f) return;

  ax /= norm;
  ay /= norm;
  az /= norm;

  // --------------------------------
  // Step 2: Build error function
  // predicted gravity - measured gravity
  // --------------------------------
  f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
  f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
  f3 = 1.0f - 2.0f * (q1 * q1 + q2 * q2) - az;

  // --------------------------------
  // Step 3: Gradient descent step
  // s = J^T * f
  // --------------------------------
  s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
  s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
  s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
  s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;

  // --------------------------------
  // Step 4: Normalize correction step
  // --------------------------------
  norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (norm > 0.0f) {
    s0 /= norm;
    s1 /= norm;
    s2 /= norm;
    s3 /= norm;
  }

  // --------------------------------
  // Step 5: Quaternion derivative from gyro
  // qDot = 0.5 * q ⊗ omega
  // --------------------------------
  qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  // --------------------------------
  // Step 6: Apply feedback correction
  // --------------------------------
  qDot0 -= beta * s0;
  qDot1 -= beta * s1;
  qDot2 -= beta * s2;
  qDot3 -= beta * s3;

  // --------------------------------
  // Step 7: Integrate quaternion
  // --------------------------------
  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // --------------------------------
  // Step 8: Normalize quaternion
  // --------------------------------
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (norm <= 0.0f) return;

  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

// ===============================
// Quaternion to Euler angles
// roll/pitch/yaw in radians
// ===============================
void getEulerRad(float &roll, float &pitch, float &yaw)
{
  roll = atan2(2.0f * (q0 * q1 + q2 * q3),
               1.0f - 2.0f * (q1 * q1 + q2 * q2));

  float pitchArg = 2.0f * (q0 * q2 - q3 * q1);
  if (pitchArg > 1.0f) pitchArg = 1.0f;
  if (pitchArg < -1.0f) pitchArg = -1.0f;
  pitch = asin(pitchArg);

  yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
              1.0f - 2.0f * (q2 * q2 + q3 * q3));
}



Gaits gaits;

RF24 radio(16, 17); // CE & CSN
byte address[][6] = {"Node1", "Node2"};

// Global char variable that saves the last key entered in the Serial Monitor
char currentcommand = '\0';

char last_command = '0';
char command = '0';
float roll_voltage = 0.0f;
float pitch_voltage = 0.0f;


unsigned long lastSendTime = 0;
unsigned long lastCommandTime = 0;

const unsigned long sendInterval = 100;     // send IMU every 20 ms
const unsigned long commandTimeout = 200;  // if no command for 100 ms, reset


float rollDeg = 0.0f;
float pitchDeg = 0.0f;


struct Packet_in {
  char command;
  float roll_voltage;
  float pitch_voltage;
};

struct Packet_out {
  float roll_out;
  float pitch_out;
};


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
  radio.openWritingPipe(address[0]);   // send IMU to controller
  radio.openReadingPipe(1, address[1]); // receive command from controller
  radio.startListening(); // RX mode


  // Initialize all legs to neutral position
  for (int i = 0; i < 3; i++) {
    computeIK(*allLegs[i], global_x, global_y, 0);
    moveLeg(*allLegs[i]);
  }
  delay(1000);
  for (int n = 3; n < 6; n++) {
    computeIK(*allLegs[n], global_x, global_y, 0);
    moveLeg(*allLegs[n]);
  }

  delay(4000);


  // MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }

  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 ready");
  lastMicros = micros();

}



void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Adafruit library gives:
  // accel in m/s^2
  // gyro in rad/s

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  

  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  // Small protection against weird dt spikes
  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  MadgwickUpdateIMU(gx, gy, gz, ax, ay, az, dt);

  float roll, pitch, yaw;
  getEulerRad(roll, pitch, yaw);

  float rollDeg  = roll  * 180.0f / PI;
  float pitchDeg = pitch * 180.0f / PI;

  // Serial.print("Roll: ");  Serial.print(rollDeg);
  // Serial.print("   Roll: ");  Serial.println(pitchDeg);


  unsigned long TX_timer = millis();

  if(TX_timer - lastSendTime >= sendInterval){
    lastSendTime = TX_timer;

    Packet_out imu_data;
    imu_data.roll_out = rollDeg;
    imu_data.pitch_out = pitchDeg;

    radio.stopListening();
    radio.write(&imu_data, sizeof(imu_data));
    radio.startListening();
  }


  Packet_in data;
  if (radio.available()) {
      radio.read(&data, sizeof(data));

      command = data.command;
      roll_voltage = data.roll_voltage;
      pitch_voltage = data.pitch_voltage;
      lastCommandTime = millis();


      Serial.print("Command: ");
      Serial.println(command);
     
  }


  unsigned long command_timer = millis();
  if (command_timer - lastCommandTime >= commandTimeout) {
    command = '0';
  }

  // reset only once when entering idle state
  if (command == '0' && last_command != '0') {
    gaits.reset_gaits();
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

    case 'z': gaits.tilt_control(roll_voltage, pitch_voltage, rollDeg, pitchDeg, global_x, global_y); break;
    
  } 

  last_command = command;


}


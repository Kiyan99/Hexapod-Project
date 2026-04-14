#ifndef GAITS_H
#define GAITS_H
#include <Arduino.h>
#include "Kinematics.h"

class Gaits{

  public:
    Gaits();

    void sit_down();
    void down_up();
    void tripod_forward();
    void tripod_revers();
    void turn_right();
    void turn_left();
    void crab_walk_right();
    void crab_walk_left();
    void reset_gaits();
    void tilt_control(float roll_voltage, float pitch_voltage, float rollDeg, float pitchDeg, float global_x, float global_y);


  private:
    int phase = 1;
    float t = 0.0;
    int standing = true;

    float roll_voltage_smooth = 0.0f;
    float rollDeg_smooth = 0.0f;
    float roll_correction_smooth = 0.0f;

    float pitch_voltage_smooth = 0.0f;
    float pitchDeg_smooth = 0.0f;
    float pitch_correction_smooth = 0.0f;
    
    int tran_x[6];
    int tran_y[6];
    int tran_z[6];

};

#endif
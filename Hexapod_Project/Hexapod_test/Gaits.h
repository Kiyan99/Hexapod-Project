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

  private:
    int phase = 1;
    float t = 0.0;
    int standing = true;  
    
    int tran_x[6];
    int tran_y[6];
    int tran_z[6];
};

#endif
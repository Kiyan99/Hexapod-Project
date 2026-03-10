#ifndef GAITS_H
#define GAITS_H
#include <Arduino.h>
#include "Kinematics.h"


class Gaits{

  public:
    Gaits();

    void tripod_forward();
    void tripod_revers();
    void turn_right();
    void turn_left();
    void crab_walk_right();
    void crab_walk_left();


};

#endif
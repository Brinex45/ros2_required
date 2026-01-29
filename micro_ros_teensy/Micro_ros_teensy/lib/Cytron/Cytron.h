#ifndef CYTRON_H
#define CYTRON_H

#include"Arduino.h"


class Cytron
{
private:
    int pwm,dir_pin;
    bool direction;
    int max_pwm;
public:
    Cytron(int temp_pwm,int temp_dir_pin,bool temp_direction, int temp_max_pwm);
    void rotate(int value);
    void test();
};

#endif
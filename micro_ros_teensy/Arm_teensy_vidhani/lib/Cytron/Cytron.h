#ifndef CYTRON_H
#define CYTRON_H

#include"Arduino.h"


class Cytron
{
private:
    int pwm,dir_pin;
    bool direction;
public:
    Cytron(int temp_pwm,int temp_dir_pin,bool temp_direction);
    void rotate(int value);
    void test();
};

#endif
#ifndef ARM_ARDUINO_JOINT_HPP
#define ARM_ARDUINO_JOINT_HPP

#include <string>
#include <cmath>


class Joint
{
    public:

    std::string name = "";
    float enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Joint() = default;

    Joint(const std::string &arm_name, int counts_per_rev)
    {
      setup(arm_name, counts_per_rev);
    }

    
    void setup(const std::string &arm_name, int counts_per_rev)
    {
      name = arm_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }



};


#endif // ARM_ARDUINO_WHEEL_HPP

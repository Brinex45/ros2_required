#ifndef ARM_ARDUINO_JOINT_HPP
#define ARM_ARDUINO_JOINT_HPP

#include <string>
#include <cmath>


class Joint
{
  public:

    std::string name = "";
    float enc = 0.0;
    double cmd = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    double rads_per_count = 0.0;
    double cmd_pwm = 0.0;

    double error = 0.0;
    double i_error = 0.0;
    double d_error = 0.0;
    double prev_error = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double ki = 0.0;
    double max_pwm = 0.0;
    double min_pwm = 0.0;

    Joint() = default;

    Joint(const std::string &arm_name, int counts_per_rev, double kp, double kd, double ki, double max_pwm, double min_pwm)
    {
      setup(arm_name, counts_per_rev, kp, kd, ki, max_pwm, min_pwm);
    }

    
    void setup(const std::string &arm_name, int counts_per_rev, 
      const double &kp, const double &kd, const double &ki, const double &max_pwm, const double &min_pwm)
    {
      name = arm_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
      this->kp = kp;
      this->kd = kd;
      this->ki = ki;
      this->max_pwm = max_pwm;
      this->min_pwm = min_pwm;
    }

    inline double constrain_pwm(double val) {
        if (val < min_pwm) return min_pwm;
        if (val > max_pwm) return min_pwm;
        return val;
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }

    double convert_to_degree(double rad){
      return rad * 180.0 / M_PI;
    }

    double calc_pwm(double dt){
      error = cmd - pos;

      error = convert_to_degree(error);

      i_error += error * dt;

      d_error = (error - prev_error) / dt;

      prev_error = error;

      // 5. Calculate total output
      double output = (ki * i_error) + (kp * error) + (kd * d_error);

      output = constrain_pwm(output);

      return output;
    }


};


#endif // ARM_ARDUINO_WHEEL_HPP

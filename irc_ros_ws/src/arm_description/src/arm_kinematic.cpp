#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

// #include "irc_custom_interfaces/msg/arm_angles.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "irc_custom_interfaces/msg/ps4.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

Eigen::Matrix4d rot_x(double angle) {
    Eigen::Matrix4d R(4, 4);
    R << 1, 0, 0, 0,
         0, cos(angle), -sin(angle), 0,
         0, sin(angle), cos(angle), 0,
         0, 0, 0, 1;
    return R;
}
Eigen::Matrix4d rot_y(double angle) {
    Eigen::Matrix4d R(4, 4);
    R << cos(angle), 0, sin(angle), 0,
         0, 1, 0, 0,
         -sin(angle), 0, cos(angle), 0,
         0, 0, 0, 1;
    return R;
}
Eigen::Matrix4d rot_z(double angle) {
    Eigen::Matrix4d R(4, 4);
    R << cos(angle), -sin(angle), 0, 0, 
         sin(angle), cos(angle), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return R;
}

Eigen::Matrix4d translate(double x, double y, double z) {
    Eigen::Matrix4d T(4, 4);
    T << 1, 0, 0, x,
         0, 1, 0, y,
         0, 0, 1, z,
         0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d home_matrix = [] {
    Eigen::Matrix4d H;
    H << 0, 0, 1, 125.5,
         0, 1, 0, 0,
        -1, 0, 0, 487.5,
         0, 0, 0, 1;
    return H;
}();

Eigen::Matrix4d Transform_matrix = home_matrix;

#define unit_trans 1
#define unit_rot 0.5

#define d1 37.5
#define d5 80.5
#define a2 450
#define a3 450

int8_t Left_X = 0;
int8_t Left_Y = 0;
int8_t L2 = 0;
int8_t R2 = 0;
int8_t Right_X = 0;
int8_t Right_Y = 0;

double k = 0.0001;


// float a1;
// float a2;
// float d1;
// float d2;

double x_target;
double y_target;
double z_target;
double y_rot;
double z_rot;

double prev_x_target;
double prev_y_target;
double prev_z_target;
double prev_y_rot;
double prev_z_rot;

// double theta_target;
// double prev_theta_target;

float length1 = 48;
float length2 = 45;
float length3 = 5;

float theta_1, theta_2, theta_3, theta_4, theta_5;
float theta_234;

float angle1;
float angle2;
float angle3;
float angle4;
float pos;   // z
float pos2;

float x;
float y;
float x2;
float y2;
float x3;
float y3;

bool flag = false;

inline double degrees(double rad) {
    return rad * (180.0 / M_PI);
}

inline double radians(double deg) {
    return deg * (M_PI / 180.0);
}

inline double constrain(double val, double min_val, double max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

float mapValue(float x,
               float in_min, float in_max,
               float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void mapping(){

    Left_X = mapValue(Left_X, -128, 127, -1.0, 1.0);
    Left_Y = mapValue(Left_Y, -128, 127, -1.0, 1.0);
    Right_X = mapValue(Right_X, -128, 127, -1.0, 1.0);
    Right_Y = mapValue(Right_Y, -128, 127, -1.0, 1.0);
    L2 = mapValue(L2, -128, 127, 0, 1.0);
    R2 = mapValue(R2, -128, 127, 0, 1.0);

    if (Left_X) {
        x_target += (Left_X * unit_trans);
    }
    if (Left_Y) {
        y_target += (Left_Y * unit_trans);
    }
    if (Right_Y) {
        z_target += (Right_Y * unit_trans);
    }
    if (L2 || R2) {
        y_rot = ((L2 - R2) * unit_rot);
    } else y_rot = 0;

}

class ArmKinematics : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<irc_custom_interfaces::msg::Ps4>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float64MultiArray target_angles_;

    void joint_command_callback(const irc_custom_interfaces::msg::Ps4::SharedPtr msg) {

        Left_X = msg->ps4_data_analog[0];
        Left_Y = msg->ps4_data_analog[1];

        L2 = msg->ps4_data_analog[2];
        R2 = msg->ps4_data_analog[5];

        Right_X = msg->ps4_data_analog[3];
        Right_Y = msg->ps4_data_analog[4];
        
        mapping();

        prev_y_rot = y_rot;
    }
    
    void matrix_prep() {
        double y_rad = radians(y_rot);
        double z_rad = radians(z_rot);
        
        Eigen::Matrix4d Y_new_rot = rot_y(y_rad);
        Eigen::Matrix4d Z_new_rot = rot_z(z_rad);
        
        Eigen::Matrix4d Trans = translate(x_target - prev_x_target, y_target - prev_y_target, z_target - prev_z_target);

        prev_x_target = x_target;
        prev_y_target = y_target;
        prev_z_target = z_target;

        Transform_matrix = (Transform_matrix * Y_new_rot * Z_new_rot) + Trans;
    }

    void kinematic(){
        matrix_prep();

        // float nx = Transform_matrix(0, 0);
        // float ny = Transform_matrix(1, 0);
        float nz = Transform_matrix(2, 0);

        // float sx = Transform_matrix(0, 1);
        // float sy = Transform_matrix(1, 1);
        float sz = Transform_matrix(2, 1);

        float ax = Transform_matrix(0, 2);
        float ay = Transform_matrix(1, 2);
        float az = Transform_matrix(2, 2);

        float px = Transform_matrix(0, 3);
        float py = Transform_matrix(1, 3);
        float pz = Transform_matrix(2, 3);

        theta_1 = atan2(py, px);
        theta_5 = atan2(sz, -nz);

        theta_234 = atan2(-((ax * cos(theta_1)) + (ay * sin(theta_1))), -az);

        float c = (px / cos(theta_1)) + d5 * sin(theta_234);
        float d = d1 - d5 * cos(theta_234) - pz;

        float costheta3 = (pow(c, 2) + pow(d, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
        costheta3 = constrain(costheta3, -1, 1);
        float sintheta3 = sqrt(1 - pow(costheta3, 2));

        theta_3 = atan2(sintheta3, costheta3);

        float r = a3 * cos(theta_3) + a2;
        float s = a3 * sin(theta_3);

        theta_2 = atan2(r * d - s * c, r * c + s * d);
        theta_4 = theta_234 - theta_2 - theta_3 + M_PI;

    }

    bool inside_ws() {
        if (isnan(angle1) || isnan(angle2) || isnan(angle3) || isnan(angle4) || angle1 > 80 || angle2 < 73 || angle2 > 118) 
            return false;
        else
            return true;
    }

    void publish_arm_angles() {

        kinematic();

        target_angles_.data.resize(5);
        target_angles_.data[0] = (theta_1);
        target_angles_.data[1] = (theta_2);
        target_angles_.data[2] = (theta_3);
        target_angles_.data[3] = (theta_4);
        target_angles_.data[4] = (0.0);

        publisher_->publish(target_angles_);
        
    }

public:
    ArmKinematics() : Node("arm_kinematics_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_joints_controller/commands", 10);
        
        subscriber_ = this->create_subscription<irc_custom_interfaces::msg::Ps4>(
            "ps4_data_arm",
            10,
            std::bind(&ArmKinematics::joint_command_callback, this, _1)
        );
        
        timer_ = this->create_wall_timer(
            0.01s,
            std::bind(&ArmKinematics::publish_arm_angles, this)
        );

        RCLCPP_INFO(this->get_logger(), "Arm Kinematics Node has been started.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
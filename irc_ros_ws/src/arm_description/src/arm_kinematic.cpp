#include <cmath>

#include "rclcpp/rclcpp.hpp"

// #include "irc_custom_interfaces/msg/arm_angles.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "irc_custom_interfaces/msg/ps4.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

int8_t Left_X = 0;
int8_t Left_Y = 0;
int8_t L2 = 0;
int8_t R2 = 0;
int8_t Right_X = 0;
int8_t Right_Y = 0;

double k = 0.0001;

double x_target = 55;
double y_target = 25;
double z_target = 0;
double theta_target = 0;

double prev_x_target;
double prev_y_target;
double prev_z_target;
double prev_theta_target;

float length1 = 48;
float length2 = 45;
float length3 = 5;

float d1;
float d2;
float a1;
float a2;
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
        
        if (abs(Left_X) > 20) {
            x_target += Left_X * k;
        }

        if (abs(Left_Y) > 20) {
            y_target += Left_Y * k;
        }

        // if (L2 > 20) {
        //     z_target += L2 * k;
        // }

        // if (R2 > 20) {
        //     z_target -= R2 * k;
        // }

        if (abs(Right_X) > 20) {
            theta_target += Right_X * k;
        }

        prev_x_target = x_target;
        prev_y_target = y_target;
        prev_z_target = z_target;
        prev_theta_target = theta_target;
    }

    void find_ik_2(float x, float y, float angle) {
        x3 = x - (length3 * cos(radians(angle)));
        y3 = y - (length3 * sin(radians(angle)));

        d1 = sqrt(pow(x3, 2) + pow(y3, 2));

        a1 = degrees(atan2(y3, x3));
        a2 = degrees(acos((pow(d1, 2) + pow(length1, 2) - pow(length2, 2)) / (2 * d1 * length1)));
        angle1 = (a1 + a2);

        x2 = length1 * cos(radians(angle1));
        y2 = length1 * sin(radians(angle1));
        d2 = sqrt(pow(x - x2, 2) + pow(y - y2, 2));

        angle2 = (degrees(acos((pow(length2, 2) + pow(length1, 2) - pow(d1, 2)) / (2 * length2 * length1))));

        angle3 = angle - angle1 + 180.0 - angle2;
    }

    void find_ik_3(float x, float y, float z, float theta) {
        angle4 = degrees(atan2(z, x));

        find_ik_2(sqrt(pow(x, 2) + pow(z, 2)), y, theta);

        if (inside_ws())
            flag = true;
        else {
            x_target = prev_x_target;
            y_target = prev_y_target;
            z_target = prev_z_target;
            theta_target = prev_theta_target;
            flag = false;
        }
    }

    bool inside_ws() {
        if (isnan(angle1) || isnan(angle2) || isnan(angle3) || isnan(angle4) || angle1 > 80 || angle2 < 73 || angle2 > 118) 
            return false;
        else
            return true;
    }

    void publish_arm_angles() {

        find_ik_3(x_target, y_target, z_target, theta_target);

        target_angles_.data.resize(4);
        target_angles_.data[0] = (angle1);
        target_angles_.data[1] = (angle2);
        target_angles_.data[2] = (angle3);
        target_angles_.data[3] = (angle4);

        if (flag)
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
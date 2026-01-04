#ifndef ARM_SYSTEM_HPP_
#define ARM_SYSTEM_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/byte.hpp"

#include "visibility_control.h"
#include "joint.hpp"

namespace arm
{
    class ArmHardware : public hardware_interface::SystemInterface
    {
        struct Config
        {
            std::vector<std::string> joint_names;
            std::vector<int> enc_counts_per_rev;

            std::vector<double> pid_p;
            std::vector<double> pid_d;
            std::vector<double> pid_i;
            std::vector<double> max_pwm;
            std::vector<double> min_pwm;
        };

        enum class ArmMode {
            IDLE,
            HOMING,
            CLOSED_LOOP
        };

        bool homing_done_ = false;
        ArmMode arm_mode_ = ArmMode::IDLE;

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardware)

        ARM_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ARM_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ARM_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ARM_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ARM_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        ARM_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ARM_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ARM_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ARM_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        Config cfg_;
        std::vector<Joint> joint;

        rclcpp::Node::SharedPtr hardware_node_;

        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_readings_sub_; // convert it to a custom msg for encoder data
        rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr read_kill_switch_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr encoder_reset_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmds_pub_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr check_sub_;

        std_msgs::msg::Int64MultiArray::SharedPtr encoder_readings_;
        std_msgs::msg::Byte::SharedPtr kill_switch_state_{nullptr};
        std_msgs::msg::Bool::SharedPtr encoder_reset_msg_;
        std::thread spin_thread_;                   // Thread for spinning the ROS node
        std::atomic<bool> stop_spin_thread_{false}; // Flag to stop spinning safely
        bool micro_ros_active_ = false;
        bool check_msg_ = false;
    };
} // namespace arm

inline double constrain(double val, double min_val, double max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

#endif
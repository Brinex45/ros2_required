#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToStampedNode : public rclcpp::Node
{
public:
    TwistToStampedNode()
    : Node("twist_to_stamped")
    {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/unstamped_vel", 10,
            std::bind(&TwistToStampedNode::twist_callback, this, std::placeholders::_1));

        twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/rover_base_controller/cmd_vel", 10);
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped stamped_msg;
        stamped_msg.header.stamp = this->get_clock()->now();
        stamped_msg.header.frame_id = "base_link";  // Change as needed

        stamped_msg.twist.linear.x = 3 * msg->linear.x;
        stamped_msg.twist.angular.z = 3 * msg->angular.z;

        twist_stamped_pub_->publish(stamped_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToStampedNode>());
    rclcpp::shutdown();
    return 0;
}
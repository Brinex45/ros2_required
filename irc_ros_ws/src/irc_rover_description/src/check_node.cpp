#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

class CheckerNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_check_status() {
        std_msgs::msg::Bool msg;
        msg.data = true;
        publisher_->publish(msg);
    }

public:
    CheckerNode() : Node("checker_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/comm_check", 10);

        timer_ = this->create_wall_timer(
            1.0s,
            std::bind(&CheckerNode::publish_check_status, this)
        );

        RCLCPP_INFO(this->get_logger(), "Checker Node has been started.");
    }
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
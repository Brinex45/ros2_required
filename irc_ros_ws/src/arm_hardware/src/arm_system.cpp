#include "arm_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arm{

    hardware_interface::CallbackReturn ArmHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        hardware_node_ = std::make_shared<rclcpp::Node>("arm_hardware");

        encoder_readings_sub_ = hardware_node_->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/arm_encoders", 10,
            [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg)
            {
                encoder_readings_ = msg;
                micro_ros_active_ = true;
            });

        joint_cmds_pub_ = hardware_node_->create_publisher<std_msgs::msg::Float32MultiArray>("/joints_command", 10);

        info_ = info;

        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Setup joints
        size_t n = info_.joints.size();

        joint.resize(n);

        cfg_.joint_names.resize(n);
        cfg_.enc_counts_per_rev.resize(n);
        cfg_.pid_p.resize(n);
        cfg_.pid_d.resize(n);
        cfg_.pid_i.resize(n);
        cfg_.max_pwm.resize(n);
        cfg_.min_pwm.resize(n);

        for (size_t i = 0; i < n; ++i) {
            cfg_.joint_names[i] = info_.joints[i].name;
            cfg_.enc_counts_per_rev[i] = std::stoi(info_.joints[i].parameters.at("encoder_counts_per_rev"));
            if (info_.joints[i].parameters.count("pid_p") > 0)
            {
                cfg_.pid_p[i] = std::stod(info_.joints[i].parameters.at("pid_p"));
                cfg_.pid_d[i] = std::stod(info_.joints[i].parameters.at("pid_d"));
                cfg_.pid_i[i] = std::stod(info_.joints[i].parameters.at("pid_i"));
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("RoverBaseHardware"), "PID values not supplied for %s, using defaults.", cfg_.joint_names[i].c_str());
            }

            if (info_.joints[i].parameters.count("max_pwm") > 0)
            {
                cfg_.max_pwm[i] = std::stod(info_.joints[i].parameters.at("max_pwm"));
                cfg_.min_pwm[i] = std::stod(info_.joints[i].parameters.at("min_pwm"));
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("RoverBaseHardware"), "Max and min pwm values not supplied for %s, using defaults.", cfg_.joint_names[i].c_str());
            }
        }

        // Initialize joints
        for (size_t i = 0; i < n; ++i) {
            joint[i].setup(cfg_.joint_names[i], cfg_.enc_counts_per_rev[i], cfg_.pid_p[i], cfg_.pid_d[i], cfg_.pid_i[i], cfg_.max_pwm[i], cfg_.min_pwm[i]);
        }

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // arm has exactly 2 states and 1 commands interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("ArmHardware"),
                             "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                             joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            // {
            //     RCLCPP_FATAL(rclcpp::get_logger("RoverBaseHardware"),
            //                  "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            //                  joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            //     return hardware_interface::CallbackReturn::ERROR;
            // }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(rclcpp::get_logger("RoverBaseHardware"),
                             "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("ArmHardware"),
                             "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
                             joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArmHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ArmHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArmHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                joint[i].name, hardware_interface::HW_IF_POSITION, &joint[i].pos));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                joint[i].name, hardware_interface::HW_IF_VELOCITY, &joint[i].vel));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArmHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                joint[i].name, hardware_interface::HW_IF_VELOCITY, &joint[i].cmd));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ArmHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Configuring ...please wait...");

        auto topic_name = "/communication_check"; // Change to an actual topic published by the microcontroller
        check_sub_ = hardware_node_->create_subscription<std_msgs::msg::Bool>(
            topic_name, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                check_msg_ = msg->data; // Update check_msg_ when a message is received
            });

        stop_spin_thread_ = false;
        spin_thread_ = std::thread([this]()
                                   {
                rclcpp::Rate rate(100);  // Adjust the loop rate as needed
                while (rclcpp::ok() && !stop_spin_thread_) {
                    rclcpp::spin_some(hardware_node_);
                    rate.sleep();
                } });

        rclcpp::sleep_for(std::chrono::seconds(2));

        if (check_msg_)
        {
            RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Micro-ROS node is active.");
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("ArmHardware"), "Micro-ROS node not responding.");
        }

        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Cleaning up ...please wait...");

        // Reset ROS communication handles (example: publisher/subscriber)
        encoder_readings_sub_.reset();
        joint_cmds_pub_.reset();
        check_sub_.reset();

        // Stop the spin thread safely
        stop_spin_thread_ = true;
        if (spin_thread_.joinable())
        {
            spin_thread_.join(); // Wait for the thread to finish
        }

        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Activating ...please wait...");

        rclcpp::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Micro-ROS node is active.");

        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    hardware_interface::CallbackReturn ArmHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        stop_spin_thread_ = true;
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }

        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArmHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        size_t n = info_.joints.size();

        double delta_seconds = period.seconds();
        if (delta_seconds <= 1e-6) {
            return hardware_interface::return_type::OK;
        }

        if (!encoder_readings_ || encoder_readings_->data.empty()) {
            // return hardware_interface::return_type::OK;
        }
        else{
            for (size_t i = 0; i < n; ++i) {
                joint[i].enc = encoder_readings_->data[i];
                // RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Joint %s enc data: %f", joint[i].name.c_str(), joint[i].enc);
            }
        }
        

        // if (encoder_readings_->data.size() < n) {
        //     RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),
        //                 "Encoder data size (%zu) < joints (%zu)",
        //                 encoder_readings_->data.size(), n);
        //     return hardware_interface::return_type::OK;
        // }

        for (size_t i = 0; i < n; ++i) {
            // joint[i].enc = encoder_readings_->data[i];

            double new_pos = joint[i].calc_enc_angle();
            joint[i].vel = (new_pos - joint[i].pos) / delta_seconds;
            joint[i].pos = new_pos;

        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArmHardware::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;

        size_t n = info_.joints.size();

        auto cmd_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        cmd_msg->data.resize(n);

        for (size_t i = 0; i < n; ++i) {

            // cmd_msg->data[i] = static_cast<float>(joint[i].cmd);
            
            // float pos = static_cast<float>(joint[i].cmd);
            // float speed = (joint[i].pos - pos) * (180.0f / 3.14159265f); // Convert rad to deg

            // switch (i){
            // case 0:
            //     cmd_msg->data[i] = constrain(-speed * 35, -100.0f, 100.0f);
            //     break;
            // case 1:
            //     cmd_msg->data[i] = constrain(-speed * 200, -255.0f, 255.0f);
            //     break;
            // case 2:
            //     cmd_msg->data[i] = constrain(-speed * 30, -70.0f, 70.0f);
            //     break;  
            // case 3:
            //     cmd_msg->data[i] = constrain(-speed * 12, -30.0f, 30.0f);
            //     break;
            // case 4:
            //     cmd_msg->data[i] = constrain(-speed, -70.0f, 70.0f);
            //     break;
            // }

            joint[i].cmd_pwm = joint[i].calc_pwm(period.seconds() * 1000);
            cmd_msg->data[i] = joint[i].cmd_pwm;

            // RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Joint %s pid: %f, %f, %f", joint[i].name.c_str(), joint[i].i_error, period.seconds() * 1000, joint[i].cmd_pwm);
            // RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Joint %s command: %f, %f, %f", joint[i].name.c_str(), joint[i].pos, joint[i].cmd, joint[i].cmd_pwm);
            // RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Joint %s enc data: %f", joint[i].name.c_str(), joint[i].enc);
            
        }

        joint_cmds_pub_->publish(std::move(cmd_msg));

        return hardware_interface::return_type::OK;
    }
    
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm::ArmHardware, hardware_interface::SystemInterface)
#include <asio.hpp>
#include <memory>
#include <string>
#include <cstring>
#include <thread>
#include <iostream>
#include <sstream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class SerialJointStateSubscriber : public rclcpp::Node
{
public:
    SerialJointStateSubscriber() : Node("serial_jointstate_subscriber"), io(),
                                   serial(io, "/dev/ttyACM0")
    {
        serial.set_option(asio::serial_port_base::baud_rate(115200));

        arduino_ready = false;

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&SerialJointStateSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState & msg) 
    {
        while (!arduino_ready)
        {
            old_a0 = msg.position[0];
            old_a1 = msg.position[1];
            old_a2 = msg.position[2];
            old_a3 = msg.position[3];
            old_a4 = msg.position[4];
            old_a5 = msg.position[5];

            asio::streambuf buffer;
            RCLCPP_INFO(this->get_logger(), "waiting for arduino...");
            asio::read_until(serial, buffer, "READY");

            std::istream is(&buffer);
            std::string received_data;
            getline(is, received_data);
            RCLCPP_INFO(this->get_logger(), "arduino status: %s", received_data.c_str());

            arduino_ready = true;
        }

        if (old_a0 != msg.position[0] || old_a1 != msg.position[1] ||
            old_a2 != msg.position[2] || old_a3 != msg.position[3] ||
            old_a4 != msg.position[4] || old_a5 != msg.position[5])
        {
            std::vector<float> positions;
            positions.push_back(rad_to_deg(msg.position[0]));
            positions.push_back(rad_to_deg(msg.position[1]));
            positions.push_back(rad_to_deg(msg.position[2]));
            positions.push_back(rad_to_deg(msg.position[3]));
            positions.push_back(rad_to_deg(msg.position[4]));
            positions.push_back(rad_to_deg(msg.position[5]));

            sendFloats(serial, positions);

            std::string arduino_response = readArduinoResponse(serial);
            RCLCPP_INFO(this->get_logger(), "rx: %s", arduino_response.c_str());

            old_a0 = msg.position[0];
            old_a1 = msg.position[1];
            old_a2 = msg.position[2];
            old_a3 = msg.position[3];
            old_a4 = msg.position[4];
            old_a5 = msg.position[5];
        }
    }

    void sendFloats(asio::serial_port& serial, const std::vector<float>& values)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);

        for (long unsigned int i = 0; i < values.size(); i++)
        {
            ss << values[i];
            if (i < values.size() - 1)
            {
                ss << ",";
            }
        }
        ss << "\n";

        std::string data = ss.str();
        asio::write(serial, asio::buffer(data));
        RCLCPP_INFO(this->get_logger(), "tx: %s", data.c_str());
    }

    std::string readArduinoResponse(asio::serial_port& serial)
    {
        asio::streambuf arduino_read;
        asio::read_until(serial, arduino_read, '\n');
        std::istream is(&arduino_read);

        std::string response;
        std::getline(is, response);
        return response;
    }

    float rad_to_deg(float angle_rad)
    {
        return angle_rad * (180.0/M_PI);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    asio::io_context io;
    asio::serial_port serial;
    bool arduino_ready;

    float old_a0;
    float old_a1;
    float old_a2;
    float old_a3;
    float old_a4;
    float old_a5;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialJointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}
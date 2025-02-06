#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <sstream>
#include <string>

class SerialReader : public rclcpp::Node {
public:
    SerialReader() : Node("raw_tof_node") {
        serial_port.setPort("/dev/ttyUSB0");  // Set the serial port
        serial_port.setBaudrate(2000000);  // Set the baud rate
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // Set a timeout
        serial_port.setTimeout(to);  // Apply the timeout setting
        serial_port.open();  // Open the serial port
        publisher_ = this->create_publisher<std_msgs::msg::String>("raw_tof", 10);  // Create a publisher

        // Create a timer to periodically call readSerialData()
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialReader::readSerialData, this)
        );
    }

    ~SerialReader() {
        if (serial_port.isOpen()) {
            serial_port.close();
        }
    }

private:
    void readSerialData() {
        size_t available = serial_port.available();
        if (available > 0) {  // Check if data is available
            std::string data;
            serial_port.read(data, available);
            buffer += data;  // Save data to buffer
            size_t pos = 0;
            while ((pos = buffer.find('\n')) != std::string::npos) {  // Find newline characters
                std::string line = buffer.substr(0, pos);  // Get one line of data
                auto msg = std_msgs::msg::String();
                msg.data = line;  // Set the message data
                publisher_->publish(msg);  // Publish the tof message
                buffer.erase(0, pos + 1);  // Erase processed data
            }
        }
    }

    serial::Serial serial_port;  // Serial port object
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // ROS2 publisher
    std::string buffer;  // Buffer for reading data
    rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic callback
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto node = std::make_shared<SerialReader>();  // Create a SerialReader object
    rclcpp::spin(node);  // Spin the node
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;
}


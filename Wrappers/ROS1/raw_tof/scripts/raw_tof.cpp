#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <sstream>
#include <string>

class SerialReader {  // Define a SerialReader class
public:
    SerialReader() {
        ros::NodeHandle nh;  // Create a node handle
        serial_port.setPort("/dev/ttyUSB1");  // Set the serial port
        serial_port.setBaudrate(2000000);  // Set the baud rate
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // Set a timeout
        serial_port.setTimeout(to);  // Apply the timeout setting
        serial_port.open();  // Open the serial port
        publisher_ = nh.advertise<std_msgs::String>("raw_tof", 10);  // Create a publisher
    }

    void readSerialData() {
        //ros::Rate loop_rate(100); // Set the loop rate in Hz
        while (ros::ok()) {
            size_t available = serial_port.available();
            if (available > 0) {  // Check if data is available
                std::string data;
                serial_port.read(data, available);
                buffer += data;  // Save data to buffer
                size_t pos = 0;
                while ((pos = buffer.find('\n')) != std::string::npos) {  // Find newline characters
                    std::string line = buffer.substr(0, pos);  // Get one line of data
                    std_msgs::String msg;
                    msg.data = line;  // Set the message data
                    publisher_.publish(msg);  // Publish the tof message
                    buffer.erase(0, pos + 1);  // Erase processed data
                }
            }
            ros::spinOnce();  // Handle ROS events
            //loop_rate.sleep();  // Sleep
        }
    }

    ~SerialReader() {
        if (serial_port.isOpen()) {
            serial_port.close();
        }
    }

private:
    serial::Serial serial_port;  // Serial port object
    ros::Publisher publisher_;  // ROS publisher
    std::string buffer;  // Buffer for reading data
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "raw_tof_node");  // Initialize ROS node
    SerialReader reader;  // Create a SerialReader object
    try {
        reader.readSerialData();  // Read serial data
    } catch (const std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());  // Handle exception
    }
    return 0;
}

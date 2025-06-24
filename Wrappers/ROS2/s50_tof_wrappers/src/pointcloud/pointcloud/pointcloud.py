#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import re
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

Number_sensors = 5  # Define the number of connected Tof sensors
First_CANID = 16    # Define the CAN ID of first Tof sensor
Last_CANID = 20     # Define the CAN ID of last Tof sensor

class SerialToPointCloud2(Node):
    def __init__(self):
        super().__init__('TOF_to_pointcloud2')  # Initialize the ROS2 node
        self.subscriber_ = self.create_subscription(
            String, 'raw_tof', self.process_data_callback, 10)  # Subscribe to raw tof data
        self.publisher_ = self.create_publisher(PointCloud2, 'tof_sensor', 10)  # Set up a publisher for the PointCloud2 data
        self.points = []  # Initialize a list to store point cloud data

    def process_data_callback(self, data):
        self.process_data(data.data)  # Process the received data
        if len(self.points) >= Number_sensors * 32:  # Check if the list of points is full
            self.publish_points()  # Publish the point cloud
            self.points = []  # Clear the list

    def process_data(self, data):
        parts = data.split(':')  # Split the data by colon
        if len(parts) < 2:  # Check if the data is complete
            self.get_logger().error(f"Received malformed data: {data}")  # Log an error message
            return

        device_id_str = re.sub(r'\D', '', parts[0])  # Extract the device ID
        if not device_id_str.isdigit():  # Check if the device ID is valid
            self.get_logger().error(f"Invalid device ID: {parts[0]}")
            return
        device_id = int(device_id_str)  # Convert device ID to integer

        data_parts = parts[1].split(' ')  # Split the coordinates and distance data
        if len(data_parts) < 2:
            self.get_logger().error(f"Malformed coordinates and distance: {parts[1]}")
            return

        coords = re.sub(r'\D', '', data_parts[0])  # Extract the coordinates
        if len(coords) < 2:
            self.get_logger().error(f"Coordinates are too short: {coords}")
            return
        distance_str = re.sub(r'\D', '', data_parts[1])  # Extract the distance

        if not distance_str.isdigit():  # Check if the distance is valid
            self.get_logger().error(f"Invalid distance: {distance_str}")
            return
        try:
            y, z = 0.01*(int(coords[1])) + 0.08*(device_id-((Last_CANID + First_CANID)/2)-0.5), 0.02*(5-(int(coords[0]))) + 0.08  # Convert to point cloud coordinates
            x = 0.001 * (int(distance_str))
        except IndexError as e:
            self.get_logger().error(f"Index error while processing data: {e}")  # Log an index error
            return
        except ValueError as e:
            self.get_logger().error(f"Value error while processing data: {e}")  # Log a value error
            return

        self.points.append([x, y, z])  # Add the point to the list

    def publish_points(self):
        header = Header()  # Create a header
        header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
        header.frame_id = "tof_sensor"  # Set the message frame ID
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]  # Define the fields for the point cloud
        cloud = pc2.create_cloud(header, fields, self.points)  # Create the point cloud message
        self.publisher_.publish(cloud)  # Publish to ROS2

def main(args=None):
    rclpy.init(args=args)
    node = SerialToPointCloud2()
    try:
        rclpy.spin(node)  # Keep the program running until the node is shut down
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python2.7
#Subscribe raw Tof data and package it into pointcloud2 format and publish to ROS
import rospy
import re
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String

Number_sensors = 5 #Define the number of connected Tof sensors
First_CANID = 16 #Define the CAN ID of first Tof sensor
Last_CANID = 20 #Define the CAN ID of last Tof sensor

class SerialToPointCloud2:
    def __init__(self):
        rospy.init_node('TOF_to_pointcloud2', anonymous=True) #Initialize ROS node
        self.subscriber_ = rospy.Subscriber('raw_tof', String, self.process_data_callback) # Subscribe to raw tof data
        self.publisher_ = rospy.Publisher('tof_sensor', PointCloud2, queue_size=1) # Set up a publisher for the PointCloud2 data
        self.points = [] # Initialize a list to store point cloud data

    def process_data_callback(self, data):
        self.process_data(data.data) # Process the received data
        if len(self.points) >= Number_sensors*32: # Check if the list of points is full
            self.publish_points() # Publish the point cloud
            self.points = [] # Clear the list

    def process_data(self, data):
        parts = data.split(':')  # Split the data by colon
        if len(parts) < 2: # Check if the data is complete
            #rospy.logerr("Received malformed data: {}".format(data))  # Log an error message
            return

        device_id_str = re.sub(r'\D', '', parts[0]) # Extract the device ID
        if not device_id_str.isdigit(): # Check if the device ID is valid
            #rospy.logerr("Invalid device ID: {}".format(parts[0]))
            return
        device_id = int(device_id_str) # Convert device ID to integer

        data_parts = parts[1].split(' ') # Split the coordinates and distance data
        if len(data_parts) < 2:
            #rospy.logerr("Malformed coordinates and distance: {}".format(parts[1]))
            return

        coords = re.sub(r'\D', '', data_parts[0])  # Extract the coordinates
        if len(coords) < 2:
            #rospy.logerr("Coordinates are too short: {}".format(coords))
            return
        distance_str = re.sub(r'\D', '', data_parts[1]) # Extract the distance

        if not distance_str.isdigit(): # Check if the distance is valid
            #rospy.logerr("Invalid distance: {}".format(distance_str))
            return
        try:
            y, z = 0.01*(int(coords[1]))+0.08*(device_id-((Last_CANID + First_CANID)/2)-0.5), 0.02*(5-(int(coords[0])))+0.08 #Convert to point cloud coordinates
            x = 0.001*(int(distance_str))

        except IndexError as e:
            #rospy.logerr("Index error while procrssing data: {}".format(e)) # Log an index error
            return
        except ValueError as e:
            #rospy.logerr("Value error while procrssing data: {}".format(e)) # Log a value error
            return

        self.points.append([x, y, z]) # Add the point to the list

    def publish_points(self):
        header = Header() # Create a header
        header.stamp = rospy.Time.now() # Set the timestamp
        header.frame_id = "tof_sensor" # Set the message frame ID
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), 
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ] # Define the fields for the point cloud
        cloud = pc2.create_cloud(header, fields, self.points) # Create the point cloud message
        self.publisher_.publish(cloud) # Publish to ROS

if __name__ == '__main__':
    node = SerialToPointCloud2()
    try:
        rospy.spin() # Keep the program running until the node is shut down
    except rospy.ROSInterruptException:
        pass

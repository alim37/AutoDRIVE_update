#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Quality of Service (tune communication between nodes)
import tf2_ros # ROS bindings for tf2 library to handle transforms
from std_msgs.msg import Int32, Float32, Header # Int32, Float32 and Header message classes
from geometry_msgs.msg import Point, TransformStamped # Point and TransformStamped message classes
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes
from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation
from ament_index_python.packages import get_package_share_directory # Access package's shared directory path

# Python module imports
from cv_bridge import CvBridge, CvBridgeError # ROS bridge for opencv library to handle images
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from geventwebsocket.handler import WebSocketHandler # Handler for WebSocket messages and lifecycle events
import socketio # Socket.IO realtime client and server
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
from io import BytesIO # Manipulate bytes data in memory
from PIL import Image # Python Imaging Library's (PIL's) Image module
import configparser # Parsing shared configuration file(s)
import autodrive_f1tenth.config_2 as config # AutoDRIVE Ecosystem ROS 2 configuration for F1TENTH vehicle

import gzip

def decompress_lidar_data(compressed_str: str) -> np.ndarray:
    """
    Decompress a LIDAR string compressed with Unity's DataCompressor.CompressArray().
    It's a GZip-compressed base64-encoded float string.
    Returns a NumPy float array.
    """
    try:
        # Decode from base64
        decoded = base64.b64decode(compressed_str)

        # Decompress from GZip
        decompressed = gzip.decompress(decoded).decode('ascii')

        # Replace commas with spaces (if needed)
        decompressed = decompressed.replace(",", " ")

        # Convert to NumPy array
        return np.fromstring(decompressed.strip(), dtype=float, sep=' ')
    except Exception as e:
        print(f"❌ Failed to decompress LIDAR data: {e}")
        return np.array([], dtype=float)

################################################################################

# Global declarations
global autodrive_incoming_bridge, cv_bridge, publishers
global throttle_command, steering_command
global throttle_command2, steering_command2 # VEHICLE 2

global tf_broadcaster # Declare tf_broadcaster as global

# Removed latest_position/orientation globals as they are no longer needed
# if transforms are broadcast immediately from the callback.
# If you still want to log them or use them for other purposes, you can keep them.
# latest_position_1 = None
# latest_orientation_1 = None
# latest_position_2 = None
# latest_orientation_2 = None

# Initialize vehicle control commands
throttle_command = config.throttle_command
steering_command = config.steering_command
throttle_command2 = config.throttle_command2
steering_command2 = config.steering_command2

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_int_msg(val):
    i = Int32()
    i.data = int(val)
    return i

def create_float_msg(val):
    f = Float32()
    f.data = float(val)
    return f

def create_joint_state_msg(joint_angle, joint_name, frame_id):
    js = JointState()
    js.header = Header()
    js.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    js.header.frame_id = frame_id
    js.name = [joint_name]
    js.position = [joint_angle]
    js.velocity = []
    js.effort = []
    return js

def create_point_msg(position):
    p = Point()
    p.x = position[0]
    p.y = position[1]
    p.z = position[2]
    return p

def create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration, frame_id: str):
    imu = Imu()
    imu.header = Header()
    imu.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    imu.header.frame_id = frame_id
    imu.orientation.x = orientation_quaternion[0]
    imu.orientation.y = orientation_quaternion[1]
    imu.orientation.z = orientation_quaternion[2]
    imu.orientation.w = orientation_quaternion[3]
    imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.angular_velocity.x = angular_velocity[0]
    imu.angular_velocity.y = angular_velocity[1]
    imu.angular_velocity.z = angular_velocity[2]
    imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.linear_acceleration.x = linear_acceleration[0]
    imu.linear_acceleration.y = linear_acceleration[1]
    imu.linear_acceleration.z = linear_acceleration[2]
    imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    return imu

def create_laser_scan_msg(lidar_scan_rate, lidar_range_array, lidar_intensity_array, frame_id: str):
    ls = LaserScan()
    ls.header = Header()
    ls.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    ls.header.frame_id = frame_id
    
    ls.angle_min = -2.35619 # Minimum angle of laser scan (-135 degrees)
    ls.angle_max = 2.35619 # Maximum angle of laser scan (135 degrees)
    ls.angle_increment = 0.004363323 # Angular resolution of laser scan (0.25 degree)
    ls.time_increment = (1 / lidar_scan_rate) / 1080 # Time required to scan 1 point
    ls.scan_time = ls.time_increment * 1080 # Time required to complete a scan
    ls.range_min = 0.06 # Minimum sensor range (in meters)
    ls.range_max = 10.0 # Maximum sensor range (in meters)
    
    ls.ranges = lidar_range_array
    ls.intensities = lidar_intensity_array
    return ls

def create_image_msg(image_array, frame_id):
    img = cv_bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
    img.header = Header()
    img.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    img.header.frame_id = frame_id
    return img

# This is the function we'll use for broadcasting transforms
def broadcast_tf(tf_broadcaster_instance, parent_frame, child_frame, position, orientation, clock):
    t = TransformStamped()
    t.header.stamp = clock.now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]

    t.transform.rotation.x = orientation[0]
    t.transform.rotation.y = orientation[1]
    t.transform.rotation.z = orientation[2]
    t.transform.rotation.w = orientation[3]

    tf_broadcaster_instance.sendTransform(t)


#########################################################
# ROS 2 PUBLISHER FUNCTIONS
#########################################################

# VEHICLE 1 DATA PUBLISHER FUNCTIONS

def publish_actuator_feedbacks(throttle, steering):
    publishers['pub_throttle'].publish(create_float_msg(throttle))
    publishers['pub_steering'].publish(create_float_msg(steering))

def publish_encoder_data(encoder_angles):
    # Changed frame_id to be specific to V1
    publishers['pub_left_encoder'].publish(create_joint_state_msg(encoder_angles[0], "left_encoder_1", "left_encoder_1"))
    publishers['pub_right_encoder'].publish(create_joint_state_msg(encoder_angles[1], "right_encoder_1", "right_encoder_1"))

def publish_ips_data(position):
    publishers['pub_ips'].publish(create_point_msg(position))

def publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration):
    publishers['pub_imu'].publish(create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration, frame_id='imu_1'))

def publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    try:
        publishers['pub_lidar'].publish(create_laser_scan_msg(lidar_scan_rate, lidar_range_array.tolist(), lidar_intensity_array.tolist(), 
                                                              frame_id = 'lidar_1'))
    except Exception as e:
        print(f"Error publishing V1 LIDAR: {e}")

def publish_camera_images(front_camera_image):
    try:
        publishers['pub_front_camera'].publish(create_image_msg(front_camera_image, "front_camera_1"))
    except Exception as e:
        print(f"Error publishing V1 camera: {e}")

# VEHICLE 2 DATA PUBLISHER FUNCTIONS

def publish_actuator_feedbacks_v2(throttle, steering):
    publishers['pub_throttle_2'].publish(create_float_msg(throttle))
    publishers['pub_steering_2'].publish(create_float_msg(steering))

def publish_encoder_data_v2(encoder_angles):
    # Changed frame_id to be specific to V2
    publishers['pub_left_encoder_2'].publish(create_joint_state_msg(encoder_angles[0], "left_encoder_2", "left_encoder_2"))
    publishers['pub_right_encoder_2'].publish(create_joint_state_msg(encoder_angles[1], "right_encoder_2", "right_encoder_2"))

def publish_ips_data_v2(position):
    publishers['pub_ips_2'].publish(create_point_msg(position))

def publish_imu_data_v2(orientation_q, angular_velocity, linear_acceleration):
    publishers['pub_imu_2'].publish(create_imu_msg(orientation_q, angular_velocity, linear_acceleration, frame_id='imu_2'))

def publish_lidar_scan_v2(rate, ranges, intensities):
    # print("🟢 Attempting to publish V2 LIDAR...") # Suppressed for cleaner output
    try:
        publishers['pub_lidar_2'].publish(create_laser_scan_msg(rate, ranges.tolist(), intensities.tolist(), frame_id='lidar_2'))
    except Exception as e:
        print(f"Error publishing V2 LIDAR: {e}")

def publish_camera_images_v2(image_array):
    try:
        publishers['pub_front_camera_2'].publish(create_image_msg(image_array, "front_camera_2"))
    except Exception as e:
        print(f"Error publishing V2 camera: {e}")

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print("Connected!")

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    print("🔵 Bridge event received") # Suppressed for cleaner output
    #print("🔍 Incoming data keys:", list(data.keys())) # Suppressed for cleaner output

    # Global declarations
    global autodrive_incoming_bridge, cv_bridge, publishers
    global throttle_command, steering_command
    global throttle_command2, steering_command2

    # Make tf_broadcaster accessible
    global tf_broadcaster 

    # Get package's shared directory path
    package_share_directory = get_package_share_directory('autodrive_f1tenth')

    # Wait for data to become available
    if data:
        # Try to read data from shared config file
        api_config = configparser.ConfigParser()
        try:
            api_config.read(package_share_directory+'/api_config.ini')
            # Update vehicle control commands
            throttle_command = float(api_config['f1tenth_1']['throttle_command'])
            steering_command = float(api_config['f1tenth_1']['steering_command'])
            throttle_command2 = float(api_config['f1tenth_2']['throttle_command'])
            steering_command2 = float(api_config['f1tenth_2']['steering_command'])
        # Pass if file cannot be read
        except:
            pass

        ########################################################################
        # VEHICLE 1 DATA
        ########################################################################
        # Actuator feedbacks
        throttle = float(data["V1 Throttle"])
        steering = float(data["V1 Steering"])
        publish_actuator_feedbacks(throttle, steering)
        
        # Wheel encoders
        encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        publish_encoder_data(encoder_angles)
        
        # IPS
        position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        publish_ips_data(position)
        
        # IMU
        orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration)
        
        # Coordinate transforms for Vehicle 1
        # Broadcast transforms here, as soon as the data arrives
        broadcast_tf(tf_broadcaster, "map", "f1tenth_1", position, orientation_quaternion, autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "left_encoder_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[0]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "right_encoder_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[1]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "ips_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "imu_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), autodrive_incoming_bridge.get_clock())
        
        broadcast_tf(tf_broadcaster, "f1tenth_1", "front_camera_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "front_left_wheel_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537-2*0.0765*np.tan(steering)))), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "front_right_wheel_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537+2*0.0765*np.tan(steering)))), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "rear_left_wheel_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[0]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_1", "rear_right_wheel_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[1]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        
        # LIDAR for Vehicle 1
        try:
            lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])

            # 1️⃣  DECOMPRESS the range array coming from Unity
            lidar_range_array = decompress_lidar_data(data["V1 LIDAR Range Array"])

            # 2️⃣  DECOMPRESS the intensity array (or fall back to ones)
            if "V1 LIDAR Intensity Array" in data and data["V1 LIDAR Intensity Array"].strip():
                lidar_intensity_array = decompress_lidar_data(data["V1 LIDAR Intensity Array"])
            else:
                lidar_intensity_array = np.ones_like(lidar_range_array)

            # (length-mismatch safeguard stays the same)
            if len(lidar_intensity_array) != len(lidar_range_array):
                lidar_intensity_array = np.ones_like(lidar_range_array)

            # print(f"V1 LIDAR: Rate={lidar_scan_rate}, " # Suppressed for cleaner output
            #     f"Points={len(lidar_range_array)}, Intensities={len(lidar_intensity_array)}")

            broadcast_tf(tf_broadcaster, "f1tenth_1", "lidar_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]), autodrive_incoming_bridge.get_clock())
            publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array)

        except Exception as e:
            print(f"Error processing V1 LIDAR data: {e}")
        
        # Cameras for Vehicle 1
        front_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
        publish_camera_images(front_camera_image)   

        ########################################################################
        # VEHICLE 2 DATA
        ########################################################################
        # Actuator feedbacks
        throttle2 = float(data["V2 Throttle"])
        steering2 = float(data["V2 Steering"])
        publish_actuator_feedbacks_v2(throttle2, steering2)
        
        # Wheel encoders
        encoder_angles2 = np.fromstring(data["V2 Encoder Angles"], dtype=float, sep=' ')
        publish_encoder_data_v2(encoder_angles2)
        
        # IPS
        position2 = np.fromstring(data["V2 Position"], dtype=float, sep=' ')
        publish_ips_data_v2(position2)

        # IMU
        orientation_quaternion2 = np.fromstring(data["V2 Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity2 = np.fromstring(data["V2 Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration2 = np.fromstring(data["V2 Linear Acceleration"], dtype=float, sep=' ')
        publish_imu_data_v2(orientation_quaternion2, angular_velocity2, linear_acceleration2)

        # Coordinate transforms for Vehicle 2
        # Broadcast transforms here, as soon as the data arrives
        broadcast_tf(tf_broadcaster, "map", "f1tenth_2", position2, orientation_quaternion2, autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "left_encoder_2", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles2[0]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "right_encoder_2", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles2[1]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "ips_2", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "imu_2", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), autodrive_incoming_bridge.get_clock())
        
        broadcast_tf(tf_broadcaster, "f1tenth_2", "front_camera_2", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "front_left_wheel_2", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering2))/(2*0.141537-2*0.0765*np.tan(steering2)))), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "front_right_wheel_2", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering2))/(2*0.141537+2*0.0765*np.tan(steering2)))), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "rear_left_wheel_2", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles2[0]%6.283, 0.0), autodrive_incoming_bridge.get_clock())
        broadcast_tf(tf_broadcaster, "f1tenth_2", "rear_right_wheel_2", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles2[1]%6.283, 0.0), autodrive_incoming_bridge.get_clock())

        # LIDAR for Vehicle 2
        try:
            lidar_scan_rate2 = float(data["V2 LIDAR Scan Rate"])

            lidar_range_array2 = decompress_lidar_data(data["V2 LIDAR Range Array"])

            if "V2 LIDAR Intensity Array" in data and data["V2 LIDAR Intensity Array"].strip():
                lidar_intensity_array2 = decompress_lidar_data(data["V2 LIDAR Intensity Array"])
            else:
                lidar_intensity_array2 = np.ones_like(lidar_range_array2)

            if len(lidar_intensity_array2) != len(lidar_range_array2):
                lidar_intensity_array2 = np.ones_like(lidar_range_array2)

            # print(f"V2 LIDAR: Rate={lidar_scan_rate2}, " # Suppressed for cleaner output
            #     f"Points={len(lidar_range_array2)}, Intensities={len(lidar_intensity_array2)}")

            broadcast_tf(tf_broadcaster, "f1tenth_2", "lidar_2", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]), autodrive_incoming_bridge.get_clock())
            publish_lidar_scan_v2(lidar_scan_rate2, lidar_range_array2, lidar_intensity_array2)

        except Exception as e:
            print(f"Error processing V2 LIDAR data: {e}")


        # Cameras for Vehicle 2
        front_camera_image2 = np.asarray(Image.open(BytesIO(base64.b64decode(data["V2 Front Camera Image"]))))
        publish_camera_images_v2(front_camera_image2)

        ########################################################################
        # CONTROL COMMANDS
        ########################################################################
        # Vehicle control commands
        # print(f"EMIT: V1 → T:{throttle_command}, S:{steering_command} | V2 → T:{throttle_command2}, S:{steering_command2}") # Suppressed for cleaner output

        #changed from 'Bridge'
        sio.emit('VehicleControl', data={
            'V1 Throttle': str(throttle_command if throttle_command is not None else 0.0),
            'V1 Steering': str(steering_command if steering_command is not None else 0.0),
            'V2 Throttle': str(throttle_command2 if throttle_command2 is not None else 0.0),
            'V2 Steering': str(steering_command2 if steering_command2 is not None else 0.0)
        })

#########################################################
# AUTODRIVE ROS 2 INCOMING BRIDGE INFRASTRUCTURE
#########################################################

def main():
    # Global declarations
    global autodrive_incoming_bridge, cv_bridge, publishers
    global throttle_command, steering_command
    global tf_broadcaster # Ensure tf_broadcaster is set globally here

    # ROS 2 infrastructure
    rclpy.init() # Initialize ROS 2 communication for this context
    autodrive_incoming_bridge = rclpy.create_node('autodrive_incoming_bridge') # Create ROS 2 node

    tf_broadcaster = tf2_ros.TransformBroadcaster(autodrive_incoming_bridge)


    qos_profile = QoSProfile( # Quality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the "history" policy was set to "keep last")
        )
    cv_bridge = CvBridge() # ROS bridge object for opencv library to handle image data
    publishers = {e.name: autodrive_incoming_bridge.create_publisher(e.type, e.topic, qos_profile)
                  for e in config.pub_sub_dict.publishers} # Publishers

    # Create socketio WSGI application
    app = socketio.WSGIApp(sio)
    
    # Deploy as a gevent WSGI server in a separate thread/process
    import threading
    server_thread = threading.Thread(
        target=lambda: pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()
    )
    server_thread.daemon = True
    server_thread.start()
    
    # Recursive operations while node is alive
    while rclpy.ok():
        # Spin the node once with timeout to process any pending ROS 2 callbacks
        # (e.g., if you had subscribers in this node for control commands from other ROS nodes)
        rclpy.spin_once(autodrive_incoming_bridge, timeout_sec=0.01) 

        # Removed the re-broadcast TF logic from here.
        # Transforms are now broadcast directly in the `bridge` callback
        # when new data arrives from the simulator.
        # This makes the TF updates synchronous with the incoming simulation data.
    
    autodrive_incoming_bridge.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # Shutdown this context

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE Ecosystem ROS 2 incoming bridge

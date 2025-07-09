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
from rclpy.node import Node # Import Node class
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # Updated: Quality of Service (tune communication between nodes)
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
# import configparser # No longer needed for control commands as per analysis
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

# Global declarations for socketio server (needs to be outside the class for the @sio.on decorator)
global sio
sio = socketio.Server(async_mode='gevent')

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
    # The timestamp will be set by the node's clock in the _handle_bridge_event method
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
    # The timestamp will be set by the node's clock in the _handle_bridge_event method
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
    # The timestamp will be set by the node's clock in the _handle_bridge_event method
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
    # The CvBridge object will be passed from the node instance
    img = CvBridge().cv2_to_imgmsg(image_array, encoding="rgb8") 
    img.header = Header()
    # The timestamp will be set by the node's clock in the _handle_bridge_event method
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
# AUTODRIVE ROS 2 INCOMING BRIDGE INFRASTRUCTURE (as a Node class)
#########################################################

# Define the 'Bridge' event handler globally, but have it call a method of the Node instance
@sio.on('Bridge')
def bridge(sid, data):
    # This global will be set in main() to the instance of AutodriveIncomingBridge
    global current_incoming_bridge_node
    if current_incoming_bridge_node:
        current_incoming_bridge_node._handle_bridge_event(sid, data)
    else:
        print("Error: AutodriveIncomingBridge node not yet initialized.")


class AutodriveIncomingBridge(Node):
    def __init__(self):
        super().__init__('autodrive_incoming_bridge')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.cv_bridge = CvBridge()

        # Quality of Service profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Updated: Use non-deprecated enum value
            history=HistoryPolicy.KEEP_LAST,        # Updated: Use non-deprecated enum value
            depth=1
        )

        # Publishers for sensor data (using config.pub_sub_dict)
        # RENAMED from self.publishers to self.sensor_publishers to avoid AttributeError
        self.sensor_publishers = {e.name: self.create_publisher(e.type, e.topic, qos_profile)
                                  for e in config.pub_sub_dict.publishers}

        # Initialize control command storage with default values from config
        self.throttle_command_v1 = config.throttle_command
        self.steering_command_v1 = config.steering_command
        self.throttle_command_v2 = config.throttle_command2
        self.steering_command_v2 = config.steering_command2

        # Subscribers for control commands from other ROS nodes (e.g., Pure Pursuit)
        # These are the topics your Pure Pursuit node PUBLISHES to
        self.create_subscription(
            Float32,
            '/autodrive/f1tenth_1/throttle_command',
            self.throttle_command_cb_v1,
            qos_profile
        )
        self.create_subscription(
            Float32,
            '/autodrive/f1tenth_1/steering_command',
            self.steering_command_cb_v1,
            qos_profile
        )
        self.create_subscription(
            Float32,
            '/autodrive/f1tenth_2/throttle_command',
            self.throttle_command_cb_v2,
            qos_profile
        )
        self.create_subscription(
            Float32,
            '/autodrive/f1tenth_2/steering_command',
            self.steering_command_cb_v2,
            qos_profile
        )

        self.get_logger().info("AutodriveIncomingBridge node initialized.")

    # Subscriber Callbacks for incoming ROS2 commands (from Pure Pursuit Node)
    def throttle_command_cb_v1(self, msg):
        self.throttle_command_v1 = float(np.round(msg.data, 3))
        # self.get_logger().info(f"Received V1 Throttle: {self.throttle_command_v1}") # For debugging

    def steering_command_cb_v1(self, msg):
        self.steering_command_v1 = float(np.round(msg.data, 3))
        # self.get_logger().info(f"Received V1 Steering: {self.steering_command_v1}") # For debugging

    def throttle_command_cb_v2(self, msg):
        self.throttle_command_v2 = float(np.round(msg.data, 3))

    def steering_command_cb_v2(self, msg):
        self.steering_command_v2 = float(np.round(msg.data, 3))

    # This method handles the 'Bridge' event received from the WebSocket
    def _handle_bridge_event(self, sid, data):
        # self.get_logger().info("🔵 Bridge event received") # Suppressed for cleaner output
        # self.get_logger().info(f"🔍 Incoming data keys: {list(data.keys())}") # Suppressed for cleaner output

        if data:
            # === VEHICLE 1 DATA ===
            # Actuator feedbacks (from simulator, reflecting what it received)
            throttle_feedback_v1 = float(data["V1 Throttle"])
            steering_feedback_v1 = float(data["V1 Steering"])
            self.sensor_publishers['pub_throttle'].publish(create_float_msg(throttle_feedback_v1))
            self.sensor_publishers['pub_steering'].publish(create_float_msg(steering_feedback_v1))
            
            # Wheel encoders
            encoder_angles_v1 = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
            js_msg_left1 = create_joint_state_msg(encoder_angles_v1[0], "left_encoder_1", "left_encoder_1")
            js_msg_left1.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_left_encoder'].publish(js_msg_left1)
            
            js_msg_right1 = create_joint_state_msg(encoder_angles_v1[1], "right_encoder_1", "right_encoder_1")
            js_msg_right1.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_right_encoder'].publish(js_msg_right1)
            
            # IPS
            position_v1 = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
            self.sensor_publishers['pub_ips'].publish(create_point_msg(position_v1))
            
            # IMU
            orientation_quaternion_v1 = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
            angular_velocity_v1 = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
            linear_acceleration_v1 = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
            imu_msg_v1 = create_imu_msg(orientation_quaternion_v1, angular_velocity_v1, linear_acceleration_v1, frame_id='imu_1')
            imu_msg_v1.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_imu'].publish(imu_msg_v1)
            
            # Coordinate transforms for Vehicle 1
            broadcast_tf(self.tf_broadcaster, "map", "f1tenth_1", position_v1, orientation_quaternion_v1, self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "left_encoder_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles_v1[0]%6.283, 0.0), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "right_encoder_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles_v1[1]%6.283, 0.0), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "ips_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "imu_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), self.get_clock())
            
            # Note: Steering angles here are from feedback, not commands. Adjust accordingly if you have separate command TFs
            # The angles for wheels usually need the actual steering input applied
            steer_angle_fl1 = np.arctan((2*0.141537*np.tan(steering_feedback_v1))/(2*0.141537-2*0.0765*np.tan(steering_feedback_v1)))
            steer_angle_fr1 = np.arctan((2*0.141537*np.tan(steering_feedback_v1))/(2*0.141537+2*0.0765*np.tan(steering_feedback_v1)))

            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "front_camera_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "front_left_wheel_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, steer_angle_fl1), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "front_right_wheel_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, steer_angle_fr1), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "rear_left_wheel_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles_v1[0]%6.283, 0.0), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_1", "rear_right_wheel_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles_v1[1]%6.283, 0.0), self.get_clock())
            
            # LIDAR for Vehicle 1
            try:
                lidar_scan_rate_v1 = float(data["V1 LIDAR Scan Rate"])

                lidar_range_array_v1 = decompress_lidar_data(data["V1 LIDAR Range Array"])
                if "V1 LIDAR Intensity Array" in data and data["V1 LIDAR Intensity Array"].strip():
                    lidar_intensity_array_v1 = decompress_lidar_data(data["V1 LIDAR Intensity Array"])
                else:
                    lidar_intensity_array_v1 = np.ones_like(lidar_range_array_v1)
                if len(lidar_intensity_array_v1) != len(lidar_range_array_v1):
                    lidar_intensity_array_v1 = np.ones_like(lidar_range_array_v1)

                broadcast_tf(self.tf_broadcaster, "f1tenth_1", "lidar_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]), self.get_clock())
                ls_msg_v1 = create_laser_scan_msg(lidar_scan_rate_v1, lidar_range_array_v1.tolist(), lidar_intensity_array_v1.tolist(), frame_id = 'lidar_1')
                ls_msg_v1.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
                self.sensor_publishers['pub_lidar'].publish(ls_msg_v1)

            except Exception as e:
                self.get_logger().error(f"Error processing V1 LIDAR data: {e}")
            
            # Cameras for Vehicle 1
            front_camera_image_v1 = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
            img_msg_v1 = create_image_msg(front_camera_image_v1, "front_camera_1")
            img_msg_v1.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_front_camera'].publish(img_msg_v1)   

            # === VEHICLE 2 DATA ===
            # Actuator feedbacks
            throttle_feedback_v2 = float(data["V2 Throttle"])
            steering_feedback_v2 = float(data["V2 Steering"])
            self.sensor_publishers['pub_throttle_2'].publish(create_float_msg(throttle_feedback_v2))
            self.sensor_publishers['pub_steering_2'].publish(create_float_msg(steering_feedback_v2))
            
            # Wheel encoders
            encoder_angles_v2 = np.fromstring(data["V2 Encoder Angles"], dtype=float, sep=' ')
            js_msg_left2 = create_joint_state_msg(encoder_angles_v2[0], "left_encoder_2", "left_encoder_2")
            js_msg_left2.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_left_encoder_2'].publish(js_msg_left2)

            js_msg_right2 = create_joint_state_msg(encoder_angles_v2[1], "right_encoder_2", "right_encoder_2")
            js_msg_right2.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_right_encoder_2'].publish(js_msg_right2)
            
            # IPS
            position_v2 = np.fromstring(data["V2 Position"], dtype=float, sep=' ')
            self.sensor_publishers['pub_ips_2'].publish(create_point_msg(position_v2))

            # IMU
            orientation_quaternion_v2 = np.fromstring(data["V2 Orientation Quaternion"], dtype=float, sep=' ')
            angular_velocity_v2 = np.fromstring(data["V2 Angular Velocity"], dtype=float, sep=' ')
            linear_acceleration_v2 = np.fromstring(data["V2 Linear Acceleration"], dtype=float, sep=' ')
            imu_msg_v2 = create_imu_msg(orientation_quaternion_v2, angular_velocity_v2, linear_acceleration_v2, frame_id='imu_2')
            imu_msg_v2.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_imu_2'].publish(imu_msg_v2)

            # Coordinate transforms for Vehicle 2
            broadcast_tf(self.tf_broadcaster, "map", "f1tenth_2", position_v2, orientation_quaternion_v2, self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "left_encoder_2", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles_v2[0]%6.283, 0.0), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "right_encoder_2", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles_v2[1]%6.283, 0.0), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "ips_2", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "imu_2", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]), self.get_clock())
            
            steer_angle_fl2 = np.arctan((2*0.141537*np.tan(steering_feedback_v2))/(2*0.141537-2*0.0765*np.tan(steering_feedback_v2)))
            steer_angle_fr2 = np.arctan((2*0.141537*np.tan(steering_feedback_v2))/(2*0.141537+2*0.0765*np.tan(steering_feedback_v2)))

            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "front_camera_2", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "front_left_wheel_2", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, steer_angle_fl2), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "front_right_wheel_2", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, steer_angle_fr2), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "rear_left_wheel_2", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles_v2[0]%6.283, 0.0), self.get_clock())
            broadcast_tf(self.tf_broadcaster, "f1tenth_2", "rear_right_wheel_2", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles_v2[1]%6.283, 0.0), self.get_clock())

            lidar_data_str2 = data.get("V2 LIDAR Range Array", "").strip()
            if lidar_data_str2:
                try:
                    lidar_scan_rate_v2 = float(data.get("V2 LIDAR Scan Rate", 0.0))
                    lidar_range_array_v2 = decompress_lidar_data(lidar_data_str2)
                    intensity_str2 = data.get("V2 LIDAR Intensity Array", "").strip()
                    if intensity_str2:
                        lidar_intensity_array_v2 = decompress_lidar_data(intensity_str2)
                    else:
                        lidar_intensity_array_v2 = np.ones_like(lidar_range_array_v2)
                    if len(lidar_intensity_array_v2) != len(lidar_range_array_v2):
                        lidar_intensity_array_v2 = np.ones_like(lidar_range_array_v2)

                    broadcast_tf(self.tf_broadcaster, "f1tenth_2", "lidar_2", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]), self.get_clock())
                    ls_msg_v2 = create_laser_scan_msg(lidar_scan_rate_v2, lidar_range_array_v2.tolist(), lidar_intensity_array_v2.tolist(), frame_id='lidar_2')
                    ls_msg_v2.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
                    self.sensor_publishers['pub_lidar_2'].publish(ls_msg_v2)

                except Exception as e:
                    self.get_logger().error(f"Error processing V2 LIDAR data: {e}")
            
            # Cameras for Vehicle 2
            front_camera_image_v2 = np.asarray(Image.open(BytesIO(base64.b64decode(data["V2 Front Camera Image"]))))
            img_msg_v2 = create_image_msg(front_camera_image_v2, "front_camera_2")
            img_msg_v2.header.stamp = self.get_clock().now().to_msg() # Timestamp using node's clock
            self.sensor_publishers['pub_front_camera_2'].publish(img_msg_v2)

            # === CONTROL COMMANDS EMISSION ===
            # These are the commands received from the ROS2 subscribers in this node
            sio.emit('VehicleControl', data={
                'V1 Throttle': str(self.throttle_command_v1),
                'V1 Steering': str(self.steering_command_v1),
                'V2 Throttle': str(self.throttle_command_v2),
                'V2 Steering': str(self.steering_command_v2)
            })

# Global variable to hold the node instance, so the `sio.on('Bridge')` decorator can access it.
current_incoming_bridge_node = None

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the AutodriveIncomingBridge node
    global current_incoming_bridge_node
    current_incoming_bridge_node = AutodriveIncomingBridge()

    # Create socketio WSGI application
    app = socketio.WSGIApp(sio)
    
    # Deploy as a gevent WSGI server in a separate thread/process
    import threading
    server_thread = threading.Thread(
        target=lambda: pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()
    )
    server_thread.daemon = True # Allow the thread to exit when the main program exits
    server_thread.start()
    
    # Spin the ROS 2 node in the main thread
    try:
        rclpy.spin(current_incoming_bridge_node)
    except KeyboardInterrupt:
        current_incoming_bridge_node.get_logger().info('Shutting down AutodriveIncomingBridge node due to KeyboardInterrupt')
    finally:
        current_incoming_bridge_node.destroy_node() # Explicitly destroy the node
        rclpy.shutdown() # Shutdown this context

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE Ecosystem ROS 2 incoming bridge

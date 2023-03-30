#!/usr/bin/env python3 

# Detect an ArUco marker and publish the ArUco marker's centroid offset.
# The marker's centroid offset is along the horizontal axis of the camera image.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary ROS 2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import qos_profile_sensor_data # Uses Best Effort reliability for camera
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Bool # Handles boolean messages
from std_msgs.msg import Int32 # Handles int 32 type message
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np  # Import Numpy library

import scipy

import os

# Import Python libraries
import cv2 # OpenCV library
import numpy as np # Import Numpy library

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class ArucoNode(Node):
  """
  Create an ArucoNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_node')

    # Declare parameters
    self.declare_parameter("aruco_dictionary_name", "DICT_4X4_50")
    # self.declare_parameter("aruco_marker_side_length", 0.085)  # in meters? 
    self.declare_parameter("aruco_marker_side_length", 0.05)  # in meters? 
    # self.declare_parameter("camera_calibration_parameters_filename", "swarm_crawler/scripts/calibration_chessboard.yaml")
    self.declare_parameter("camera_calibration_parameters_filename", "realsense_calibration.yaml")
    self.declare_parameter("image_topic", "/minibot_a_d435/color/image_raw")

    # realsense_calibration
    # self.declare_parameter("camera_calibration_parameters_filename", ".t265_calibration.yaml")
    
    self.declare_parameter("aruco_marker_name", "aruco_marker")
    
    # image_topic_string =  '/minibot_a_t265/fisheye2/image_raw'
   
    # Read parameters
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    # self.camera_calibration_parameters_filename = self.get_parameter(
    #   "camera_calibration_parameters_filename").get_parameter_value().string_value
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value

    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))
    cwd = os.getcwd()
    cv_file = cv2.FileStorage(cwd + '/scripts/realsense_calibration.yaml', cv2.FILE_STORAGE_READ)
        # cv_file = cv2.FileStorage(
        #   '~/', cv2.FILE_STORAGE_READ)
    self.mtx = cv_file.getNode('K').mat()
    self.dst = cv_file.getNode('D').mat()
    
    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(
  	  aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
    # self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    self.this_aruco_parameters = cv2.aruco.DetectorParameters()
    self.tfbroadcaster = TransformBroadcaster(self)

    
    # Create the subscriber. This subscriber will receive an Image
    # from the image_topic. 
    self.subscription = self.create_subscription(
      Image, 
      image_topic, 
      self.listener_callback, 
      qos_profile=qos_profile_sensor_data)
    self.subscription # prevent unused variable warning
    
    # Create the publishers
    # Publishes if an ArUco marker was detected
    self.publisher_aruco_marker_detected = self.create_publisher(Bool, 'aruco_marker_detected', 10)
    
    # Publishes x-centroid offset with respect to the camera image
    self.publisher_offset_aruco_marker = self.create_publisher(Int32, 'aruco_marker_offset', 10)
    self.offset_aruco_marker = 0
      
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
    
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
      # cameraMatrix=self.mtx, distCoeff=self.dst)
    
    # ArUco detected (True or False)
    aruco_detected_flag = Bool()
    aruco_detected_flag.data = False
    
    # ArUco center offset
    aruco_center_offset_msg = Int32()
    aruco_center_offset_msg.data = self.offset_aruco_marker
    
    image_width = current_frame.shape[1]

    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
    
      # ArUco marker has been detected
      aruco_detected_flag.data = True
    
      # Draw a square around detected markers in the video frame
      cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

      # Update the ArUco marker offset 
      M = cv2.moments(corners[0][0])
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])

      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                self.mtx,
                self.dst)
      
      self.offset_aruco_marker = cX - int(image_width/2)
      aruco_center_offset_msg.data = self.offset_aruco_marker

      cv2.putText(current_frame, "Center Offset: " + str(self.offset_aruco_marker), (cX - 40, cY - 40), 
        cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2) 
      
      cv2.drawFrameAxes(current_frame, self.mtx,
                                  self.dst, rvecs[0], tvecs[0], 0.05)
      
      for i, marker_id in enumerate(marker_ids):

                # Create the coordinate transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                # t.header.frame_id = 'base_link'
                t.header.frame_id = 'camera_link'
                # t.header.frame_id = 'aruco_marker'
                # t.header.frame_id = 'map'
                # t.header.frame_id = 'map'
                t.child_frame_id = 'aruco_marker'
                # Store the translation (i.e. position) information
                # t.transform.translation.x = tvecs[i][0][0]
                # t.transform.translation.y = tvecs[i][0][1]
                # t.transform.translation.z = tvecs[i][0][2]
                # fixed
                t.transform.translation.y = tvecs[i][0][0]
                t.transform.translation.z = tvecs[i][0][1]
                t.transform.translation.x = tvecs[i][0][2]

                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(
                    np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                # Quaternion format
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                # self.get_logger().info('broadcasting transform')
                self.tfbroadcaster.sendTransform(t)
                # self.get_logger().info('X,Y,Z,W Rotations: ;' + str(t.transform.rotation.x) + ';' + str(t.transform.rotation.y) + ';' + str(t.transform.rotation.z) + ';' + str(t.transform.rotation.w))

                # Send the transform
                # pose_array.header.frame_id = t.header.frame_id

                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                # pose_array.poses.append(pose)

    # Publish if ArUco marker has been detected or not
    self.publisher_aruco_marker_detected.publish(aruco_detected_flag)
    
    # Publish the center offset of the ArUco marker
    self.publisher_offset_aruco_marker.publish(aruco_center_offset_msg)
        
    # Display image for testing
    # cv2.imshow("camera", current_frame)
    # cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  aruco_node = ArucoNode()
  
  # Spin the node so the callback function is called.
  rclpy.spin(aruco_node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  aruco_node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

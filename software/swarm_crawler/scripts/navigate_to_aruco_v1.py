#! /usr/bin/env python3

"""
Description:
  Navigate to a charging dock once the battery gets low.
  We first navigate to a staging area in front of the docking station (~1.5 meters is good)
  We rotate around to search for the ArUco marker using the robot's front camera.
  Once the ArUco marker is detected, move towards it, making minor heading adjustments as necessary.
  Stop once the robot gets close enough to the charging dock or starts charging.
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
  
  A boolean variable that is True of ArUco marker detected, otherwise False
  /aruco_marker_detected – std_msgs/Bool
  
  The number of pixels offset of the ArUco marker from the center of the camera image
  /aruco_marker_offset - std_msgs/Int32
  
  LaserScan readings for object detection
  /scan - sensor_msgs/LaserScan
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------

"""

import math # Math library
import time  # Time library

from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data # Handle quality of service for LaserScan data
from nav2_simple_commander.robot_navigator import BasicNavigator# Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist # Velocity command
from sensor_msgs.msg import BatteryState # Battery status
from sensor_msgs.msg import LaserScan # Handle LIDAR scans
from std_msgs.msg import Bool # Handle boolean values
from std_msgs.msg import Int32 # Handle integer values
from std_msgs.msg import Float64 # Handle integer values
from std_msgs.msg import String
from bot_stats import HubbotStats, MinibotStats

# Holds the current pose of the aruco_marker
# base_link (parent frame) -> aruco_marker (child frame)
current_x = 0.0
current_y = 0.0
current_yaw_angle = 0.0

# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()

# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.25

# Flag to determine if the ArUco marker has been detected (True or False)
aruco_marker_detected = False

# Store the ArUco marker center offset (in pixels)
aruco_center_offset = 0

# Keep track of obstacles in front of the robot in meters
obstacle_distance_front = 999999.9

aruco_marker_distance = 999999.9


scan_topic = '/minibot_a/scan'
prev_readings = []


SEARCH = 4
FOUND = 5
UNDOCK = -1
WAIT_UNDOCK = -2
DONE = 6
NONE = 0
ARUCO_MARKER_MIN_DISTANCE = 0.11

class ConnectToChargingDockNavigator(Node):
    """
    Navigates to the aruco marker
    """      
    hubbot_current_stat: HubbotStats.STAT
    minibot_a_current_stat: MinibotStats.STAT

    def __init__(self): 

  
      # Initialize the class using the constructor
      super().__init__('connect_to_charging_dock_navigator')
    
      # Create a publisher
      # This node publishes the desired linear and angular velocity of the robot
      self.publisher_cmd_vel = self.create_publisher(
        Twist,
        '/minibot_a/cmd_vel',
        10)  
      timer_period = 0.1
      self.timer = self.create_timer(timer_period, self.state_machine)
      
      # Declare linear and angular velocities
      self.linear_velocity = 0.4  # meters per second
      # self.angular_velocity = 0.63 # radians per second
      # self.angular_velocity = 1.5 # radians per second
      self.angular_velocity = 1.2 # radians per second

      self.can_undock = False
      self.recently_docked = False
      
      # Keep track of which goal we're headed towards
      self.goal_idx = NONE
      self.can_dock = False
      
      # Declare obstacle tolerance 
      self.obstacle_tolerance = 0.22

      # Center offset tolerance in pixels
      # self.center_offset_tolerance = 10
      self.center_offset_tolerance = 50

      
      # Undocking distance
      self.undocking_distance = 1.25

      self.PUBLISH_PERIOD_S = 1.0
      self.hubbot_current_stat = HubbotStats.STAT.HubUnknown
      self.minibot_a_current_stat = MinibotStats.STAT.MiniNormalOperating

      self.hubbot_stat_sub = self.create_subscription(
            Int32,
            'hub_stat',
            self.hubbot_listener_callback,
            10)

      self.minibot_stat_pub =  self.create_publisher(Int32, '/minibot_a_stat', 10)

    def hubbot_listener_callback(self, msg: Int32):
        self.get_logger().info('I heard: "%s"' % msg.data)

        if  msg.data == HubbotStats.STAT.HubReadyForMinibotUndocking :
            self.get_logger().info('Hubbot indicating We can begin UNDOCKING sequence.')
            # if( not self.recently_docked) :
            self.goal_idx = UNDOCK
            self.can_undock = True
            # new_minibot_status_handler(self.minibot_a_current_stat)
        elif msg.data == HubbotStats.STAT.HubReadyForMinibotDocking:
            self.get_logger().info('Hubbot indicating We can begin DOCKING sequence.')
            self.goal_idx = SEARCH
            self.can_undock = False
            self.can_dock = True

        else:
            print("Non-New Minibot Status Recieved: " +
                  str(tempStat.name) + ":" + str(tempStat.value))
        

    def state_machine(self):
      """
      Navigate from somewhere in the environment to a staging area near
      the charging dock.
      """    
      global low_battery
      
      # If we have enough battery, don't navigate to the charging dock.
      if low_battery == False:
        return None 
      
      # Launch the ROS 2 Navigation Stack
      # navigator = BasicNavigator()
      # navigator.cancelTask()

      if (self.goal_idx == NONE ): 
        self.get_logger().info('Awaiting Docking Command.')
        return
        
      # if (self.goal_idx != 3 and self.goal_idx != -1): 
      if (self.goal_idx == SEARCH or self.goal_idx == FOUND):
        self.get_logger().info('Recieved Docking Command. Navigating to the charging dock...')
        self.connect_to_dock()
      
      if(self.goal_idx == WAIT_UNDOCK):
        self.get_logger().info('we are docked, but cannot yet undock.')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(cmd_vel_msg)
        

      if(self.goal_idx == UNDOCK and self.can_undock):
        self.undock()
      

    def connect_to_dock(self): 
      """
      Go to the charging dock.
      """ 
      counter = 0
      # variable that holds the battery swapping notification.
      # if the battery has been swaped, and we are currently at goal_idx == 2
      # we will then do the undokcing sequence 
      msg = Int32()

      # msg.data = MinibotStats.STAT.MiniNormalOperating
      # self.minibot_stat_pub.publish(msg)

      # While the battery is not charging
      while this_battery_state.power_supply_status != 1 and self.goal_idx != -1 and not self.can_undock:
      # while self.goal_idx != 1:

        # Publish the current battery state
        # self.get_logger().info('NOT CHARGING...')
        counter = counter + 1
        msg = Int32()
        
        if (self.goal_idx == SEARCH ):
          self.search_for_aruco_marker()
          if (counter % 20 == 0):
            self.get_logger().info('Searching for the ArUco marker...')
            msg.data = MinibotStats.STAT.MiniSearchingForHub
            self.minibot_stat_pub.publish(msg)
            counter = 0
        elif (self.goal_idx == FOUND):
          self.navigate_to_aruco_marker()
          # print every 100 loops
          msg.data = MinibotStats.STAT.MiniSearchingForHub
          if (counter % 20 == 0):
            self.get_logger().info('Navigating to ArUco marker')
            self.get_logger().info('Distance: ' + '{:.2f}'.format(obstacle_distance_front) + ' meters.')
            self.minibot_stat_pub.publish(msg)
            counter = 0

        else:
          # Stop the robot
          # self.goal_idx = -1
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = 0.0
          cmd_vel_msg.angular.z = 0.0
          self.publisher_cmd_vel.publish(cmd_vel_msg)
          self.get_logger().info('Arrived at charging dock. Robot is idle...')
          msg = Int32()
          msg.data = MinibotStats.STAT.MiniDocked
          self.minibot_stat_pub.publish(msg)
          # self.goal_idx = WAIT_UNDOCK
          if (self.can_undock):
            self.goal_idx = UNDOCK
          return 

          

          
    
        time.sleep(0.02)
    
      self.get_logger().info('CHARGING...')
      self.get_logger().info('Successfully connected to the charging dock!')
      cmd_vel_msg = Twist()
      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      self.publisher_cmd_vel.publish(cmd_vel_msg)
      
      # Reset the node
      self.goal_idx = NONE

      # While the battery is not full
      # while this_battery_state.percentage != 1.0:      
      #   self.get_logger().info('CHARGING...')
        
 
      # -1 means undock
      # while self.goal_idx == -1  or obstacle_distance_front <= self.undocking_distance or '{:.2f}'.format(obstacle_distance_front) == "nan":
      #        # Undock from the docking station
      #   if( aruco_marker_distance < 0.3):
      #     self.get_logger().info('UNDOCKING...')
      #     cmd_vel_msg = Twist()
      #     cmd_vel_msg.linear.x = -self.linear_velocity
      #     self.publisher_cmd_vel.publish(cmd_vel_msg)
      #   else:
      #     self.get_logger().info('Done Undocking...')
      #     self.goal_idx == 0

      #   # cmd_vel_msg = Twist()
      #   # cmd_vel_msg.linear.x = -0.5
      #   # self.publisher_cmd_vel.publish(cmd_vel_msg)
      #   self.get_logger().info('Undocking from the charging dock...')
      #   self.get_logger().info('Distance: ' + '{:.2f}'.format(obstacle_distance_front) + ' meters.')
        
        
      
      # Stop the robot
      cmd_vel_msg = Twist()
      cmd_vel_msg.linear.x = 0.0     
      self.publisher_cmd_vel.publish(cmd_vel_msg)
      self.get_logger().info('Ready for my next goal!')

    def undock(self): 
      while self.goal_idx == -1: # or obstacle_distance_front <= self.undocking_distance or '{:.2f}'.format(obstacle_distance_front) == "nan":
             # Undock from the docking station
        
        if( aruco_marker_distance < 0.70):
          self.get_logger().info('UNDOCKING...')
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = -self.linear_velocity
          self.publisher_cmd_vel.publish(cmd_vel_msg)
        else:
          msg = Int32()
          msg.data = MinibotStats.STAT.MiniNormalOperating
          self.minibot_stat_pub.publish(msg)
          self.get_logger().info('Done Undocking...')
          self.goal_idx = 0
          self.can_undock = False
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = 0.0
          self.publisher_cmd_vel.publish(cmd_vel_msg)


    def search_for_aruco_marker(self):
      """
      Rotate around until the robot finds the charging dock
      """
      if aruco_marker_detected == False:
      
        # Create a velocity message
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = -self.angular_velocity           
      
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 
      else: 
        self.goal_idx = FOUND
    
    def navigate_to_aruco_marker(self):
      """
      Go straight to the ArUco marker
      """
      self.get_logger().info('Distance: ' + '{:.2f}'.format(aruco_marker_distance) + ' meters.')
      formatted_distance=  '{:.2f}'.format(obstacle_distance_front)
      # prev_readings.append(obstacle_distance_front)
      # # limit the buffer to 15 items
      # if(len(prev_readings) > 30) :
        # prev_readings = []
      if (aruco_marker_distance < ARUCO_MARKER_MIN_DISTANCE): 
        # prev_readings.append(aruco_marker_distance)
        # if(len(prev_reading) > 5) :
        #   list_sum = sum(prev_readings)
        #   avg = list_sum/len(prev_reading)
        #   prev_readings = []
        #   self.get_logger().info('Average distance ' + '{:.2f}'.format(avg) + ' meters.')
        #   if(formatted_distance == "nan" or avg < 0.12):
      #   for i in range(300): 
      #       cmd_vel_msg = Twist()
      #       cmd_vel_msg.linear.x = 0.4 
      #       self.publisher_cmd_vel.publish(cmd_vel_msg)  
      #   # for r in prev_readings:
      #     # if r <= 0.6 or '{:.2f}'.format(r) == 'nan':
      #   # check previous 5 readings. If any of them contain a integer value, that means that it has been getting closer.         
        self.get_logger().info('we are at the dock and less than 5 cm away.')
        self.goal_idx = WAIT_UNDOCK
        return
        
            # break
      # If we have detected the ArUco marker and there are no obstacles in the way
      # elif aruco_marker_detected and (obstacle_distance_front > self.obstacle_tolerance):
      if aruco_marker_detected and (abs(aruco_center_offset) > self.center_offset_tolerance):
        self.adjust_heading()
      # If we have detected the ArUco marker and there are obstacles in the way, we have reached the charging dock
      # elif aruco_marker_detected and (obstacle_distance_front <= self.obstacle_tolerance):
      #   self.goal_idx = WAIT_UNDOCK
      # If we have not detected the ArUco marker, and there is an obstacle in the way at a close distance,
      # we have reached the charging dock
      elif not aruco_marker_detected :# and (obstacle_distance_front <= self.obstacle_tolerance):
        self.goal_idx = SEARCH
      # # Search for charging dock  
      
           
      elif (aruco_marker_distance > 0.11):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.4 
        self.publisher_cmd_vel.publish(cmd_vel_msg) 

      else: 
        self.goal_idx = WAIT_UNDOCK

        # self.goal_idx = SEARCH
      
    def adjust_heading(self):
      """
      Adjust heading to keep the Aruco marker centerpoint centererd.
      """
      cmd_vel_msg = Twist()
      if aruco_center_offset < -self.center_offset_tolerance:
        # Turn left
        cmd_vel_msg.angular.z = self.angular_velocity   
      elif aruco_center_offset > self.center_offset_tolerance:  
        # Turn right       
        cmd_vel_msg.angular.z = -self.angular_velocity 
      else:
        # Go straight
        cmd_vel_msg.linear.x = self.linear_velocity 
      
      if(aruco_marker_distance > 1) :
          # the further away, the more slower the turns, this is to minimize the offset.  
        cmd_vel_msg.angular.z = cmd_vel_msg.angular.z * 0.80
        # cmd_vel_msg.linear.x = self.linear_velocity / 2
      # Publish the velocity message  
      self.publisher_cmd_vel.publish(cmd_vel_msg)  

class BatteryStateSubscriber(Node):
    """
    Subscriber node to the current battery state
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('battery_state_subscriber')
    
      # Create a subscriber 
      # This node subscribes to messages of type
      # sensor_msgs/BatteryState
      self.subscription_battery_state = self.create_subscription(
        BatteryState,
        '/battery_status',
        self.get_battery_state,
        10)
      
    def get_battery_state(self, msg):
      """
      Update the current battery state.
      """
      global this_battery_state
      global prev_battery_state
      global low_battery
      prev_battery_state = this_battery_state
      this_battery_state = msg
      # self.get_logger().info('test')
      
      # self.get_logger().info('prev_battery_state.percentage: ' + str(prev_battery_state.percentage))
      # self.get_logger().info('this_battery_state.percentage: ' + str(this_battery_state.percentage))


      
      # Check for low battery
      if prev_battery_state.percentage <= low_battery_min_threshold and this_battery_state.percentage < low_battery_min_threshold:
        # self.get_logger().info('in here')
        low_battery = True
        
class ArucoMarkerSubscriber(Node):
    """
    Subscriber node to help for the ArUco marker navigation routine.
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('aruco_marker_subscriber')
    
      # Create a subscriber 
      # This node subscribes to messages of type
      # std_msgs/Bool
      self.subscription_aruco_detected = self.create_subscription(
        Bool,
        '/aruco_marker_detected', 
        self.get_aruco_detected,
        1)

      # Create a subscriber 
      # This node subscribes to messages of type
      # std_msgs/Int32
      self.subscription_center_offset = self.create_subscription(
        Int32,
        '/aruco_marker_offset', 
        self.get_center_offset,
        1)
        
      self.publisher_aruco_marker_id = self.create_subscription(
        Int32,
        '/aruco_marker_id', 
        self.get_marker_id,
        1)

      self.publisher_aruco_marker_distance = self.create_subscription(
        Float64,
        '/aruco_marker_distance', 
        self.get_marker_distance,
        1)
        
      # Create a subscriber 
      # This node subscribes to messages of type
      # sensor_msgs/LaserScan
      self.subscription_laser_scan = self.create_subscription(
        LaserScan,
        scan_topic, 
        self.scan_callback,
        qos_profile=qos_profile_sensor_data)

    def get_aruco_detected(self, msg):
      """
      Update if the ArUco marker has been detected or not
      """
      global aruco_marker_detected 
      aruco_marker_detected = msg.data
        
    def get_center_offset(self, msg):
      """
      Update the ArUco marker center offset
      """
      global aruco_center_offset
      aruco_center_offset = msg.data

    def get_marker_id(self, msg):
      """
      Update the ArUco marker id
      """
      global aruco_marker_id
      aruco_marker_id = msg.data

    def get_marker_distance(self, msg):
      """
      Update the ArUco marker id
      """
      global aruco_marker_distance
      aruco_marker_distance = msg.data
            
    def scan_callback(self, msg):
      """
      Update obstacle distance.
      """
      global obstacle_distance_front
      
      # obstacle_distance_front = msg.ranges[179]
      obstacle_distance_front = msg.ranges[100]

        
def main(args=None):
  """
  Entry point for the program.
  """
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  try: 
  
    # Create the nodes
    connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
    battery_state_subscriber = BatteryStateSubscriber()
    aruco_marker_subscriber = ArucoMarkerSubscriber()
    
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(connect_to_charging_dock_navigator)
    executor.add_node(battery_state_subscriber)
    executor.add_node(aruco_marker_subscriber)
    
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
      
    finally:
      # Shutdown the nodes
      executor.shutdown()
      connect_to_charging_dock_navigator.destroy_node()
      battery_state_subscriber.destroy_node()
      aruco_marker_subscriber.destroy_node()
      cmd_vel_msg = Twist()
      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      self.publisher_cmd_vel.publish(cmd_vel_msg)

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
  main()

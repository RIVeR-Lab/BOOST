# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from .bot_stats import HubbotStats, MinibotStats
from .axis_control import swap_battery, init_axis_control
from .charge_controller import ChargeController
import sys
import os


class HubbotMainControllerNode(Node):
    PUBLISH_PERIOD_S = 1.0
    CHARGE_CONTROLLER_LOOP_TIME_S = 0.5
    VERBOSE_CHARGER_CONTROLLER = False

    hubbot_current_stat: HubbotStats.STAT
    minibot_a_current_stat: MinibotStats.STAT

    def __init__(self):
        super().__init__('hubbot_main_controller_node')
        self.__init_hubbot_stat_pub()
        self.__init_minibot_a_sub()
        self.__init_charge_controller()
        self.__init_axis_controller()
        self.hubbot_current_stat = HubbotStats.STAT.HubReadyForMinibotDocking

    def __init_hubbot_stat_pub(self):
        self.publisher_ = self.create_publisher(Int32, 'hub_stat', 10)
        timer_period = self.PUBLISH_PERIOD_S
        self.hubbot_stat_pub_timer = self.create_timer(
            timer_period, self.hubbot_stat_pub_callback)
        self.i = 0
        self.hubbot_current_stat = HubbotStats.STAT.HubNotReady

    def __init_minibot_a_sub(self):
        self.subscription = self.create_subscription(
            Int32,
            'minibot_a_stat',
            self.minibot_a_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.minibot_a_current_stat = MinibotStats.STAT.MiniUnknown

    def __init_charge_controller(self):
        self.charge_controller = ChargeController()
        timer_period = self.CHARGE_CONTROLLER_LOOP_TIME_S
        self.charge_controller_timer = self.create_timer(
            timer_period, self.charge_controller_callback)

    def __init_axis_controller(self):
        init_axis_control()
    
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    def enablePrint(self):
        sys.stdout = sys.__stdout__

    def charge_controller_callback(self):
        if not self.VERBOSE_CHARGER_CONTROLLER:
            self.blockPrint()
        self.charge_controller.loopHook()
        if not self.VERBOSE_CHARGER_CONTROLLER:
            self.enablePrint()

    def hubbot_stat_pub_callback(self):
        msg = HubbotStats().stat_to_ros_msg(HubbotStats.STAT.HubNotReady)
        msg.data = self.hubbot_current_stat.value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s:%s"' %
                               (self.hubbot_current_stat.name, msg.data))
        self.i += 1
        print("Current Minibot Status: " + str(self.minibot_a_current_stat.name) +
              ":" + str(self.minibot_a_current_stat.value))

    def new_minibot_status_handler(self, newStat: MinibotStats.STAT):
        '''
        Main state machine for
        '''
        print("New Minibot Status: " + str(newStat.name) +
              ":" + str(newStat.value))

        if newStat == MinibotStats.STAT.MiniUnknown:
            print("Minibot status unknown.")
        elif newStat = MinibotStats.STAT.MiniNormalOperating:
            self.hubbot_current_stat = HubbotStats.STAT.HubReadyForMinibotUndocking
        elif newStat = MinibotStats.STAT.MiniSearchingForHub:
            print("Minibot searching for hub")
        elif newStat == MinibotStats.STAT.MiniDocked:
            success: bool = swap_battery()
            if success:
                print("\n\nSUCCESSFULLY SWAPPED BATTERY!!\n\n")
            else:
                print("\n\nBATTERY SWAP FAILED!!\n\n")
            self.hubbot_current_stat = HubbotStats.STAT.HubReadyForMinibotUndocking

    def minibot_a_listener_callback(self, msg: Int32):
        self.get_logger().info('I heard: "%s"' % msg.data)
        tempStat: MinibotStats.STAT = MinibotStats.ros_msg_to_stat(msg)
        if tempStat != self.minibot_a_current_stat:
            print("New Minibot Status Recieved: " +
                  str(tempStat.name) + ":" + str(tempStat.value))
            self.minibot_a_current_stat = tempStat
            self.new_minibot_status_handler(self.minibot_a_current_stat)
        else:
            print("Non-New Minibot Status Recieved: " +
                  str(tempStat.name) + ":" + str(tempStat.value))


def main(args=None):
    rclpy.init(args=args)
    hubbot_main_controller_node = HubbotMainControllerNode()
    rclpy.spin(hubbot_main_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hubbot_main_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

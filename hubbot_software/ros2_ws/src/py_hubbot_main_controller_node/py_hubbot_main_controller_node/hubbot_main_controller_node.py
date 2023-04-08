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


class HubbotStatusPublisher(Node):
    PUBLISH_PERIOD_S = 0.5
    current_stat: HubbotStats.STAT = HubbotStats.STAT.HubNotReady

    def __init__(self):
        super().__init__('HubbotStatPub')
        self.publisher_ = self.create_publisher(Int32, 'hubstatus', 10)
        timer_period = self.PUBLISH_PERIOD_S
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = HubbotStats().stat_to_ros_msg(HubbotStats.STAT.HubNotReady)
        msg.data = self.current_stat.value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s:%s"' %
                               (self.current_stat.name, msg.data))
        self.i += 1


class MinibotStatusSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'minibotstat',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    hubbot_stat_pub = HubbotStatusPublisher()
    minibot_status_subscriber = MinibotStatusSubscriber()

    rclpy.spin(hubbot_stat_pub)
    rclpy.spin(minibot_status_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hubbot_stat_pub.destroy_node()
    minibot_status_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
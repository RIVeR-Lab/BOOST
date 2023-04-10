from std_msgs.msg import Int32
from enum import IntEnum, auto
import unittest

class HubbotStats:
  class STAT(IntEnum):
    HubUnknown = -1
    HubNotReady = 1
    HubReadyForMinibotDocking = 2
    HubSwappingBattery = 3
    HubReadyForMinibotUndocking = 4

  @classmethod
  def ros_msg_to_stat(self, msg: Int32) -> STAT:
    print("Hubbot ros_msg_to_stat input" + str(msg))
    return self.STAT(msg.data)

  @classmethod
  def stat_to_ros_msg(self, stat: STAT) -> Int32:
    print("Hubbot stat_to_ros_msg input: " + str(stat))
    tmpMsg = Int32()
    tmpMsg.data = stat.value
    return tmpMsg


class MinibotStats:
  class STAT(IntEnum):
    MiniUnknown = -1
    MiniNormalOperating = 1
    MiniSearchingForHub = 2
    MiniDocked = 3

  @classmethod
  def ros_msg_to_stat(self, msg: Int32) -> STAT:
    print("msg" + str(msg))
    return self.STAT(msg.data)

  @classmethod
  def stat_to_ros_msg(self, stat: STAT) -> Int32:
    print("state" + str(stat))
    msg = Int32()
    msg.data = stat.value
    return msg

# TESTS
class Tests(unittest.TestCase):
  def test_MinibotStats_ros_msg_to_stat(self):
    print("Testing test_MinibotStats_ros_msg_to_stat")
    ros_msg = Int32()

    ros_msg.data = 1
    expected = MinibotStats.STAT(ros_msg.data)
    actual = MinibotStats.ros_msg_to_stat(ros_msg)
    assert(actual, expected)

    ros_msg.data = 2
    expected = MinibotStats.STAT(ros_msg.data)
    actual = MinibotStats.ros_msg_to_stat(ros_msg)
    assert(actual, expected)

  def test_MinibotStats_stat_to_msg(self):
    print("Testing test_MinibotStats_stat_to_msg")
    expected_ros_msg = Int32()

    stat = MinibotStats.STAT.MiniDocked
    expected_ros_msg.data = stat.value
    actual_ros_msg = MinibotStats.stat_to_ros_msg(stat)
    assert(actual_ros_msg, expected_ros_msg)

    stat = MinibotStats.STAT.MiniNormalOperating
    expected_ros_msg.data = stat.value
    actual_ros_msg = MinibotStats.stat_to_ros_msg(stat)
    assert(actual_ros_msg, expected_ros_msg)

  def test_HubbotStats_ros_msg_to_stat(self):
    print("Testing test_HubbotStats_ros_msg_to_stat")
    ros_msg = Int32()

    ros_msg.data = 1
    expected = HubbotStats.STAT(ros_msg.data)
    actual = HubbotStats.ros_msg_to_stat(ros_msg)
    assert(actual, expected)

    ros_msg.data = 2
    expected = HubbotStats.STAT(ros_msg.data)
    actual = HubbotStats.ros_msg_to_stat(ros_msg)
    assert(actual, expected)

  def test_HubbotStats_stat_to_msg(self):
    print("Testing test_HubbotStats_stat_to_msg")
    expected_ros_msg = Int32()

    stat = HubbotStats.STAT.HubNotReady
    expected_ros_msg.data = stat.value
    actual_ros_msg = HubbotStats.stat_to_ros_msg(stat)
    assert(actual_ros_msg, expected_ros_msg)

    stat = HubbotStats.STAT.HubReadyForMinibotUndocking
    expected_ros_msg.data = stat.value
    actual_ros_msg = HubbotStats.stat_to_ros_msg(stat)
    assert(actual_ros_msg, expected_ros_msg)

if __name__ == '__main__':
    unittest.main()

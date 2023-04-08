from std_msgs.msg import Int32
from enum import IntEnum, auto


class HubbotStats:
  class STAT(IntEnum):
    HubNotReady = auto()
    HubReadyForMinibotDocking = auto()
    HubReadyForMinibotUndocking = auto()

  

  @classmethod
  def ros_msg_to_stat(msg: Int32) -> STAT:
    print("msg" + msg)
    return STAT(msg.data)


  @classmethod
  def stat_to_ros_msg(stat: STAT) -> Int32:
    print("state" + stat)
    return Int32(stat.value)


class MinibotStats:
  class STAT(IntEnum):
    MiniBattLow = auto()
    MiniDocked = auto()
    # @classmethod
    # def get
    # def __init__(self):


def main():
    print(HubbotStats.STATS.HubReadyForMinibotDocking)


if __name__ == '__main__':
    main()

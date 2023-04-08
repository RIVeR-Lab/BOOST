from std_msgs.msg import Int32
from enum import Enum


class HubbotStats:
    # Stats
    STATS = Enum('HubbotStats',
                 ['HubReadyForMinibotDocking',
                  'HubReadyForMinibotUndocking',

                  ])
    # @classmethod
    # def get
    # def __init__(self):


class MinibotStats:
    # Stats
    STATS = Enum('MinibotStates',
                 ['MiniBattLow',
                  'MiniDocked',

                  ])
    # @classmethod
    # def get
    # def __init__(self):


def main():
  print(HubbotStats.STATS.HubReadyForMinibotDocking)

if __name__ == '__main__':
    main()

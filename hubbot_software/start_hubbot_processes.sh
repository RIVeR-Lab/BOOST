#! /bin/bash
BASEPATH=$(pwd)
VERBOSE=0

if [[ "$1" == "-v" ]]; then
    VERBOSE=1
else
    VERBOSE=0
fi

echo $VERBOSE

echo "Starting ChargeController..."
if [[ "$VERBOSE" -eq 1 ]]; then
  # python3 $BASEPATH/charge_controller.py &
  cd "ros2_ws" && sudo colcon build && source install/setup.sh && ros2 run py_hubbot_main_controller_node hubbot_main_controller_node
else
  # python3 $BASEPATH/charge_controller.py > /dev/null 2>&1 &
  cd "ros2_ws" && sudo colcon build && source install/setup.sh && ros2 run py_hubbot_main_controller_node hubbot_main_controller_node
fi

echo "DONE"
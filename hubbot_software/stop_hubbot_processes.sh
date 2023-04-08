#! /bin/bash
BASEPATH=$(pwd)
P1=$BASEPATH/charge_controller.py

kill $(ps aux | grep "$P1" | awk '{print $2}')
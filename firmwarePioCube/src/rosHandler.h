#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "utils/log.h"
#include <math.h>
#include <algorithm>

class RosHandler {
public:
    RosHandler(HardwareSerial &serialPort) : nodeHardware(serialPort), chatter("chatter", &str_msg),
    subDiffDrive("/cmd_vel", &subDiffDrive_cb) 
    {

    };
    ~RosHandler(){};

    static constexpr uint32_t rosSerialBaud = 57600;

    bool init() {
        bool success = true;
        
        nodeHandle.getHardware()->setBaud(rosSerialBaud);
        nodeHandle.getHardware()->setPort(const_cast<HardwareSerial*>(&nodeHardware));
        nodeHandle.initNode();
        nodeHandle.advertise(chatter);
        nodeHandle.subscribe(subDiffDrive);

        if(!success) {
            LOGERROR("Failed to initialize ROS node.");
         } else {
            LOGINFO("Successfully initialized ROS node.");
         }

        return success;
    }

    bool loop() {
        static uint32_t counter = 0;
        bool success = true;

        // if(!nodeHandle.connected()){
        //     isRosConnected = false;
        // } else {
            isRosConnected = true;
            // Only publish chatter every 1 second
            
            if((millis() - counter) > 1000) {
                counter = millis();
                str_msg.data = str_msg_data.c_str();
                chatter.publish( &str_msg );
            }
            nodeHandle.spinOnce();
        // }

        return success;
    }

    ros::NodeHandle nodeHandle;

private: 
    const HardwareSerial &nodeHardware;
    bool isRosConnected = false;

    static constexpr float WHEEL_BASE = 0.2; // (meters per radian)
    static constexpr float WHEEL_RADIUS = 0.02; // (meters per radian)
    static constexpr float PWM_MIN = 0;
    static constexpr float PWM_MAX = 255;
    static constexpr float MOTOR_RPM = 60;
    static constexpr float MOTOR_MAX_RAD_PER_SEC = MOTOR_RPM * ((2.0 * M_PI) / 60.0); // rad/s
    static constexpr float MOTOR_MIN_RAD_PER_SEC = -MOTOR_MAX_RAD_PER_SEC; // rad/s
    


    // PUBLISHERS
    ros::Publisher chatter;
    std_msgs::String str_msg;
    std::string str_msg_data = "Chatter Topic: MCU blink!";

    // SUBSCRIBERS
    // For diff drive control
    ros::Subscriber<geometry_msgs::Twist> subDiffDrive;
    static void subDiffDrive_cb(const geometry_msgs::Twist &msg);

};


#endif  // ROSHANDLER_H

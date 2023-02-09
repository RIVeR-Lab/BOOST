#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros.h>
#include <std_msgs/String.h>

class RosHandler {
public:
    RosHandler() : nodeHardware(Serial2), chatter("chatter", &str_msg) {

    };
    ~RosHandler(){};
    bool init() {
        bool success = true;
        
        nodeHandle.getHardware()->setBaud(rosSerialBaud);
        nodeHandle.getHardware()->setPort(const_cast<HardwareSerial*>(&nodeHardware));
        nodeHandle.initNode();
        nodeHandle.advertise(chatter);

        return success;
    }

    bool loop() {
        bool success = true;

        if(!nodeHandle.connected()){
            isRosConnected = false;
        } else {
            isRosConnected = true;
            str_msg.data = str_msg_data.c_str();
            chatter.publish( &str_msg );
            nodeHandle.spinOnce();
        }

        return success;
    }

private:
    static constexpr uint32_t rosSerialBaud = 57600;
    const HardwareSerial &nodeHardware;
    ros::NodeHandle nodeHandle;


    // PUBLISHERS
    ros::Publisher chatter;
    std_msgs::String str_msg;
    std::string str_msg_data = "Chatter Topic: MCU blink!";

    // SUBSCRIBERS
    // ros::Subscriber();

};


#endif  // ROSHANDLER_H

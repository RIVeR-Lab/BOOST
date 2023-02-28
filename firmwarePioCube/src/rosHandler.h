#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "utils/log.h"
#include <math.h>
#include <algorithm>
#include "FakeThread.h"

class RosHandler : public FakeThread {
public:
    RosHandler(HardwareSerial &serialPort) : nodeHardware(serialPort), chatter("chatter", &str_msg), 
    bno055_imu_pub("bno055_imu", &bno055_imu_msg),
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

    bool loopHook();

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
    


    /* -------------------------- PUBLISHERS -------------------------- */
    // Chatter
    ros::Publisher chatter;
    std_msgs::String str_msg;
    std::string str_msg_data = "Chatter Topic: MCU blink!";
    // IMUS
    ros::Publisher bno055_imu_pub;
    sensor_msgs::Imu bno055_imu_msg;
    /* -------------------------- END PUBLISHERS -------------------------- */

    /* -------------------------- SUBSCRIBERS -------------------------- */
    // For diff drive control
    ros::Subscriber<geometry_msgs::Twist> subDiffDrive;
    static void subDiffDrive_cb(const geometry_msgs::Twist &msg);
    /* -------------------------- END SUBSCRIBERS -------------------------- */

};


#endif  // ROSHANDLER_H

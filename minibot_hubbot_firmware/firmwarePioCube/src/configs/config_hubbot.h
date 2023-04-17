#define LOG_LEVEL LOGLEVEL_ERROR  // Used in utils/log.h

// Logs raw serial over the defined 'Console'.
// You must use LOGINFO, LOGERROR, etc. macros in your code to use this.
#define CONSOLE_LOGGING 1

// Logs to ROS over ROS Serial using ROS packets.
// You must use ROSLOGINFO, ROSLOGERROR, etc. macros in your code to use this.
#define ROS_LOGGING 1

#define ROSTOPIC_PREFIX "hubbot"

#define ENABLE_ROSMANAGER_LOGLOOP 0
#define ENABLE_ROSMANAGER 1

#define ENABLE_IMU_LOGLOOP 1
#define ENABLE_IMU 1

#define ENABLE_ODOMETRY_LOGLOOP 0
#define ENABLE_ODOMETRY 0

#define ENABLE_GPS 1
#define ENABLE_GPS_LOGLOOP 1
#define GPS_DEBUG 1

#define ENABLE_DRVMANAGER_LOGLOOP 0
#define ENABLE_DRVMANAGER 0

#define ENABLE_BATTMANAGER_LOGLOOP 0
#define ENABLE_BATTMANAGER 0
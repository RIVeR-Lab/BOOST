#define LOG_LEVEL LOGLEVEL_ERROR  // Used in utils/log.h

#define CONSOLE_LOGGING 0 // Logs raw serial over USB. DO NOT HAVE THIS ON WHEN USING ROSSERIAL!!!
#define ROS_LOGGING 0 // Logs to ROS over ROS Serial using ROS packets

#define ENABLE_ROSMANAGER_LOGLOOP 0
#define ENABLE_ROSHANDLER 0

#define PRINT_IMU_DATA 0
#define ENABLE_IMU 1

#define PRINT_ODOMETRY_DATA 0
#define ENABLE_ODOMETRY 1

#define ENABLE_GPS 1
#define ENABLE_GPS_LOGLOOP 0
#define GPS_DEBUG 0
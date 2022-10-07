cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(swarm_crawler_001)


set(app_sources
    ../src/main.cpp
    ../src/uartBase.cpp
    ../src/ubxNeo6M.cpp
    ../src/gpsManager.cpp
    ../src/3rd_party/TinyGPS++.cpp
    ../src/testGpsConsumerManager.cpp
    ../src/mainThread.cpp
)
target_sources(app PRIVATE ${app_sources})

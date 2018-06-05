cmake_minimum_required(VERSION 3.7)
project(WaveNetwork)

set(CMAKE_CXX_STANDARD 11)

add_definitions(-DNS3_LOG_ENABLE)

include_directories(ns-3.28/build)
link_directories(ns-3.28/build)

set(he2.cc My_util)
add_executable(ns3 he.cc he2.cc wifi.cc my.cc My_util utils.cc)

target_link_libraries(ns3
        ns-3.28/build/libns3.28-buildings-debug.so
        ns-3.28/build/libns3.28-core-debug.so
        ns-3.28/build/libns3.28-wifi-debug.so
        ns-3.28/build/libns3.28-internet-debug.so
        ns-3.28/build/libns3.28-energy-debug.so
        ns-3.28/build/libns3.28-mobility-debug.so
        ns-3.28/build/libns3.28-point-to-point-debug.so
        ns-3.28/build/libns3.28-traffic-control-debug.so
        ns-3.28/build/libns3.28-topology-read-debug.so
        ns-3.28/build/libns3.28-wave-debug.so
        ns-3.28/build/libns3.28-applications-debug.so
        ns-3.28/build/libns3.28-buildings-test-debug.so
        )
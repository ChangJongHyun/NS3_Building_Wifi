cmake_minimum_required(VERSION 3.7)
project(WaveNetwork)

set(CMAKE_CXX_STANDARD 11)

add_definitions(-DNS3_LOG_ENABLE)

include_directories(../ns-3.26/build)
link_directories(../ns-3.26/build)

set(he.cc wireless.cc)
set(he2.cc)
add_executable(ns3 he.cc he2.cc wifi.cc my.cc)

target_link_libraries(ns3
        ../ns-3.26/build/libns3.26-buildings-debug.so
        ../ns-3.26/build/libns3.26-core-debug.so
        ../ns-3.26/build/libns3.26-wifi-debug.so
        ../ns-3.26/build/libns3.26-internet-debug.so
        ../ns-3.26/build/libns3.26-energy-debug.so
        ../ns-3.26/build/libns3.26-mobility-debug.so
        ../ns-3.26/build/libns3.26-point-to-point-debug.so
        ../ns-3.26/build/libns3.26-traffic-control-debug.so
        ../ns-3.26/build/libns3.26-topology-read-debug.so
        ../ns-3.26/build/libns3.26-wave-debug.so
        ../ns-3.26/build/libns3.26-applications-debug.so
        ../ns-3.26/build/libns3.26-buildings-test-debug.so
        )
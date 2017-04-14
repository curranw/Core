project(ARMTOUCH)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB ARMTOUCH_SRC
    "src/*.cpp"
)

file(GLOB ARMTOUCH_INCLUDE
    "include/*.h"
)
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp tf std_srvs)

add_library(${PROJECT_NAME} ${ARMTOUCH_SRC} ${ARMTOUCH_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain ${catkin_LIBRARIES})

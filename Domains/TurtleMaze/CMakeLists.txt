project(TURTLEMAZE)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB TURTLEMAZE_SRC
    "src/*.cpp"
)

file(GLOB TURTLEMAZE_INCLUDE
    "include/*.h"
)
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp tf std_srvs)

add_library(${PROJECT_NAME} ${TURTLEMAZE_SRC} ${TURTLEMAZE_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain ${catkin_LIBRARIES})

project(BALLBALANCE)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB BALLBALANCE_SRC
    "src/*.cpp"
)

file(GLOB BALLBALANCE_INCLUDE
    "include/*.h"
)
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp tf std_srvs)

add_library(${PROJECT_NAME} ${BALLBALANCE_SRC} ${BALLBALANCE_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain ${catkin_LIBRARIES})

project(ROBOTCHASE)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB ROBOTCHASE_SRC
    "src/*.cpp"
)

file(GLOB ROBOTCHASE_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${ROBOTCHASE_SRC} ${ROBOTCHASE_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain)

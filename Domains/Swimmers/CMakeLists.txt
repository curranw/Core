project(SWIMMERS)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB SWIMMERS_SRC
    "src/*.cpp"
)

file(GLOB SWIMMERS_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${SWIMMERS_SRC} ${SWIMMERS_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain X11)

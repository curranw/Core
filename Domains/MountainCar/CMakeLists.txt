project(MTNCAR)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB MTN_SRC
    "src/*.cpp"
)

file(GLOB MTN_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${MTN_SRC} ${MTN_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain)

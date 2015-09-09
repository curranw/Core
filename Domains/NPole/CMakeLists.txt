project(NPOLE)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB NPOLE_SRC
    "src/*.cpp"
)

file(GLOB NPOLE_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${NPOLE_SRC} ${NPOLE_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain)

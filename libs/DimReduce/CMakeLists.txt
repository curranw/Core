project(DIMREDUCE)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB DIMREDUCE_SRC
    "src/*.cpp"
)

file(GLOB DIMREDUCE_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${DIMREDUCE_SRC} ${DIMREDUCE_INCLUDE})

target_link_libraries(${PROJECT_NAME} cfitsio)

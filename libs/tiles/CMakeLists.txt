project(TILES)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB TILES_SRC
    "src/tiles2.cc"
)

file(GLOB TILES_INCLUDE
    "include/tiles2.h"
)

add_library(${PROJECT_NAME} ${TILES_SRC} ${TILES_INCLUDE})

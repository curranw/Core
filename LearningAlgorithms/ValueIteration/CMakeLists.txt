project(VI)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB VI_SRC
    "src/*.cpp"
)

file(GLOB VI_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${VI_SRC} ${VI_INCLUDE})


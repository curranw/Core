project(NN)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB NN_SRC
    "src/*.cpp"
)

file(GLOB NN_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${NN_SRC} ${NN_INCLUDE})

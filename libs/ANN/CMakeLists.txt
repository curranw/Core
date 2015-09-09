project(ANN)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB ANN_SRC
    "src/*.cpp"
)

file(GLOB ANN_INCLUDE
    "include/ANN/*.h"
)

add_library(${PROJECT_NAME} ${ANN_SRC} ${ANN_INCLUDE})

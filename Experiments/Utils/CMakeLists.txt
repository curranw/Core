project(ExperimentUtils)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB UTILS_SRC
    "src/*.cpp"
)

file(GLOB UTILS_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${UTILS_SRC} ${UTILS_INCLUDE})

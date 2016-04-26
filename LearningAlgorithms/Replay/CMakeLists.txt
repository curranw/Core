project(REPLAY)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB REPLAY_SRC
    "src/*.cpp"
)

file(GLOB REPLAY_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${REPLAY_SRC} ${REPLAY_INCLUDE})

project(RMAX)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB RMAX_SRC
    "src/*.cpp"
)

file(GLOB RMAX_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${RMAX_SRC} ${RMAX_INCLUDE})

#target_link_libraries(${PROJECT_NAME} TILES)

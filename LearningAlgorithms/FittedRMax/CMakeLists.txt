project(FITTEDRMAX)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB FITTEDRMAX_SRC
    "src/*.cpp"
)

file(GLOB FITTEDRMAX_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${FITTEDRMAX_SRC} ${FITTEDRMAX_INCLUDE})

target_link_libraries(${PROJECT_NAME} TILES)

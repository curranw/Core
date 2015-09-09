project(QTILESREUSE)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB QTILESREUSE_SRC
    "src/*.cpp"
)

file(GLOB QTILESREUSE_INCLUDE
    "include/*.h"
)

#include_directories(${TILES_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${QTILESREUSE_SRC} ${QTILESREUSE_INCLUDE})

target_link_libraries(${PROJECT_NAME} PCAInterface)


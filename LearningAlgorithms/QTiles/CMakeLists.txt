project(QTILES)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB QTILES_SRC
    "src/*.cpp"
)

file(GLOB QTILES_INCLUDE
    "include/*.h"
)

#include_directories(${TILES_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${QTILES_SRC} ${QTILES_INCLUDE})

target_link_libraries(${PROJECT_NAME} NN doublefann)

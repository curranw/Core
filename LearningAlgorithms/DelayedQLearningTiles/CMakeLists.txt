project(DELAYEDQTILES)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB DELAYEDQTILES_SRC
    "src/*.cpp"
)

file(GLOB DELAYEDQTILES_INCLUDE
    "include/*.h"
)

#include_directories(${TILES_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${DELAYEDQTILES_SRC} ${DELAYEDQTILES_INCLUDE})

#target_link_libraries(${PROJECT_NAME} TILES)

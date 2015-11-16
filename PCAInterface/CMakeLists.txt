project(PCAInterface)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB PCAI_SRC
    "src/*.cpp"
)

file(GLOB PCAI_INCLUDE
    "include/*.h"
)

#include_directories(${PCA_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${PCAI_SRC} ${PCAI_INCLUDE})
target_link_libraries(${PROJECT_NAME} PCA)

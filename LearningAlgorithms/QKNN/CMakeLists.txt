project(QKNN)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB QKNN_SRC
    "src/*.cpp"
)

file(GLOB QKNN_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${QKNN_SRC} ${QKNN_INCLUDE})
target_link_libraries(${PROJECT_NAME} ANN)

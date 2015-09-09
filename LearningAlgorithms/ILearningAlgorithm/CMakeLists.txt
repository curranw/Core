project(ILearningAlgorithm)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB ILEARNINGALGORITHM_SRC
    "src/*.cpp"
)

file(GLOB ILEARNINGALGORITHM_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${ILEARNINGALGORITHM_SRC} ${ILEARNINGALGORITHM_INCLUDE})

INCLUDE_DIRECTORIES(${TILES_INCLUDE_DIR})
#target_link_libraries(${PROJECT_NAME} TILES)

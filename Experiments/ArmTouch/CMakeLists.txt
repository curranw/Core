project(ArmTouchExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB ARMTOUCHEXP_SRC
    "src/*.cpp"
)

file(GLOB ARMTOUCHEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${ARMTOUCHEXP_SRC} ${ARMTOUCHEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms ARMTOUCH IExperiment ANN PCAInterface TILES)


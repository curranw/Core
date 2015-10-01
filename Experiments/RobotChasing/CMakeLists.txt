project(RobotChaseExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB ROBOTCHASEEXP_SRC
    "src/*.cpp"
)

file(GLOB ROBOTCHASEEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${ROBOTCHASEEXP_SRC} ${ROBOTCHASEEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms ROBOTCHASE PCAInterface VI IExperiment)


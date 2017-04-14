project(BallBalanceExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB BALLBALANCEEXP_SRC
    "src/*.cpp"
)

file(GLOB BALLBALANCEEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${BALLBALANCEEXP_SRC} ${BALLBALANCEEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms BALLBALANCE IExperiment ANN PCAInterface TILES)


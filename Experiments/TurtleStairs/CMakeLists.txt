project(TurtleStairsExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB TURTLESTAIRSEXP_SRC
    "src/*.cpp"
)

file(GLOB TURTLESTAIRSEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${TURTLESTAIRSEXP_SRC} ${TURTLESTAIRSEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms TURTLEMAZE IExperiment ANN PCAInterface TILES)


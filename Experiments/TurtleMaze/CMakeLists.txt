project(TurtleMazeExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB TURTLEMAZEEXP_SRC
    "src/*.cpp"
)

file(GLOB TURTLEMAZEEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${TURTLEMAZEEXP_SRC} ${TURTLEMAZEEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms TURTLEMAZE IExperiment ANN PCAInterface TILES)


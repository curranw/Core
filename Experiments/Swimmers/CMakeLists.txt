project(SwimmersExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB SWIMMERSEXP_SRC
    "src/*.cpp"
)

file(GLOB SWIMMERSEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${SWIMMERSEXP_SRC} ${SWIMMERSEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms SWIMMERS IExperiment ANN PCAInterface TILES)


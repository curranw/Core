project(NArmExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB NARMEXP_SRC
    "src/*.cpp"
)

file(GLOB NARMEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${NARMEXP_SRC} ${NARMEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms ANN NARM PCAInterface TILES IExperiment)


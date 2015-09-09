project(NPoleExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB NPOLEEXP_SRC
    "src/*.cpp"
)

file(GLOB NPOLEEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${NPOLEEXP_SRC} ${NPOLEEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms ANN NPOLE PCAInterface TILES IExperiment)


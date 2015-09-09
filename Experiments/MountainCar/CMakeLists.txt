project(MountainCarExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB MTNCAREXP_SRC
    "src/*.cpp"
)

file(GLOB MTNCAREXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${MTNCAREXP_SRC} ${MTNCAREXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms ANN MTNCAR PCAInterface TILES IExperiment)


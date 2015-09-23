project(BlackjackExperiments)
cmake_minimum_required(VERSION 2.8)
include_directories(include)

file(GLOB BLACKJACKEXP_SRC
    "src/*.cpp"
)

file(GLOB BLACKJACKEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${BLACKJACKEXP_SRC} ${BLACKJACKEXP_INCLUDE})
target_link_libraries(${PROJECT_NAME} LearningAlgorithms ANN PCAInterface TILES IExperiment BLACKJACK)


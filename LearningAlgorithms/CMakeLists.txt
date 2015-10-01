project(LearningAlgorithms)
cmake_minimum_required(VERSION 2.8)

add_subdirectory(ILearningAlgorithm)
add_subdirectory(QTiles)
add_subdirectory(QKNN)
add_subdirectory(QTilesReuse)
add_subdirectory(ValueIteration)

add_library(LearningAlgorithms STATIC)
target_link_libraries(LearningAlgorithms QKNN QTILES QTILESREUSE ILearningAlgorithm TILES)

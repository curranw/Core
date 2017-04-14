project(LearningAlgorithms)
cmake_minimum_required(VERSION 2.8)

add_subdirectory(ILearningAlgorithm)
add_subdirectory(QTiles)
add_subdirectory(QKNN)
add_subdirectory(QTilesReuse)
add_subdirectory(ValueIteration)
add_subdirectory(RMax)
add_subdirectory(FittedRMax)
add_subdirectory(Replay)
add_subdirectory(DelayedQLearningTiles)


add_library(LearningAlgorithms dummy.c)
target_link_libraries(LearningAlgorithms QKNN QTILES QTILESREUSE ILearningAlgorithm TILES RMAX FITTEDRMAX DELAYEDQTILES)

project(ManifoldLearning)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
FIND_PACKAGE(OpenMP REQUIRED)
FIND_PACKAGE(Armadillo REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(MANIFOLD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PCAI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/PCAInterface/include)

#libs
set(PCA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libpca-1.3.3/src/lib)
set(ANN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/ANN/include)
set(TILES_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/tiles/include)
set(MLINTERP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/linterp/src/)

#Learning Algorithms
set(ILEARNINGALGORITHM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/ILearningAlgorithm/include)
set(QKNN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/QKNN/include)
set(QTILES_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/QTiles/include)
set(QTILESREUSE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/QTilesReuse/include)
set(VALUEITERATION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/ValueIteration/include)
set(RMAX_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/RMax/include)
set(FRMAX_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/FittedRMax/include)

#Domains
set(IDOMAIN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/IDomain/include)
set(MTN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/MountainCar/include)
set(NARM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/NArm/include)
set(NPOLE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/NPole/include)
set(BLACKJACK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/Blackjack/include)
set(ROBOTCHASE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/RobotChasing/include)

#Experiments
set(IEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/IExperiment/include)
set(MTNCAREXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/MountainCar/include)
set(NARMEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/NArm/include)
set(NPOLEEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/NPole/include)
set(BLACKJACKEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/Blackjack/include)
set(ROBOTCHASEEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/RobotChasing/include)
set(UTILS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/Utils/include)

set(SRC_LIST
    main.cpp
    src/IROS2015.cpp
    src/AAAI2015.cpp
    )

set(HEADER_LIST
    include/IROS2015.h
    include/AAAI2015.h
    )



include_directories(${INCLUDE_DIR} ${SRC_DIR})

#libs
include_directories(${TILES_INCLUDE_DIR} ${ANN_INCLUDE_DIR} ${PCA_INCLUDE_DIR} ${MLINTERP_INCLUDE_DIR})
include_directories(${ARMADILLO_INCLUDE_DIRS})


#Domains
include_directories(${MTN_INCLUDE_DIR} ${IDOMAIN_INCLUDE_DIR} ${NARM_INCLUDE_DIR} ${NPOLE_INCLUDE_DIR} ${BLACKJACK_INCLUDE_DIR} ${ROBOTCHASE_INCLUDE_DIR})

#Experiments
include_directories(${MTNCAREXP_INCLUDE_DIR} ${NARMEXP_INCLUDE_DIR} ${UTILS_INCLUDE_DIR} ${IEXP_INCLUDE_DIR} ${NPOLEEXP_INCLUDE_DIR} ${BLACKJACKEXP_INCLUDE_DIR})

#Learning Algorithms
include_directories(${PCAI_INCLUDE_DIR} ${QKNN_INCLUDE_DIR} ${QTILES_INCLUDE_DIR} ${ILEARNINGALGORITHM_INCLUDE_DIR} ${QTILESREUSE_INCLUDE_DIR}
${ROBOTCHASEEXP_INCLUDE_DIR} ${VALUEITERATION_INCLUDE_DIR} ${RMAX_INCLUDE_DIR} ${FRMAX_INCLUDE_DIR})

add_subdirectory(libs/libpca-1.3.3)
add_subdirectory(libs/ANN)
add_subdirectory(libs/tiles)
add_subdirectory(libs/linterp)
add_subdirectory(PCAInterface)
add_subdirectory(Domains)
add_subdirectory(LearningAlgorithms)
add_subdirectory(Experiments)
#link_directories(/nfs/attic/smartw/users/curranw/ManifoldLearning/Core/libs/OpenBLAS)
#include_directories(/nfs/attic/smartw/users/curranw/Documents/armadillo-6.200.3/include)
add_executable(${PROJECT_NAME} ${SRC_LIST} ${HEADER_LIST})


target_link_libraries(${PROJECT_NAME} PCAInterface LearningAlgorithms MTNCAR Experiments NARM NPOLE BLACKJACK ROBOTCHASE)

target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES} blas)


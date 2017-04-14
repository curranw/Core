#set(CMAKE_CXX_COMPILER "g++")


set(CMAKE_CXX_FLAGS "-std=c++0x -idirafter /usr/X11R6/include")
set(CMAKE_VERBOSE_MAKEFILE "true")
add_definitions(-DCLOCK_GETTIMEOFDAY)
project(ManifoldLearning)
cmake_minimum_required(VERSION 2.8)
FIND_PACKAGE(OpenMP REQUIRED)


FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp tf std_srvs)
#include("/nfs/attic/smartw/users/curranw/Downloads/armadillo-7.500.0")
FIND_PACKAGE(Armadillo REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(MANIFOLD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PCAI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/PCAInterface/include)

#libs
set(PCA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libpca-1.3.3/src/lib)
set(ANN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/ANN/include)
set(TILES_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/tiles/include)
set(KPCA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/kpca-eigen/include)
set(DIMREDUCE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/DimReduce/include)
set(NN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/NeuralNetwork/include)

#Learning Algorithms
set(ILEARNINGALGORITHM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/ILearningAlgorithm/include)
set(QKNN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/QKNN/include)
set(QTILES_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/QTiles/include)
set(QTILESREUSE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/QTilesReuse/include)
set(VALUEITERATION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/ValueIteration/include)
set(RMAX_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/RMax/include)
set(FRMAX_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/FittedRMax/include)
set(REPLAY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/Replay/include)
set(DELAYEDQTILES_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LearningAlgorithms/DelayedQLearningTiles/include)

#Domains
set(IDOMAIN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/IDomain/include)
set(MTN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/MountainCar/include)
set(NARM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/NArm/include)
set(NPOLE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/NPole/include)
set(BLACKJACK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/Blackjack/include)
set(ROBOTCHASE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/RobotChasing/include)
set(SWIMMERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/Swimmers/include)
set(TURTLEMAZE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/TurtleMaze/include)
set(TURTLESTAIRS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/TurtleStairs/include)
set(ARMTOUCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/ArmTouch/include)
set(BALLBALANCE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Domains/BallBalance/include)

#Experiments
set(IEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/IExperiment/include)
set(MTNCAREXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/MountainCar/include)
set(NARMEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/NArm/include)
set(NPOLEEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/NPole/include)
set(BLACKJACKEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/Blackjack/include)
set(ROBOTCHASEEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/RobotChasing/include)
set(SWIMMERSEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/Swimmers/include)
set(TURTLEMAZEEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/TurtleMaze/include)
set(TURTLESTAIRSEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/TurtleStairs/include)
set(ARMTOUCHEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/ArmTouch/include)
set(BALLBALANCEEXP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/BallBalance/include)
set(UTILS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Experiments/Utils/include)

set(SRC_LIST
    main.cpp
    src/IROS2015.cpp
    src/AAAI2015.cpp
    src/HRI2016.cpp
    src/IROS2017.cpp
    src/RobotExperiments.cpp
    )

set(HEADER_LIST
    include/IROS2015.h
    include/AAAI2015.h
    include/HRI2016.h
    include/IROS2017.h
    include/RobotExperiments.h
    )


#include_directories("/usr/include/eigen3")

include_directories(${INCLUDE_DIR} ${SRC_DIR})
#libs
include_directories(${TILES_INCLUDE_DIR} ${ANN_INCLUDE_DIR} ${PCA_INCLUDE_DIR} ${MLINTERP_INCLUDE_DIR})
include_directories(${ARMADILLO_INCLUDE_DIRS} ${DIMREDUCE_INCLUDE_DIR} ${NN_INCLUDE_DIR})


#Domains
include_directories(${MTN_INCLUDE_DIR} ${IDOMAIN_INCLUDE_DIR} ${NARM_INCLUDE_DIR} ${NPOLE_INCLUDE_DIR}
    ${BLACKJACK_INCLUDE_DIR} ${ROBOTCHASE_INCLUDE_DIR} ${SWIMMERS_INCLUDE_DIR} ${TURTLEMAZE_INCLUDE_DIR}
    ${TURTLESTAIRS_INCLUDE_DIR} ${ARMTOUCH_INCLUDE_DIR} ${BALLBALANCE_INCLUDE_DIR})

#Experiments
include_directories(${MTNCAREXP_INCLUDE_DIR} ${NARMEXP_INCLUDE_DIR} ${UTILS_INCLUDE_DIR} ${IEXP_INCLUDE_DIR}
    ${NPOLEEXP_INCLUDE_DIR} ${BLACKJACKEXP_INCLUDE_DIR} ${SWIMMERSEXP_INCLUDE_DIR} ${TURTLEMAZEEXP_INCLUDE_DIR}
    ${TURTLESTAIRSEXP_INCLUDE_DIR} ${ARMTOUCHEXP_INCLUDE_DIR} ${BALLBALANCEEXP_INCLUDE_DIR})

#Learning Algorithms
include_directories(${PCAI_INCLUDE_DIR} ${QKNN_INCLUDE_DIR} ${QTILES_INCLUDE_DIR} ${ILEARNINGALGORITHM_INCLUDE_DIR} ${QTILESREUSE_INCLUDE_DIR}
${ROBOTCHASEEXP_INCLUDE_DIR} ${VALUEITERATION_INCLUDE_DIR} ${RMAX_INCLUDE_DIR} ${FRMAX_INCLUDE_DIR} ${REPLAY_INCLUDE_DIR} ${DELAYEDQTILES_INCLUDE_DIR})

set(_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


set(TAPKEE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/tapkee/include/)
include_directories("/usr/include/eigen3" ${TAPKEE_INCLUDE_DIR})
#include_directories("/usr/local/include/shogun")
include_directories("/nfs/attic/smartw/users/curranw/Downloads/shogun-4.1.0/build/src/shogun")
include_directories("/nfs/attic/smartw/users/curranw/Downloads/armadillo-7.500.0/include")
include_directories("/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/src/Core/cfitsio/include")
link_directories("/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/src/Core/cfitsio/lib")
LINK_DIRECTORIES("/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/src/Core/PCAInterface/libs")
add_subdirectory(PCAInterface)
add_subdirectory(Domains)
add_subdirectory(LearningAlgorithms)
add_subdirectory(Experiments)
add_subdirectory(libs)

#include_directories(/nfs/attic/smartw/users/curranw/Documents/armadillo-6.200.3/include)
add_executable(${PROJECT_NAME} ${SRC_LIST} ${HEADER_LIST})

target_link_libraries(${PROJECT_NAME} PCAInterface LearningAlgorithms MTNCAR Experiments NARM NPOLE BLACKJACK ROBOTCHASE ${ARMADILLO_LIBRARIES} openblas SWIMMERS ARMTOUCH TURTLEMAZE ${catkin_LIBRARIES} TURTLESTAIRS ${PYTHON_LIBRARIES} NN fann)


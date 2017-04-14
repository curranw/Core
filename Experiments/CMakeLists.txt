project(Experiments)
cmake_minimum_required(VERSION 2.8)

add_subdirectory(IExperiment)
add_subdirectory(Utils)
add_subdirectory(MountainCar)
add_subdirectory(NArm)
add_subdirectory(NPole)
add_subdirectory(Blackjack)
add_subdirectory(RobotChasing)
add_subdirectory(Swimmers)
add_subdirectory(TurtleMaze)
add_subdirectory(TurtleStairs)
add_subdirectory(ArmTouch)
add_subdirectory(BallBalance)

add_library(Experiments dummy.c)
target_link_libraries(Experiments TurtleMazeExperiments MountainCarExperiments ExperimentUtils NArmExperiments
    NPoleExperiments BlackjackExperiments RobotChaseExperiments SwimmersExperiments TurtleStairsExperiments ArmTouchExperiments BallBalanceExperiments)


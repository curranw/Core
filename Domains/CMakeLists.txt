project(Domains)
cmake_minimum_required(VERSION 2.8)

add_subdirectory(IDomain)
add_subdirectory(MountainCar)
add_subdirectory(NArm)
add_subdirectory(NPole)
add_subdirectory(Blackjack)
add_subdirectory(RobotChasing)
add_subdirectory(Swimmers)
add_subdirectory(TurtleMaze)

add_library(Domains dummy.c)
target_link_libraries(Domains IDomain MTNCAR NARM NPOLE BLACKJACK TURTLEMAZE)

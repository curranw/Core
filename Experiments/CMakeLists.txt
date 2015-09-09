project(Experiments)
cmake_minimum_required(VERSION 2.8)

add_subdirectory(IExperiment)
add_subdirectory(Utils)
add_subdirectory(MountainCar)
add_subdirectory(NArm)
add_subdirectory(NPole)

add_library(Experiments STATIC)
target_link_libraries(Experiments MountainCarExperiments ExperimentUtils NArmExperiments NPoleExperiments)


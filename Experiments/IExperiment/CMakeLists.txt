project(IExperiment)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB IEXP_SRC
    "src/*.cpp"
)

file(GLOB IEXP_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${IEXP_SRC} ${IEXP_INCLUDE})




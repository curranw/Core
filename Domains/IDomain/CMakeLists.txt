project(IDomain)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB IDOMAIN_SRC
    "src/*.cpp"
)

file(GLOB IDOMAIN_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${IDOMAIN_SRC} ${IDOMAIN_INCLUDE})



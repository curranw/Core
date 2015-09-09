project(PCA)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_library(${PROJECT_NAME} src/lib/pca.cpp src/lib/utils.cpp)

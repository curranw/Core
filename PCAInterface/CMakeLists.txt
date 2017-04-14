project(PCAInterface)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
include_directories(${PYTHON_INCLUDE_DIRS})
file(GLOB PCAI_SRC
    "src/*.cpp"
    "src/keras_model.cc"
)

file(GLOB PCAI_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${PCAI_SRC} ${PCAI_INCLUDE})
target_link_libraries(${PROJECT_NAME} PCA shogun OpenCL DIMREDUCE ${PYTHON_LIBRARIES})

add_dependencies(${PROJECT_NAME} sklearn_interface_generate_messages_cpp)

project(BLACKJACK)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB BLACKJACK_SRC
    "src/*.cpp"
)

file(GLOB BLACKJACK_INCLUDE
    "include/*.h"
)

add_library(${PROJECT_NAME} ${BLACKJACK_SRC} ${BLACKJACK_INCLUDE})
target_link_libraries(${PROJECT_NAME} IDomain)

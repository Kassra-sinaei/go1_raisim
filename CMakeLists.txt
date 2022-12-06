cmake_minimum_required(VERSION 3.10)
project(go1_simulation LANGUAGES CXX)

#Set environment variables
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

#cmake config directories
set(raisim_DIR ./libs/raisim/lib/cmake/raisim)

#find packages
find_package(raisim CONFIG REQUIRED)

#includes
include_directories(
    ./include 
    ./libs/eigen3/include
    # ./libs/osqp/include
    ./libs/qpoases/include 
    # ./libs/matplotlib-cpp 
    /home/romahoney/miniconda3/envs/robots_3.7/include/python3.7m
)

#executables
add_executable(${PROJECT_NAME} 
    ./src/go1_spawn.cpp
)

#Link external libraries
target_link_libraries(${PROJECT_NAME} 
    PUBLIC raisim::raisim pthread 
    ${CMAKE_SOURCE_DIR}/libs/qpoases/build/libs/libqpOASES.a 
    #/home/romahoney/miniconda3/envs/robots_3.7/lib/libpython3.7m.so
)

target_include_directories(${PROJECT_NAME} PUBLIC ./libs/raisim/include)
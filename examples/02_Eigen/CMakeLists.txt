cmake_minimum_required( VERSION 3.5 )

#
# project
#
project(EigenMatrix)
add_executable(eigenMatrix eigenMatrix.cpp)
include_directories("/usr/include/eigen3/")

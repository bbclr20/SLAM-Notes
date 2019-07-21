cmake_minimum_required( VERSION 3.5 )

#
# project
#
project(HelloSlam)
add_executable(helloSLAM helloSLAM.cpp)
target_link_libraries(helloSLAM libShowImage)

#
# external libs
#
add_library(libShowImage SHARED libShowImage.cpp)
target_link_libraries(libShowImage opencv_world)

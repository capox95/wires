cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 11)

project(Wires)

find_package(Boost REQUIRED)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#add_executable( ariadne ariadne.cpp )
#target_link_libraries( ariadne ${OpenCV_LIBS} )

add_executable (segmentation_fast segmentation_fast.cpp) 
target_link_libraries (segmentation_fast ${PCL_LIBRARIES})
target_link_libraries(segmentation_fast ${Boost_LIBRARIES})

#add_executable (seg_debug seg_debug.cpp) 
#target_link_libraries (seg_debug ${PCL_LIBRARIES})
#target_link_libraries(seg_debug ${Boost_LIBRARIES})



































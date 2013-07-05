cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative)

find_package(PCL)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (kinect Includes/functions.cpp DBscan/dbscan.cpp Show/Show.cpp Matching/Matching.cpp Track/track.cpp Filtry/Filtry.cpp kinect.cpp RGBD/RGBDprocessing.cpp  main.cpp )
target_link_libraries (kinect ${PCL_LIBRARIES} ${OpenCV_LIBS})

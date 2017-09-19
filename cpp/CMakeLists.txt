cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(plant)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/kodda/Dropbox/p2pflab/LettuceScan/PCD_proc/cpp/cmake/)
find_package(PCL 1.8 REQUIRED)
find_package(Jsoncpp)
find_package(OpenCV 3.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS} ${Jsoncpp_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloudgen src/cloudgen.cpp src/cnpy.cpp)
target_link_libraries(cloudgen ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Jsoncpp_LIBRARY})

add_executable(cloudreg src/cloudreg.cpp)
target_link_libraries(cloudreg ${PCL_LIBRARIES} ${Jsoncpp_LIBRARY})


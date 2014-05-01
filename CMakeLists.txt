cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Debug)
project(PCL_SLAM)
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters features registration visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


OPTION(SOFTKINETIC "Build the softkinetic device frontend" OFF)
IF(SOFTKINETIC)
	include_directories(/opt/softkinetic/DepthSenseSDK/include/)
	LINK_DIRECTORIES(/opt/softkinetic/DepthSenseSDK/lib/)
   	add_executable(softkinetic src/softkinetic.cpp src/pcl_slam.cpp)
	target_link_libraries(softkinetic ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES}  DepthSense DepthSensePlugins)
ENDIF(SOFTKINETIC)


add_executable(opennigrabber src/openni_grabber_example.cpp src/pcl_slam.cpp )
target_link_libraries(opennigrabber ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})



cmake_minimum_required(VERSION 2.8.3)
project(vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ################ USE THIS TO EXPOSE HEADER
#  LIBRARIES imgproc_node
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include/vision
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(imgproc_node
#   src/${PROJECT_NAME}/imgproc_node.cpp
# )

add_library(EvalUtils src/utils/EvalUtils.cpp src/utils/EvalUtils.hpp src/utils/Timer.hpp)
add_library(LineDetectionThresholding src/vision/Clusters.hpp src/vision/Clusters.cpp src/vision/linear_transform.hpp src/vision/linear_transform.cpp include/vision/lib_line_detection.hpp src/vision/lib_line_detection.cpp)
add_library(FlagDetection include/vision/lib_flag_detection.hpp src/vision/lib_flag_detection.cpp)

# create some executables so that we can test the libraries without changing the ros nodes
add_executable(line_detection_thresh src/line_detection_thresh.cpp)
add_executable(histogram_detection src/histogram_detection.cpp)
add_executable( flag_detection src/flag_detection.cpp )

## Add cmake target dependencies of the executable/library
add_dependencies(EvalUtils LineDetectionThresholding) 

## Specify libraries to link a library or executable target against
target_link_libraries(line_detection_thresh ${OpenCV_LIBS} EvalUtils LineDetectionThresholding)
target_link_libraries(histogram_detection ${OpenCV_LIBS} EvalUtils)
target_link_libraries( flag_detection ${OpenCV_LIBS} EvalUtils FlagDetection)

#http://stackoverflow.com/questions/23124655/cmake-opencv-cannot-specify-link-libraries




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS imgproc_node imgproc_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
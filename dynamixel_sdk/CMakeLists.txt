################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})

find_package(Boost 1.58 REQUIRED)
find_package(Eigen3 REQUIRED)
################################################################################
# Catkin specific configuration
################################################################################
catkin_package()

################################################################################
# Build
################################################################################
include_directories("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/" ${catkin_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/include/")
#MESSAGE(STATUS ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/${PROJECT_NAME})
#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

file(GLOB_RECURSE sources src/*.cpp)
if (CMAKE_SYSTEM_NAME MATCHES Windows)
file(GLOB to_remove src/port_handler_linux.cpp) 
elseif (CMAKE_SYSTEM_NAME MATCHES Linux) 
file(GLOB to_remove src/port_handler_windows.cpp)  
endif()
list(REMOVE_ITEM sources ${to_remove})

#add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(dynamixel_sdk src/myDXL_COM.cpp ${sources} src/odefun.c)
target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} )

cmake_minimum_required(VERSION 2.8.3)
project(controller)
find_package(catkin REQUIRED 
	COMPONENTS
	roscpp 
	std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

# Chai3D dependency
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
link_directories (${CHAI3D_LIBRARY_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})

# OpenGL dependency
find_package(OpenGL REQUIRED)

# GLUT dependency
find_package (GLUT REQUIRED)

#INCLUDE HEADERS
include_directories("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include")
include_directories("${CMAKE_SOURCE_DIR}/include/")

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

add_executable(controller src/controller.cpp src/kinodynamics.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} ${CHAI3D_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
set_target_properties(controller PROPERTIES COMPILE_FLAGS "-fPIC -march=native -std=c++0x")
target_compile_definitions(controller PUBLIC -DLINUX -D_LINUX)

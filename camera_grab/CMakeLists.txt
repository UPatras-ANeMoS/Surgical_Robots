cmake_minimum_required(VERSION 2.8.3)
project(cam_grab)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED 
	COMPONENTS
	cv_bridge
	image_transport
	image_geometry
	roscpp 
	std_msgs
	message_generation)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()
include_directories("${PROJECT_SOURCE_DIR}/include")


#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#message(STATUS ${OpenCV_VERSION_MAJOR})

find_package(PkgConfig REQUIRED)

pkg_check_modules(OPENCV REQUIRED opencv)
#MESSAGE(STATUS ${OPENCV_LIBRARY_DIRS})
#MESSAGE(STATUS ${OPENCV_LIBRARIES})
pkg_check_modules(ARUCO REQUIRED aruco)
MESSAGE(STATUS ${ARUCO_LIBRARY_DIRS})
MESSAGE(STATUS ${ARUCO_LIBRARIES})
include_directories(${OPENCV_INCLUDE_DIRS} ${ARUCO_INCLUDE_DIRS})

#find_package(Eigen3)
pkg_search_module(EIGEN3 REQUIRED eigen3)
#include_directories(${EIGEN3_INCLUDE_DIR})

#message(STATUS ${catkin_LIBRARIES})
#message(STATUS ${OpenCV_VERSION_MAJOR})
#message(STATUS ${catkin_INCLUDE_DIRS})

#find_package(aruco REQUIRED)
#set(CMAKE_MODULE_PATH "~/aruco-2.0.14/build/")
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(PAPARUCO aruco REQUIRED)
#find_package(aruco REQUIRED)
#include_directories(${aruco_INCLUDE_DIRS})
#message(STATUS ${aruco_INCLUDE_DIRS})
#message(STATUS ${aruco_LIBS})

find_package(OpenMP REQUIRED)
include_directories(${openmp_INCLUDE_DIRS})


#add_executable(cam_grab src/cam_grab.cpp)
#target_link_libraries(cam_grab ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(cam_sub src/cam_subtest.cpp)
#target_link_libraries(cam_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(cam_sub2 src/cam_subtest2.cpp)
#target_link_libraries(cam_sub2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PAPARUCO_LIBS})

#add_executable(cam_sub3 src/cam_subtest3.cpp)
#target_link_libraries(cam_sub3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS} ${openmp_LIBS})

#add_executable(cam_intrinsics src/cam_intrinsics.cpp)
#target_link_libraries(cam_intrinsics ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS} ${openmp_LIBS})
link_directories(${ARUCO_LIBRARY_DIRS})
LIST(REMOVE_ITEM OpenCV_INCLUDE_DIRS "aruco.hpp")
LIST(REMOVE_ITEM ARUCO_LIBRARIES "opencv_aruco3")
add_executable(single_cam src/single_camera.cpp)
#target_link_libraries(single_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS} ${openmp_LIBS})
target_link_libraries(single_cam ${catkin_LIBRARIES} ${ARUCO_LIBRARIES} ${openmp_LIBS})
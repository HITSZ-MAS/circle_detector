cmake_minimum_required(VERSION 3.0.2)
project(competition1)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Wstrict-null-sentinel -Wno-unused")
# set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
# -l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
# -lm -lc -lgcc_s -lgcc  
# -lstdc++fs -fmax-errors=10")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
)

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#eigen
include_directories( "/usr/include/eigen3" )

#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_library(mylibs
  src/ColorDetect.cpp
  src/RansacIter.cpp
)

set(LIBS
mylibs
${catkin_LIBRARIES}
${OpenCV_LIBS}
${Sophus_LIBRARIES}
)

add_executable(colortest src/colortest.cpp)
target_link_libraries(colortest ${LIBS})

add_executable(depthtest src/depthtest.cpp)
target_link_libraries(depthtest ${LIBS})

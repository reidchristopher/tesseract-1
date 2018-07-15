cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ext)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS bullet3/src bullet3/Extras
  LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition
)

set(BUILD_PYBULLET, ON)
set(BUILD_PYBULLET_NUMPY, OFF)
set(USE_DOUBLE_PRECISION, ON)
add_subdirectory(bullet3)

SET(FCL_BUILD_TESTS OFF CACHE BOOL "Disable FCL tests")
add_subdirectory(fcl)

cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

## set definition, path, compileflags
add_definitions(-DLINUX -DLINUX64 -DPOSIX)
set(OPENVR "$ENV{HOME}/libraries/openvr")
add_definitions(-DOPENVR_PATH="${OPENVR}") ## for source code to get sample textures and shader files path
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

## find openvr
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    linux64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers ${OPENVR}/samples)

## common src from openvr sample
file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

## add include path
include_directories(
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ## (OpenGL specific)
  ${GLEW_INCLUDE_DIR} ## (OpenGL specific)
  ${VULKAN_INCLUDE_DIR} ## (Vulkan specific)
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf
)

set(CMAKE_BUILD_TYPE "Release")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_srvs
    tf
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vive_node
  ${SHARED_SRC_FILES}
  src/vive_node.cpp
  src/vive_ros.cpp
  src/vr_interface.cpp
)

target_link_libraries(vive_node
  ${catkin_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
)

install(TARGETS vive_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

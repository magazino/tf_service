cmake_minimum_required(VERSION 2.8.3)
project(simple_tf_buffer_server)

add_compile_options(-std=c++14 -Wall -Wpedantic -O2)
if (NOT CMAKE_BUILD_TYPE)
    message(WARNING "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
else()
    message(STATUS "build type: ${CMAKE_BUILD_TYPE}")
endif()

set(PACKAGE_DEPENDENCIES
  geometry_msgs
  roscpp
  roslib
  tf2_msgs
  tf2_ros
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

# pybind11 is header-only.
add_subdirectory(3rdparty/pybind11)


##############
## setup.py ##
##############

catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  CanTransform.srv
  LookupTransform.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  tf2_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_tf_buffer_client
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)


###########
## Build ##
###########

include_directories(
  include
  src
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
  src/buffer_client.cc
  src/buffer_server.cc
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

# Python bindings
# TODO chooses Python version automatically?
pybind11_add_module(client src/python_bindings/buffer_client_py.cc)
target_include_directories(client SYSTEM PRIVATE ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(client PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# server executable
add_executable(server src/server_main.cc)
add_dependencies(server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(server SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(client_test test/client_test_main.cc)
add_dependencies(client_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(client_test SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(client_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TODO: remove after prototyping
install(TARGETS client_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS client
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# TODO install to different location than BIN
install(PROGRAMS
  test/client_test_main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TODO install to different location than SHARE
install(FILES
  test/benchmark_py.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## rostests with gtest executables
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(client_rostest test/client_rostest.launch test/client_rostest.cc)
  target_link_libraries(client_rostest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

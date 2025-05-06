cmake_minimum_required(VERSION 3.0.2)
project(tf_service)

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
find_package(pybind11)


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
  LIBRARIES ${PROJECT_NAME}
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
pybind11_add_module(client_binding src/python_bindings/buffer_client_py.cc)
target_include_directories(client_binding SYSTEM PRIVATE ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(client_binding PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
# See: http://wiki.ros.org/catkin/CMakeLists.txt#Custom_output_directory
set_target_properties(client_binding PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
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


#############
## Install ##
#############

install(TARGETS server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS client_binding
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
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

## rostests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  # Python
  install(PROGRAMS
    test/client_rostest.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  add_rostest(test/client_rostest_py.launch DEPENDENCIES ${PROJECT_NAME} server client_binding)
  
  # C++
  add_rostest_gtest(client_rostest test/client_rostest.launch test/client_rostest.cc)
  target_link_libraries(client_rostest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


################
## Benchmarks ##
################

# TODO: remove after prototyping
set(INCLUDE_BENCHMARKS YES)

if (INCLUDE_BENCHMARKS)
  # Python
  install(PROGRAMS
    test/benchmark/client_test_main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  # C++
  add_executable(client_test test/benchmark/client_test_main.cc)
  add_dependencies(client_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(client_test SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  target_link_libraries(client_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  install(TARGETS client_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  # launch
  install(FILES
    test/benchmark/benchmark_py.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

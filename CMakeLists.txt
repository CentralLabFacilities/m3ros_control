###############################################################################
# States that CMake required version must be greater than 2.8
cmake_minimum_required(VERSION 2.8)

###############################################################################
# Project name is not mandatory, but you should use it
project(m3ros_control)

###############################################################################
# Set the PROJECT_ROOT variable
set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

###############################################################################
# Set where to find the FindXXX.cmake
set(CMAKE_MODULE_PATH "${PROJECT_ROOT}/cmake/Modules/" $ENV{M3_CMAKE_MODULES} ${CMAKE_MODULE_PATH})

###############################################################################
# Set all bunch of usefull variables
set(PROJECT_INCLUDE_DIR ${PROJECT_ROOT}/include)
set(PROJECT_PROTO_DIR ${PROJECT_ROOT}/proto)
set(PROJECT_SRC_DIR ${PROJECT_ROOT}/src)
set(PROJECT_INSTALL_BIN ${PROJECT_ROOT}/bin)
set(PROJECT_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT}/bin)
set(PROJECT_INSTALL_LIB ${PROJECT_ROOT}/lib)
set(PROJECT_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_ROOT}/lib)
set(PROJECT_PYTHON_DIR ${PROJECT_ROOT}/python)

###############################################################################
# Forcing the generation dirs to be local
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})

find_package(catkin QUIET)
if(catkin_FOUND)
#  catkin_python_setup()
endif(catkin_FOUND)

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  controller_interface 
  control_toolbox
  realtime_tools
  m3meka_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf
)

set(LIBRARIES ${LIBRARIES} ${catkin_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


###############################################################################
# Find RTAI
find_package(RTAI REQUIRED)
set(LIBRARIES ${LIBRARIES} ${RTAI_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${RTAI_INCLUDE_DIR})

###############################################################################
# Find M3
find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3_INCLUDE_DIR})

###############################################################################
# Find M3RT
find_package(M3rt REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3RT_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR})


###############################################################################
# Find google protobuff
find_package(Protobuf REQUIRED)
set(PROTOBUF_IMPORT_DIRS ${INCLUDE_DIRS})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})

###############################################################################
# Find Eigen
find_package(Eigen3 3.2 REQUIRED)
set(LIBRARIES ${LIBRARIES} ${EIGEN3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Find boost packages
find_package(Boost 1.34 COMPONENTS filesystem system serialization REQUIRED)
set(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${PROJECT_INCLUDE_DIR})


###############################################################################
# Find Yamlcpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
if(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
find_package(YamlCpp  REQUIRED)
set_property(
   DIRECTORY
   PROPERTY COMPILE_DEFINITIONS YAMLCPP_05
   )
endif(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
link_directories(${YAMLCPP_LIBRARY_DIRS})
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
   INCLUDE_DIRS include
#   LIBRARIES ros_control_rtai
   CATKIN_DEPENDS roscpp
  controller_manager
  hardware_interface
  controller_interface 
  control_toolbox
  realtime_tools
  m3core
  m3meka
  m3meka_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf
#   #DEPENDS boost ...
)


###############################################################################
# Protobuff stuff
#file(GLOB ControllerProtoFiles "${PROJECT_PROTO_DIR}/${PROJECT_NAME}/*.proto")
#PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR}/src/m3ros_control ${ControllerProtoFiles})
#add_custom_target(${PROJECT_NAME}_cpp_protobuf_output  ALL DEPENDS ${ProtoSources}) # force the generation
#PROTOBUF_GENERATE_PYTHON(PySources ${PROJECT_PYTHON_DIR}/${PROJECT_NAME} ${ControllerProtoFiles})
#add_custom_target(${PROJECT_NAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
#execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}) # To create a directory at build time

add_subdirectory(src/m3ros_control)
###############################################################################
# Include directories
include_directories(
    ${PROJECT_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/src # Include this directory for the generated protobufs headers
    ${INCLUDE_DIRS}
    ${ProtoHeaders}
)


## Declare a cpp executable
add_library(${PROJECT_NAME} src/m3ros_control/ros_control_component.cpp
                            src/m3ros_control/meka_robot_hw.cpp
                            src/m3ros_control/omnibase_ctrl.cpp
                            src/m3ros_control/factory_proxy.cpp
 			    ${ProtoSources}
                            )
add_dependencies(${PROJECT_NAME} m3meka_msgs_gencpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} )

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


###############################################################################
# Generate the setup.bash
configure_file(setup.bash.in ${PROJECT_ROOT}/setup.bash)
# Generate the robot config file (the lib path is hardcoded) 
configure_file(${PROJECT_ROOT}/robot_config/m3_config.yml.in ${PROJECT_ROOT}/robot_config/m3_config.yml)

install(DIRECTORY launch robot_config config
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/state_manager.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

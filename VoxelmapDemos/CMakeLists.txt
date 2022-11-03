cmake_minimum_required(VERSION 3.21.0)
project(VoxelmapDemos)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
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
        INCLUDE_DIRS
            LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
            ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
########################################################################################################
#add_executable(simple_demo
#        demo/simple_demo_node.cpp
#        )
########################################################################################################

## Specify libraries to link a library or executable target against
########################################################################################################
#target_link_libraries(
#        simple_demo
#        ${catkin_LIBRARIES}
#)
########################################################################################################

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS
        ########################################################################################################
        #        simple_demo
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
        DIRECTORY data launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

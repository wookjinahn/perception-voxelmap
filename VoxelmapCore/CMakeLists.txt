cmake_minimum_required(VERSION 3.21.0)
project(VoxelmapCore)     # package name, same as in package.xml

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED    # listed in package.xml
        roscpp
)
find_package(camel-euclid-vector REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(                 # Specify build export information
        INCLUDE_DIRS            # Derectories with header files
            include
        LIBRARIES               # Libraries created in this project
            ${PROJECT_NAME}
        CATKIN_DEPENDS          # Packages dependent projects also need
            roscpp
        DEPENDS                 # System dependencies dependent projects also need -> listed in package.xml
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(            # Specify locations of header files
        include
            SYSTEM
            ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/BoundingBox.cpp
        src/Voxelmap.cpp
        src/Point2.cpp
        src/Point3.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

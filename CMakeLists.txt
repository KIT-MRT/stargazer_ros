cmake_minimum_required(VERSION 2.8.12)
project(stargazer_ros_tool)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

###################
## find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtAutoTarget)

include(GatherDeps)
# Remove libs which cannot be found automatically
#list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

################################################
## Declare ROS messages, services and actions ##
################################################

# Add message, service and action files
glob_ros_files(add_message_files msg)
glob_ros_files(add_service_files srv)
glob_ros_files(add_action_files action)

# Generate added messages and services with any dependencies listed here
if (ROS_GENERATE_MESSAGES)
    generate_messages(
        DEPENDENCIES
        std_msgs
        )
endif()

# Generate dynamic reconfigure options
file(GLOB CFG_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "cfg/*cfg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "srv/*.srv")
if (CFG_FILES)
  generate_dynamic_reconfigure_options(${CFG_FILES})
endif ()

###################################
## catkin specific configuration ##
###################################
catkin_package(
    )

###########
## Build ##
###########
# Add include and library directories
include_directories(
    ${mrt_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

link_directories(
    ${mrt_LIBRARY_DIRS}
    )

glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
if (SRC_DIRECTORIES)
    # Found subfolders, add executable for each subfolder
    foreach(SRC_DIR ${SRC_DIRECTORIES})
        mrt_add_node_and_nodelet(${SRC_DIR} FOLDER "src/${SRC_DIR}")
    endforeach()
else()
    # No subfolder found, add executable for src folder
    mrt_add_node_and_nodelet(${PROJECT_NAME} FOLDER "src")
endif()

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files)
mrt_install(PROGRAMS scripts FILES launch rviz maps res data nodelet_plugins.xml plugin_description.xml)

#############
## Testing ##
#############
# Add test targets for cpp tests
if (CATKIN_ENABLE_TESTING)
    include_directories("src")
    mrt_add_ros_tests(test)
endif()

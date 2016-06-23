#pkg_version=1.3
cmake_minimum_required(VERSION 2.8.3)
project(stargazer_ros)

###################
## find packages ##
###################
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs std_msgs
        cmake_modules dynamic_reconfigure message_generation stargazer rosbag nodelet tf)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
## include catkin
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

# Generate dynamic reconfigure options
file(GLOB CFG_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "cfg/*.cfg")
if (CFG_FILES)
    generate_dynamic_reconfigure_options(${CFG_FILES})
endif ()

# Generate messages
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}/msg" "*.msg")
if (MSG_FILES)
    add_message_files(
            DIRECTORY msg
            FILES ${MSG_FILES}
    )
    generate_messages(DEPENDENCIES std_msgs)
endif ()


###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
#compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" Cpp14CompilerFlag)
if (${Cpp14CompilerFlag})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "17")
    #no additional flag is required
else ()
    message(FATAL_ERROR "Compiler does not have c++14 support. Use at least g++4.9 or Visual Studio 2013 and newer.")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g")

# Add include and library directories
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
)

link_directories(
        ${catkin_INCLUDE_DIRS}
)

function(add_exec EXEC_NAME SEARCH_FOLDER)
    #glob all files in this directory
    file(GLOB EXEC_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.h" "${SEARCH_FOLDER}/*.hpp")
    file(GLOB EXEC_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.cpp")
    if (EXEC_SOURCE_FILES_SRC)
        #add executable
        add_executable(${EXEC_NAME}
                ${EXEC_SOURCE_FILES_INC}
                ${EXEC_SOURCE_FILES_SRC}
                )
        # Add cmake target dependencies of the executable/library
        # as an example, message headers may need to be generated before nodes
        add_dependencies(${EXEC_NAME} ${catkin_EXPORTED_TARGETS})
        if (CFG_FILES)
            add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_gencfg)
        endif ()
        if (MSG_FILES)
            add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_generate_messages)
        endif ()
        # Specify libraries to link a library or executable target against
        target_link_libraries(${EXEC_NAME}
                ${catkin_LIBRARIES}
                stargazer
                )
        # Mark executables and/or libraries for installation
        install(TARGETS ${EXEC_NAME}
                ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                )
    endif ()
endfunction()

function(add_nodelet EXEC_NAME SEARCH_FOLDER)
    #glob all files in this directory
    file(GLOB EXEC_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.h" "${SEARCH_FOLDER}/*.hpp")
    file(GLOB EXEC_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.cpp")
    # Find nodelet
    file(GLOB NODELET_CPP RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*_nodelet.cpp")
    if (NODELET_CPP)
        STRING(REGEX REPLACE "_node" "" NODELET_NAME ${EXEC_NAME})
        add_library(${NODELET_NAME}_nodelet
                ${EXEC_SOURCE_FILES_INC}
                ${EXEC_SOURCE_FILES_SRC}
                )
        target_link_libraries(${NODELET_NAME}_nodelet
                ${catkin_LIBRARIES}
                stargazer
                )
        install(TARGETS ${NODELET_NAME}_nodelet
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                )
    endif ()
endfunction()
#search for subfolders in src
function(glob_folders DIRECTORY_LIST SEARCH_DIRECTORY)
    execute_process(COMMAND find . -mindepth 1 -type d WORKING_DIRECTORY "${SEARCH_DIRECTORY}" OUTPUT_VARIABLE DIRECTORIES)

    set(_DIRECTORY_LIST_ "")
    if (NOT DIRECTORIES STREQUAL "")
        string(REPLACE "\n" ";" DIRECTORIES ${DIRECTORIES})

        foreach(SRC_DIR ${DIRECTORIES})
            #remove trailing "./"
            string(SUBSTRING "${SRC_DIR}" 2 -1 EXEC_NAME)
            list(APPEND _DIRECTORY_LIST_ ${EXEC_NAME})
        endforeach()

        set(${DIRECTORY_LIST} ${_DIRECTORY_LIST_} PARENT_SCOPE)
    endif()
endfunction()


glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
if (SRC_DIRECTORIES)
    #found subfolders, add executable for each subfolder
    foreach (SRC_DIR ${SRC_DIRECTORIES})
        add_exec(${SRC_DIR} "src/${SRC_DIR}")
        add_nodelet(${SRC_DIR} "src/${SRC_DIR}")
    endforeach ()
else ()
    #no subfolder found, add executable for src folder
    add_exec(${PROJECT_NAME} "src")
    add_nodelet(${PROJECT_NAME} "src")
endif ()

#############
## Install ##
#############
# Find and install nodelet plugin description file
file(GLOB PLUGINS_FILE RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "nodelet_plugins.xml")
if (PLUGINS_FILE)
    install(FILES nodelet_plugins.xml
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif ()

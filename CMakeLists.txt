cmake_minimum_required(VERSION 2.8.3)
project(pivot_control_messages_ros)
find_package(catkin REQUIRED
        roscpp
        std_msgs
        std_srvs
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        message_generation
        )

add_message_files(
        FILES
        LaparoscopeDOFPose.msg
        LaparoscopeDOFBoundaries.msg
        PivotError.msg
)

add_service_files(
        FILES
        SetInt.srv
        GetInt.srv
        SetPose.srv
)
generate_messages(
        DEPENDENCIES
        std_msgs
)
catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES esvr2_ros
        CATKIN_DEPENDS roscpp message_runtime
        #  DEPENDS system_lib
)

if (TARGET pivot_control_messages)
    message("found pivot_control_messages")
else()
    add_subdirectory(pivot_control_messages)
endif()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${franka_pivot_control_INCLUDE_DIR}
        pivot_control_messages/include
)

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

add_recursive(src SOURCES)

add_library(${PROJECT_NAME} SHARED
        ${SOURCES})

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_generate_messages_cpp
        )

target_link_libraries( ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${franka_pivot_control_LIBRARIES}
        pivot_control_messages)

catkin_install_python(PROGRAMS scripts/keyboardController.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
catkin_install_python(PROGRAMS scripts/ControlUserStudy.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
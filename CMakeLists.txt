cmake_minimum_required(VERSION 3.0)
project(diablo_sdk)

add_compile_options(
		-std=c++17
		-Wno-psabi
)


set(WITH_ROS    1)
set(library_name ${PROJECT_NAME}_core)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
add_definitions(-D_LOG_CONSOLE)

FILE(GLOB OSDK_LIB_SRCS
        diablo_utils/src/*.cpp
    )
    
add_library(
    ${library_name}
    SHARED
    ${OSDK_LIB_SRCS}
)


include_directories(
    include
)

if (WITH_ROS)
    add_definitions(-D_WITH_ROS)

    find_package(
        catkin REQUIRED
        message_generation
        roscpp
        rospy
        std_msgs
        geometry_msgs
        #roslib
    )

    add_message_files(
        FILES
        OSDK_ACCL.msg
        OSDK_GYRO.msg
        OSDK_LEGMOTORS.msg
        OSDK_POWER.msg
        OSDK_QUATERNION.msg
        OSDK_STATUS.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
    )

    catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES
        CATKIN_DEPENDS
        message_runtime
		# DEPENDS system_lib
    )

    include_directories(
        ${catkin_INCLUDE_DIRS}
    )

    link_libraries(
        ${catkin_LIBRARIES}
    )

	add_executable(movement_ctrl_example ${OSDK_LIB_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/example/movement_ctrl/main.cpp)
    add_dependencies(movement_ctrl_example diablo_sdk_generate_messages_cpp)
	target_link_libraries(movement_ctrl_example PRIVATE Threads::Threads)
    
	add_executable(robot_status_example ${OSDK_LIB_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/example/robot_status/main.cpp)
    add_dependencies(robot_status_example diablo_sdk_generate_messages_cpp)
	target_link_libraries(robot_status_example PRIVATE Threads::Threads)

    add_executable(status_update_and_ctrl_example ${OSDK_LIB_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/example/status_update_and_ctrl/main.cpp)
    add_dependencies(status_update_and_ctrl_example diablo_sdk_generate_messages_cpp)
	target_link_libraries(status_update_and_ctrl_example PRIVATE Threads::Threads)
    
else()
	add_executable(diablo_source main.cpp)
	install(TARGETS diablo_source RUNTIME DESTINATION bin)
	target_link_libraries(diablo_source PRIVATE Threads::Threads ${library_name})

endif (WITH_ROS)

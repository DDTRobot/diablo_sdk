cmake_minimum_required(VERSION 2.8)
project(osdk_status_update_and_ctrl_example)

# add_compile_options(-std=c++17)

FILE(GLOB SOURCE_FILES *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../api/src/*.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

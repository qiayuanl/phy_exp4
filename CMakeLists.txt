cmake_minimum_required(VERSION 3.6)
project(phy_exp4)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    message_generation
    dynamic_reconfigure
    std_msgs
    geometry_msgs
    image_transport
    cv_bridge
    )

find_package(OpenCV REQUIRED)

add_message_files(
    FILES
    PosSpeedStamp.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/process.cfg
    cfg/detection.cfg
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include/
    ./
)

FILE(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(phy_exp4 ${SOURCE_FILES})
add_dependencies(phy_exp4 ${PROJECT_NAME}_gencfg)
target_link_libraries(phy_exp4 ${OpenCV_LIBS} ${catkin_LIBRARIES})

# 编译时自动打开MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#-ofast
#set(CMAKE_CXX_FLAGS "-Ofast ${CMAKE_CXX_FLAGS}")
#set(CMAKE_C_FLAGS "-Ofast ${CMAKE_C_FLAGS}")

#SET(CMAKE_BUILD_TYPE "debug")
#SET(CMAKE_BUILD_TYPE "Release")

#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


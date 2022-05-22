cmake_minimum_required(VERSION 2.8.3)
project(pekf_slam)

find_package(PkgConfig)


find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf2
            tf
            nav_msgs
            std_msgs
            geometry_msgs
            sensor_msgs
            message_generation
            pcl_conversions
            pcl_ros
        )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(pekf_slam 
                       src/pekf_slam.cpp 
                       src/pekf_slam_node.cpp)
target_link_libraries(pekf_slam
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
add_dependencies(pekf_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



cmake_minimum_required(VERSION 2.8.12)

project(qt_ros)

find_package(Qt5Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
                        roscpp
                        geometry_msgs )

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(qt_ros_INCLUDE_DIRECTORIES
        src
        include
        resources )

include_directories(${qt_ros_INCLUDE_DIRECTORIES}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS} )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs
    DEPENDS Qt5Widgets )

file(GLOB qt_ros_SRCS src/*.cpp)
list(REMOVE_ITEM qt_ros_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )
file(GLOB qt_ros_HDRS include/*.h)
file(GLOB qt_ros_UIS resources/*.ui)
file(GLOB qt_ros_QRSCS resources/qrc/*.qrc)

qt5_add_resources(qt_ros_VFS ${qt_ros_QRSCS}) #VFS = Virtual File System
qt5_wrap_cpp(qt_ros_MOCS ${qt_ros_HDRS})
qt5_wrap_ui(qt_ros_UI_HDRS ${qt_ros_UIS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(${Qt5Widgets_DEFINITIONS}
                ${catkin_DEFINITIONS} )

add_library(_qt_ros
            ${qt_ros_SRCS}
            ${qt_ros_MOCS}
            ${qt_ros_UI_HDRS}
            ${qt_ros_VFS} )

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
                        _qt_ros
                        Qt5::Widgets
                        ${catkin_LIBRARIES} )

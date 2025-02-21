cmake_minimum_required(VERSION 3.8)
project(desktop_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

#FILE(GLOB HEAD_FILES "include/*.h")
#FILE(GLOB SRC_FILES "src/*.cpp")
#FILE(GLOB UI_FILES "src/*.ui")

include_directories(
    #include/ahrs_driver.h
    #include/crc_table.h
    #include/fdilink_data_struct.h
    include
    src
)

add_executable(qt_c_node
            src/main.cpp
            src/ros_joy.cpp
            src/mainwindow.cpp
            src/mainwindow.ui
            src/mainwindow.h)
ament_target_dependencies(qt_c_node rclcpp std_msgs sensor_msgs px4_msgs)

target_link_libraries(qt_c_node Qt5::Widgets)

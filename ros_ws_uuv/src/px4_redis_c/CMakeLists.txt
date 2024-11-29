cmake_minimum_required(VERSION 3.8)
project(px4_redis_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#set(px4_msgs_DIR ${PROJECT_SOURCE_DIR}/../../px4_msgs)
#set(px4_msgs_DIR /home/orangepi/ros_ws_uuv/src/px4_msgs)
#add_subdirectory(${px4_msgs_DIR})
find_package(px4_msgs REQUIRED)

set(CMAKE_MODULE_PATH /usr/lib/aarch64-linux-gnu/cmake/Hiredis ${CMAKE_MODULE_PATH})

#find_package(hiredis REQUIRED)


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

include_directories(/usr/local/include/hiredis)
link_directories(/usr/local/lib)

add_executable(px4_redis_write_c src/px4_redis_write_c.cpp)
ament_target_dependencies(px4_redis_write_c rclcpp px4_msgs)

target_link_libraries(px4_redis_write_c hiredis)

install(TARGETS
  px4_redis_write_c
  DESTINATION lib/${PROJECT_NAME}
)

#设置编译规则
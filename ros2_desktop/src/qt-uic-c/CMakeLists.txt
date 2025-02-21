cmake_minimum_required(VERSION 3.8)
project(qt-uic-c)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
 
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)  #一定要添加这几行，不然编译时可能会提示找不到Qt的ui文件
set(CMAKE_AUTORCC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
#find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(px4_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt5 COMPONENTS Charts REQUIRED)


include_directories(
  ./include
  ${CMAKE_CURRENT_BINARY_DIR}/qt_c_node_autogen/include  # 关键调整！
)

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

FILE(GLOB HEAD_FILES "./include/*.h" "./include/matrix/*.h")
FILE(GLOB SRC_FILES "./src/*.cpp")
FILE(GLOB UI_FILES "./src/*.ui")

add_executable(qt_c_node ${SRC_FILES} ${HEAD_FILES} ${UI_FILES})
ament_target_dependencies(qt_c_node rclcpp std_msgs sensor_msgs px4_msgs)

target_link_libraries(qt_c_node Qt5::Widgets Qt5::Charts)

install(TARGETS
  qt_c_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  ./include
  ${CMAKE_CURRENT_BINARY_DIR}/qt_c_node_autogen/include  # 关键！匹配生成路径
)
#include_directories(/opt/ros/humble/include)
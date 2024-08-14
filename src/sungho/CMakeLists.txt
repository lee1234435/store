cmake_minimum_required(VERSION 3.8)
project(sungho)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(UI_FILES
    ${UI_DIR}/main_window.ui
    ${UI_DIR}/teleop_window.ui
)

set(HEADER_FILES
    include/storagy/main_window.hpp
    include/storagy/teleop_window.hpp
)

set(SOURCE_FILES
    src/main.cpp
    src/image_sub_node.cpp
    src/battery_sub_node.cpp
    src/robot_control_node.cpp
    src/main_window.cpp
    src/teleop_window.cpp
)

qt5_wrap_ui(UIC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_executable(main_window
  src/main.cpp
  src/image_sub_node.cpp
  src/battery_sub_node.cpp
  src/robot_control_node.cpp
  src/main_window.cpp
  src/teleop_window.cpp
  ${UIC_FILES}
  ${MOC_FILES}
)

set_property(TARGET main_window PROPERTY AUTOUIC_SEARCH_PATHS 
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
)

target_include_directories(main_window PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Use ament_target_dependencies before target_link_libraries
ament_target_dependencies(main_window
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
)

# Change this to use the "plain" signature
target_link_libraries(main_window
  Qt5::Widgets
  ${OpenCV_LIBS}
)

install(TARGETS
  main_window
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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

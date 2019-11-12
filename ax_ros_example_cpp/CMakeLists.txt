cmake_minimum_required(VERSION 3.5)
project(ax_ros_example_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ax_ros_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    rcl_interfaces
    ax_ros_interfaces
    std_msgs
)


#include_directories(include)

#set(library_name ax_ros_example_cpp)

add_executable(simple_ax_server_cpp
  src/simple_ax_server.cpp
)

ament_target_dependencies(simple_ax_server_cpp
  ${dependencies}
)

#install(DIRECTORY include/
#  DESTINATION include/
#)

install(TARGETS simple_ax_server_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
endif()

#ament_export_include_directories(include)
#ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()

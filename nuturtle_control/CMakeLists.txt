cmake_minimum_required(VERSION 3.20)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OdomConfig.srv"
  "srv/TurtCircle.srv"
)


find_package(catch_ros2 REQUIRED)

add_executable(turtle_control src/turtle_control.cpp)
target_include_directories(turtle_control PRIVATE include)

target_link_libraries(
  turtle_control PRIVATE
  turtlelib::turtlelib
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

install(TARGETS
  turtle_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

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

  include(CTest)

  add_executable(turtle_control_test src/turtle_control_test.cpp)

  target_link_libraries(turtle_control_test
    catch_ros2::catch_ros2_with_node_main
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${nuturtlebot_msgs_TARGETS}
  )

  install(FILES
    test/turtle_control_test.launch.xml
    DESTINATION share/${PROJECT_NAME}
  )

  catch_ros2_add_integration_test(turtle_control_testXML
    LAUNCH_FILE turtle_control_test.launch.xml
  )

endif()

ament_package()

cmake_minimum_required(VERSION 3.20)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)
find_package(turtlelib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(ekf src/ekf.cpp)

target_include_directories(ekf PUBLIC include)
target_link_libraries(ekf PUBLIC ${ARMADILLO_LIBRARIES})

add_executable(slammer src/slammer.cpp)

target_include_directories(slammer PRIVATE include)
target_link_libraries(
  slammer PRIVATE
  rclcpp::rclcpp
  turtlelib::turtlelib
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ####### begin_citation [24] #######
  ${ARMADILLO_LIBRARIES}
  ####### end_citation [24] #######
  ekf
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

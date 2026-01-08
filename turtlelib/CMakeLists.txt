cmake_minimum_required(VERSION 3.22)
project(turtlelib LANGUAGES CXX)

add_library(turtlelib
  src/geometry2d.cpp
  src/angle.cpp
)

target_include_directories(turtlelib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(turtlelib PUBLIC cxx_std_23)
target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

# Install headers: results in <prefix>/include/turtlelib/...
install(DIRECTORY include/turtlelib DESTINATION include)

# Install the library + export target
install(TARGETS turtlelib
  EXPORT ${PROJECT_NAME}-targets
)

install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-config.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)
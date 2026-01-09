####### begin_citation [1] #######

cmake_minimum_required(VERSION 3.22)
project(turtlelib LANGUAGES CXX)

add_library(turtlelib
  src/geometry2d.cpp
  src/converter.cpp
)

add_executable(converter src/converter.cpp)
target_link_libraries(converter PRIVATE turtlelib)

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

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

####### end_citation [1] #######
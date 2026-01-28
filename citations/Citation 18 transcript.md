To include `nuturtlebot_msgs` as a dependency, add this line to your `find_package()` section (it's already there, so no changes needed):

```cmake
find_package(nuturtlebot_msgs REQUIRED)
```

Then link it to your `turtle_control` executable. Update the `target_link_libraries()` for `turtle_control`:

```cmake
add_executable(turtle_control src/turtle_control.cpp)
target_include_directories(turtle_control PRIVATE include)
target_link_libraries(
  turtle_control PRIVATE
  rclcpp::rclcpp
  ${nuturtlebot_msgs_TARGETS}
)
```

Also add it to your `package.xml` if you haven't already:

```xml
<depend>nuturtlebot_msgs</depend>
```

Note: `nuturtlebot_msgs` is already in your CMakeLists.txt but only linked to `nusimulator`. If you need it in `turtle_control` as well, add the target_link_libraries section above.
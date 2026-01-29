Prompt:
why does this file have an error that robotState isn't initialized, even though I do so later in the constructor?

Response:
The issue is that `robotState` is declared as a member variable in the header but has no default constructor available. In C++, member variables must be initialized before the constructor body executes.

Looking at your header:

```cpp
turtlelib::DiffDrive robotState;
```

When the constructor body starts, `robotState` needs to already be constructed. Since you're trying to assign it in the constructor body, you need to either:

1. **Use member initialization list** (preferred):

```cpp
turtle_control::turtle_control(): Node("turtle_control"),
    robotState(get_parameter("wheel_radius").as_double(), get_parameter("track_width").as_double())
{
}
```

2. **Or give `DiffDrive` a default constructor** in diff_drive.hpp:

```cpp
DiffDrive() = default;  // or DiffDrive(): wheel_radius(0.0), wheel_base(0.0), ... {}
```

The member initialization list approach is cleaner and more efficient since it constructs `robotState` directly with the correct values rather than default-constructing it first and then assigning.
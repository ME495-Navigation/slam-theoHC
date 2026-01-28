The issue is a **path mismatch**. Your header file is at:

```
nuturtle_control/include/nuturtle-control/turtle_control.hpp
```

(note the hyphen: `nuturtle-control`)

But your code is trying to include:

```cpp
#include <nuturtle_control/turtle_control.hpp>
```

(note the underscore: nuturtle_control)

**Fix**: Rename the directory from `nuturtle-control` to nuturtle_control:

```bash
mv nuturtle_control/include/nuturtle-control nuturtle_control/include/nuturtle_control
```

The include path should match the directory structure exactly.
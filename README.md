# BehaviorTree.PY

Minimal python binding to https://github.com/BehaviorTree/BehaviorTree.CPP

## Usage

By default, the library registers the built-in types. To register custom types, you need to create a shared library with the type converters and load it in python.

```cmake
add_library(bt_custom_types SHARED src/custom_types.cpp)
target_compile_features(bt_custom_types PRIVATE cxx_std_20)
target_compile_definitions(bt_custom_types PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(
  bt_custom_types PRIVATE behaviortree_py::behaviortree_py_lib
                          py_binding_tools::py_binding_tools)
install(
  TARGETS bt_custom_types
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_type_converters
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_type_converters
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_type_converters)
```

```cpp
#include <behaviortree_py/behaviortree_py.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// For the typecasters
#include <py_binding_tools/ros_msg_typecasters.h>

BT_REGISTER_TYPE_CONVERTER(geometry_msgs::msg::PoseStamped,
                           geometry_msgs::msg::Pose);
```

```python
import behaviortree_py as bt
...
bt.load_type_converters(
    get_package_share_directory("my_package")
    + "/bt_type_converters/libbt_custom_types.so"
)
...
```

## Limitations

- Only `register_simple_action`, `register_simple_condition`, and `register_simple_decorator` are supported.
- Due to limitations of `BT::Any` only copy-constructible types are supported.
- Due to how `pybind11` handle memory, mixing `std::shared_ptr` and `std::unique_ptr` between python/c++ is not supported.

## Acknowledgements

- A few functions were based on https://github.com/BehaviorTree/BehaviorTree.CPP/pull/634
- The TypeConverter is based on [moveit_task_constructor](https://github.com/moveit/moveit_task_constructor/)'s pybind11 implementation.

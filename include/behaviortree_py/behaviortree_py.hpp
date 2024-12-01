#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <fmt/compile.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

#include <rosidl_runtime_cpp/traits.hpp>

#include "behaviortree_cpp/utils/shared_library.h"

#if defined _WIN32 || defined __CYGWIN__
#define BTPY_EXPORT __declspec(dllexport)
#else
#define BTPY_EXPORT __attribute__((visibility("default")))
#endif

void log_info(const std::string& msg);
void log_error(const std::string& msg);

class TypeConverterRegistry {
 public:
  using to_python_converter_function = pybind11::object (*)(const BT::Any&);
  using from_python_converter_function = BT::Any (*)(const pybind11::object&);

  struct Entry {
    to_python_converter_function to_;
    from_python_converter_function from_;
  };

  // map from type_index to corresponding converter functions
  using RegistryMap = std::map<std::type_index, Entry>;
  RegistryMap types_;
  // map from ros-msg-names to entry in types_
  using RosMsgTypeNameMap = std::map<std::string, RegistryMap::iterator>;
  RosMsgTypeNameMap msg_names_;

  TypeConverterRegistry();

  static TypeConverterRegistry& get();

  BTPY_EXPORT bool insert(const std::type_index& type_index,
                          const std::string& ros_msg_name,
                          to_python_converter_function to,
                          from_python_converter_function from);
};

BTPY_EXPORT pybind11::object toPython(const BT::Any& value);

BTPY_EXPORT BT::Any fromPython(const pybind11::object& bpo);

// Get the name of all registered type converters
std::vector<std::string> get_registered_converters();

/// Utility class to register C++ / Python converters for a BT::Any of type T
template <typename T>
class TypeConverter {
 public:
  explicit TypeConverter(TypeConverterRegistry& registery) {
    registery.insert(typeid(T), rosMsgName<T>(), &toPython, &fromPython);
  }

 private:
  static pybind11::object toPython(const BT::Any& value) { return pybind11::cast(value.cast<T>()); }

  static BT::Any fromPython(const pybind11::object& po) { return BT::Any(pybind11::cast<T>(po)); }

  template <class Q = T>
  typename std::enable_if<rosidl_generator_traits::is_message<Q>::value, std::string>::type rosMsgName() {
    return rosidl_generator_traits::name<Q>();
  }

  template <class Q = T>
  typename std::enable_if<!rosidl_generator_traits::is_message<Q>::value, std::string>::type rosMsgName() {
    return std::string();
  }
};

namespace BTPY {
constexpr const char* PLUGIN_SYMBOL = "BTPY_RegisterTypeConverterFromPlugin";
}

namespace BT {
// Add a conversion specialization from string values into general py::objects
// by evaluating as a Python expression.
template <>
inline pybind11::object convertFromString(BT::StringView str) {
  try {
    // First, try evaluating the string as-is. Maybe it's a number, a list, a
    // dict, an object, etc.
    return pybind11::eval(str);
  } catch (pybind11::error_already_set& e) {
    // If that fails, then assume it's a string literal with quotation marks
    // omitted.
    return pybind11::str(str);
  }
}
}  // namespace BT

template <typename T>
void RegisterConverter(TypeConverterRegistry& registry) {
  TypeConverter<T>{registry};
}

template <typename First, typename Second, typename... Rest>
void RegisterConverter(TypeConverterRegistry& registry) {
  TypeConverter<First>{registry};
  RegisterConverter<Second, Rest...>(registry);
}

// Use this macro to automatically register one or more custom types.
// For instance:
//
//   BT_REGISTER_TYPE_CONVERTER(type1, type2, type3, ...);
//
// IMPORTANT: this function MUST be declared in a cpp file, NOT a header file.
// You must add the definition [BT_PLUGIN_EXPORT] in CMakeLists.txt using:
//
//   target_compile_definitions(my_plugin_target PRIVATE BT_PLUGIN_EXPORT)

#define BT_REGISTER_TYPE_CONVERTER(...)                                                     \
  BTCPP_EXPORT void BTPY_RegisterTypeConverterFromPlugin(TypeConverterRegistry& registry) { \
    RegisterConverter<__VA_ARGS__>(registry);                                               \
  }

void LoadTypeConverters(const std::string& file_path);

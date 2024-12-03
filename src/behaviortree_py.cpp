#include <behaviortree_cpp/actions/always_success_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/controls/fallback_node.h>
#include <behaviortree_cpp/controls/parallel_node.h>
#include <behaviortree_cpp/controls/sequence_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/decorators/keep_running_until_failure_node.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/tree_node.h>
#include <fmt/ranges.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_py/behaviortree_py.hpp>
#include <memory>
#include <utility>

namespace py = pybind11;

inline static TypeConverterRegistry registry_singleton;

TypeConverterRegistry& TypeConverterRegistry::get() { return registry_singleton; }

TypeConverterRegistry::TypeConverterRegistry() {
  TypeConverter<bool>{TypeConverterRegistry::get()};
  TypeConverter<int>{TypeConverterRegistry::get()};
  TypeConverter<float>{TypeConverterRegistry::get()};
  TypeConverter<double>{TypeConverterRegistry::get()};
  TypeConverter<std::string>{TypeConverterRegistry::get()};
}

bool TypeConverterRegistry::insert(const std::type_index& type_index,
                                   const std::string& ros_msg_name,
                                   to_python_converter_function to,
                                   from_python_converter_function from) {
  auto it_inserted = types_.insert(std::make_pair(type_index, Entry{to, from}));
  if (!it_inserted.second) return false;

  if (!ros_msg_name.empty()) {  // is this a ROS msg type?
    msg_names_.insert(std::make_pair(ros_msg_name, it_inserted.first));
  } else {
    msg_names_.insert(std::make_pair(BT::demangle(type_index), it_inserted.first));
  }

  return true;
}

py::object toPython(const BT::Any& value) {
  if (value.empty()) return py::object();

  auto it = TypeConverterRegistry::get().types_.find(value.type());
  if (it == TypeConverterRegistry::get().types_.end()) {
    std::string name = BT::demangle(value.type());
    throw py::type_error("No Python -> C++ conversion for: " + name);
  }

  return it->second.to_(value);
}

std::string rosMsgName(PyObject* object) {
  py::object o = py::reinterpret_borrow<py::object>(object);
  auto cls = o.attr("__class__");
  auto name = cls.attr("__name__").cast<std::string>();
  auto module = cls.attr("__module__").cast<std::string>();
  auto pos = module.find(".msg");
  if (pos == std::string::npos)
    // object is not a ROS message type, return it's class name instead
    return module + "." + name;
  else
    return module.substr(0, pos) + "/msg/" + name;
}

BT::Any fromPython(const py::object& po) {
  PyObject* o = po.ptr();

  if (PyBool_Check(o)) return BT::Any((o == Py_True));
  if (PyLong_Check(o)) return BT::Any(PyLong_AS_LONG(o));
  if (PyFloat_Check(o)) return BT::Any(PyFloat_AS_DOUBLE(o));
  if (PyUnicode_Check(o)) return BT::Any(py::cast<std::string>(o));

  const std::string& ros_msg_name = rosMsgName(o);
  auto it = TypeConverterRegistry::get().msg_names_.find(ros_msg_name);
  if (it == TypeConverterRegistry::get().msg_names_.end()) {
    try {
      // Get the Python type object
      PyTypeObject* type = po.ptr()->ob_type;

      // Check if it's a wrapped C++ type by checking for the type_info holder
      if (auto* type_info = pybind11::detail::get_type_info(type)) {
        const std::string cpptype = BT::demangle(*type_info->cpptype);
        it = TypeConverterRegistry::get().msg_names_.find(cpptype);
        if (it == TypeConverterRegistry::get().msg_names_.end()) {
          throw py::type_error("No C++ conversion available for type: '" + cpptype +
                               "'\n Available conversions : " + fmt::format("{}", get_registered_converters()));
        }
      }
    } catch (const std::runtime_error& e) {
      throw py::type_error(e.what());
    }
  }

  return it->second->second.from_(po);
}

void LoadTypeConverters(const std::string& file_path) {
  try {
    BT::SharedLibrary loader(file_path);
    if (loader.hasSymbol(BTPY::PLUGIN_SYMBOL)) {
      typedef void (*Func)(TypeConverterRegistry&);
      auto func = (Func)loader.getSymbol(BTPY::PLUGIN_SYMBOL);
      func(TypeConverterRegistry::get());
    } else {
      log_error(fmt::format("Failed to load Plugin from file: {}", file_path));
      return;
    }
    log_info(fmt::format("Loaded ROS Plugin: {}", file_path));
  } catch (const std::exception& ex) {
    log_error(fmt::format("Failed to load ROS Plugin: {} \n {}", file_path, ex.what()));
  }
}

void log_warn(const std::string& msg) {
  printf(
      "["
      "\x1b[33m"
      "warn"
      "\x1b[0m"
      "]: %s",
      msg.c_str());
  std::cout << std::endl;
}

void log_info(const std::string& msg) {
  printf(
      "["
      "\x1b[32m"
      "info"
      "\x1b[0m"
      "]: %s",
      msg.c_str());
  std::cout << std::endl;
}

void log_error(const std::string& msg) {
  printf(
      "["
      "\x1b[31m"
      "error"
      "\x1b[0m"
      "]: %s",
      msg.c_str());
  std::cout << std::endl;
}

std::vector<std::string> get_registered_converters() {
  std::vector<std::string> result;
  for (const auto& entry : TypeConverterRegistry::get().types_) {
    result.emplace_back(BT::demangle(entry.first));
  }
  return result;
}

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
#include <fmt/args.h>
#include <py_binding_tools/ros_msg_typecasters.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_py/behaviortree_py.hpp>
#include <memory>
#include <utility>

namespace py = pybind11;

// Mention that it's copied from MTC
namespace {

/** In order to assign new property values in Python, we need to convert the
 *Python object to a boost::any instance of the correct type. As the C++ type
 *cannot be inferred from the Python type, we can support this assignment only
 *for a few basic types (see fromPython()) as well as ROS message types. For
 *other types, a generic assignment via stage.properties["property"] = value is
 *not possible. Instead, use the .property<Type> declaration on the stage to
 *allow for direct assignment like this: stage.property = value
 **/
class PropertyConverterRegistry {
  using to_python_converter_function = pybind11::object (*)(const BT::Any&);
  using from_python_converter_function = BT::Any (*)(const pybind11::object&);

  struct Entry {
    to_python_converter_function to_;
    from_python_converter_function from_;
  };

  // map from type_index to corresponding converter functions
  typedef std::map<std::type_index, Entry> RegistryMap;
  RegistryMap types_;
  // map from ros-msg-names to entry in types_
  using RosMsgTypeNameMap = std::map<std::string, RegistryMap::iterator>;
  RosMsgTypeNameMap msg_names_;

 public:
  PropertyConverterRegistry();

  inline bool insert(const std::type_index& type_index,
                     const std::string& ros_msg_name,
                     to_python_converter_function to,
                     from_python_converter_function from);

  static py::object toPython(const BT::Any& value);

  static BT::Any fromPython(const py::object& bpo);
};

inline constexpr static PropertyConverterRegistry REGISTRY_SINGLETON;

/// utility class to register C++ / Python converters for a property of type T
template <typename T>
class PropertyConverter {
 public:
  PropertyConverter() { REGISTRY_SINGLETON.insert(typeid(T), rosMsgName<T>(), &toPython, &fromPython); }

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

PropertyConverterRegistry::PropertyConverterRegistry() {
  // register property converters
  PropertyConverter<bool>();
  PropertyConverter<int>();
  PropertyConverter<unsigned int>();
  PropertyConverter<long>();
  PropertyConverter<float>();
  PropertyConverter<double>();
  PropertyConverter<std::string>();
  PropertyConverter<std::set<std::string>>();
  PropertyConverter<std::map<std::string, double>>();
}

bool PropertyConverterRegistry::insert(const std::type_index& type_index,
                                       const std::string& ros_msg_name,
                                       to_python_converter_function to,
                                       from_python_converter_function from) {
  auto it_inserted = types_.insert(std::make_pair(type_index, Entry{to, from}));
  if (!it_inserted.second) return false;

  if (!ros_msg_name.empty())  // is this a ROS msg type?
    msg_names_.insert(std::make_pair(ros_msg_name, it_inserted.first));

  return true;
}

py::object PropertyConverterRegistry::toPython(const BT::Any& value) {
  if (value.empty()) return py::object();
  for (const auto& [name, entry] : REGISTRY_SINGLETON.msg_names_) {
    std::cout << name << " " << BT::demangle(entry->first) << std::endl;
  }

  auto it = REGISTRY_SINGLETON.types_.find(value.type());
  if (it == REGISTRY_SINGLETON.types_.end()) {
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

BT::Any PropertyConverterRegistry::fromPython(const py::object& po) {
  PyObject* o = po.ptr();

  if (PyBool_Check(o)) return BT::Any((o == Py_True));
  if (PyLong_Check(o)) return BT::Any(PyLong_AS_LONG(o));
  if (PyFloat_Check(o)) return BT::Any(PyFloat_AS_DOUBLE(o));
  if (PyUnicode_Check(o)) return BT::Any(py::cast<std::string>(o));

  const std::string& ros_msg_name = rosMsgName(o);
  auto it = REGISTRY_SINGLETON.msg_names_.find(ros_msg_name);
  if (it == REGISTRY_SINGLETON.msg_names_.end())
    throw py::type_error("No C++ conversion available for (property) type: " + ros_msg_name);

  return it->second->second.from_(po);
}

}  // end anonymous namespace

// WTF??
namespace BT {
template <>
inline BT::Any convertFromString<BT::Any>(BT::StringView str) {
  return BT::Any(str);
}
}  // namespace BT

PYBIND11_MODULE(behaviortree_py, m) {
  m.doc() = "Python wrapper for BehaviorTree.CPP";

  py::register_exception<nonstd::bad_expected_access<std::string>>(m, "BadExpectedAccess");

  py::enum_<BT::NodeStatus>(m, "NodeStatus")
      .value("SUCCESS", BT::NodeStatus::SUCCESS)
      .value("FAILURE", BT::NodeStatus::FAILURE)
      .value("RUNNING", BT::NodeStatus::RUNNING)
      .value("IDLE", BT::NodeStatus::IDLE)
      .export_values();

  py::class_<BT::Result>(m, "Result").def("__bool__", &BT::Result::operator bool).def("error", [](BT::Result* self) {
    return self->error();
  });

  py::class_<BT::Blackboard, std::shared_ptr<BT::Blackboard>>(m, "Blackboard")
      .def_static("create", &BT::Blackboard::create);

  py::class_<BT::Tree>(m, "Tree")
      .def("root_node", &BT::Tree::rootNode, py::return_value_policy::reference_internal)
      .def("tick_while_running", &BT::Tree::tickWhileRunning, py::arg("sleep_time") = std::chrono::milliseconds(10));

  py::class_<BT::TreeNode, std::shared_ptr<BT::TreeNode>>(m, "TreeNode")
      .def("name", &BT::TreeNode::name)
      .def("status", &BT::TreeNode::status)
      .def("type", &BT::TreeNode::type)
      .def("get_input",
           [](BT::TreeNode* self, const std::string& key) {
             return PropertyConverterRegistry::toPython(self->getInput<BT::Any>(key).value());
           })
      .def("set_output", [](BT::TreeNode* self, const std::string& key, py::object value) {
        return self->setOutput(key, PropertyConverterRegistry::fromPython(value));
      });

  py::class_<BT::BehaviorTreeFactory>(m, "BehaviorTreeFactory")
      .def(py::init<>())
      .def("create_tree_from_text",
           &BT::BehaviorTreeFactory::createTreeFromText,
           py::arg("xml_text"),
           py::arg("blackboard") = BT::Blackboard::create())
      .def("create_tree",
           &BT::BehaviorTreeFactory::createTree,
           py::arg("tree_name"),
           py::arg("blackboard") = BT::Blackboard::create())
      .def("register_behavior_tree_from_text",
           &BT::BehaviorTreeFactory::registerBehaviorTreeFromText,
           py::arg("xml_text"))
      .def("register_scripting_enum", &BT::BehaviorTreeFactory::registerScriptingEnum)
      .def(
          "register_simple_action",
          [](BT::BehaviorTreeFactory* self,
             const std::string& id,
             std::function<BT::NodeStatus(BT::TreeNode*)> tick_functor,
             BT::PortsList ports) {
            self->registerSimpleAction(
                id, [tick_functor](BT::TreeNode& tree_node) { return tick_functor(&tree_node); }, std::move(ports));
          },
          py::arg("id"),
          py::arg("tick_functor"),
          py::arg("ports") = BT::PortsList())
      .def(
          "register_simple_decorator",
          // We need to use a pointer cause pybind11 does copy for references
          // https://github.com/pybind/pybind11/issues/1123
          [](BT::BehaviorTreeFactory* self,
             const std::string& id,
             std::function<BT::NodeStatus(BT::NodeStatus, BT::TreeNode*)> tick_functor,
             BT::PortsList ports) {
            self->registerSimpleDecorator(
                id,
                [tick_functor](BT::NodeStatus node_status, BT::TreeNode& tree_node) {
                  return tick_functor(node_status, &tree_node);
                },
                std::move(ports));
          },
          py::arg("id"),
          py::arg("tick_functor"),
          py::arg("ports") = BT::PortsList())
      .def(
          "register_simple_condition",
          [](BT::BehaviorTreeFactory* self,
             const std::string& id,
             std::function<BT::NodeStatus(BT::TreeNode*)> tick_functor,
             BT::PortsList ports) {
            self->registerSimpleCondition(
                id, [tick_functor](BT::TreeNode& tree_node) { return tick_functor(&tree_node); }, std::move(ports));
          },
          py::arg("id"),
          py::arg("tick_functor"),
          py::arg("ports") = BT::PortsList());

  py::class_<BT::StdCoutLogger>(m, "StdCoutLogger").def(py::init<const BT::Tree&>(), py::arg("tree"));

  py::class_<BT::Groot2Publisher>(m, "Groot2Publisher")
      .def(py::init<const BT::Tree&, unsigned>(), py::arg("tree"), py::arg("port") = 1667);

  py::class_<BT::PortInfo> give_me_a_name(m, "PortInfo");

  m.def(
      "get_tree_recursively",
      [](const BT::TreeNode* root_node) {
        std::ostringstream s;
        BT::printTreeRecursively(root_node, s);
        return s.str();
      },
      py::arg("root_tree"));

  m.def(
       "input_port",
       [](BT::StringView name, BT::StringView description) { return BT::InputPort<BT::Any>(name, description); },
       py::arg("name"),
       py::arg("description") = "")
      .def(
          "output_port",
          [](BT::StringView name, BT::StringView description) { return BT::OutputPort<BT::Any>(name, description); },
          py::arg("name"),
          py::arg("description") = "");
}

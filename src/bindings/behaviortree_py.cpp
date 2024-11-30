#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_py/behaviortree_py.hpp>

namespace py = pybind11;

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
           [](BT::TreeNode* self, const std::string& key) { return toPython(self->getInput<BT::Any>(key).value()); })
      .def("set_output", [](BT::TreeNode* self, const std::string& key, py::object value) {
        return self->setOutput(key, fromPython(value));
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

  m.def("load_type_converters", &LoadTypeConverters, py::arg("file_path"));
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

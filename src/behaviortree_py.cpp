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
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_py/behaviortree_py.hpp>
#include <memory>
#include <utility>

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
      .def("type", &BT::TreeNode::type);

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

  bind_port_methods<int>(m, "int");
  bind_port_methods<double>(m, "double");
  bind_port_methods<std::string>(m, "string");
  bind_port_methods<bool>(m, "bool");
}

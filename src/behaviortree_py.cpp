#include <behaviortree_cpp/behavior_tree.h>
#include <memory>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_cpp/actions/always_success_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/controls/fallback_node.h>
#include <behaviortree_cpp/controls/parallel_node.h>
#include <behaviortree_cpp/controls/sequence_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/decorators/keep_running_until_failure_node.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/tree_node.h>

namespace py = pybind11;
using namespace BT;

PYBIND11_MODULE(behaviortree_py, m) {
  m.doc() = "Python wrapper for BehaviorTree.CPP";

  py::register_exception<nonstd::bad_expected_access<std::string>>(
      m, "BadExpectedAccess");

  py::enum_<NodeStatus>(m, "NodeStatus")
      .value("SUCCESS", NodeStatus::SUCCESS)
      .value("FAILURE", NodeStatus::FAILURE)
      .value("RUNNING", NodeStatus::RUNNING)
      .value("IDLE", NodeStatus::IDLE)
      .export_values();

  py::class_<BT::Result>(m, "Result")
      .def("__bool__", &BT::Result::operator bool)
      .def("error", [](BT::Result *self) { return self->error(); });

  py::class_<Blackboard, std::shared_ptr<Blackboard>>(m, "Blackboard")
      .def_static("create", &Blackboard::create);

  py::class_<Tree>(m, "Tree")
      .def("root_node", &Tree::rootNode,
           py::return_value_policy::reference_internal)
      .def("tick_while_running", &Tree::tickWhileRunning,
           py::arg("sleep_time") = std::chrono::milliseconds(10));

  py::class_<TreeNode, std::shared_ptr<TreeNode>>(m, "TreeNode")
      .def("name", &TreeNode::name)
      .def("status", &TreeNode::status)
      .def("type", &TreeNode::type)
      .def("get_input_string",
           [](TreeNode *self, const std::string &key) {
             return self->getInput<std::string>(key).value();
           })
      .def("get_input_int",
           [](TreeNode *self, const std::string &key) {
             return self->getInput<int>(key).value();
           })
      .def("get_input_double",
           [](TreeNode *self, const std::string &key) {
             return self->getInput<double>(key).value();
           })
      .def("set_output",
           py::overload_cast<const std::string &, const std::string &>(
               &TreeNode::setOutput<std::string>))
      .def("set_output", py::overload_cast<const std::string &, const int &>(
                             &TreeNode::setOutput<int>))
      .def("set_output", py::overload_cast<const std::string &, const double &>(
                             &TreeNode::setOutput<double>));

  py::class_<BehaviorTreeFactory>(m, "BehaviorTreeFactory")
      .def(py::init<>())
      .def("create_tree_from_text", &BehaviorTreeFactory::createTreeFromText,
           py::arg("xml_text"), py::arg("blackboard") = Blackboard::create())
      .def("create_tree", &BehaviorTreeFactory::createTree,
           py::arg("tree_name"), py::arg("blackboard") = Blackboard::create())
      .def("register_behavior_tree_from_text",
           &BehaviorTreeFactory::registerBehaviorTreeFromText,
           py::arg("xml_text"))
      .def("register_scripting_enum",
           &BehaviorTreeFactory::registerScriptingEnum)
      .def(
          "register_simple_action",
          [](BehaviorTreeFactory *self, const std::string &id,
             std::function<NodeStatus(TreeNode *)> tick_functor,
             PortsList ports) {
            self->registerSimpleAction(
                id,
                [tick_functor](TreeNode &tree_node) {
                  return tick_functor(&tree_node);
                },
                ports);
          },
          py::arg("id"), py::arg("tick_functor"),
          py::arg("ports") = PortsList())
      .def(
          "register_simple_decorator",
          // We need to use a pointer cause pybind11 does copy for references
          // https://github.com/pybind/pybind11/issues/1123
          [](BehaviorTreeFactory *self, const std::string &id,
             std::function<NodeStatus(NodeStatus, TreeNode *)> tick_functor,
             PortsList ports) {
            self->registerSimpleDecorator(
                id,
                [tick_functor](NodeStatus node_status, TreeNode &tree_node) {
                  return tick_functor(node_status, &tree_node);
                },
                ports);
          },
          py::arg("id"), py::arg("tick_functor"),
          py::arg("ports") = PortsList())
      .def(
          "register_simple_condition",
          [](BehaviorTreeFactory *self, const std::string &id,
             std::function<NodeStatus(TreeNode *)> tick_functor,
             PortsList ports) {
            self->registerSimpleCondition(
                id,
                [tick_functor](TreeNode &tree_node) {
                  return tick_functor(&tree_node);
                },
                ports);
          },
          py::arg("id"), py::arg("tick_functor"),
          py::arg("ports") = PortsList());

  py::class_<StdCoutLogger>(m, "StdCoutLogger")
      .def(py::init<const BT::Tree &>(), py::arg("tree"));

  py::class_<BT::Groot2Publisher>(m, "Groot2Publisher")
      .def(py::init<const BT::Tree &, unsigned>(), py::arg("tree"),
           py::arg("port") = 1667);

  py::class_<BT::PortInfo>(m, "PortInfo");

  m.def(
      "get_tree_recursively",
      [](const TreeNode *root_node) {
        std::ostringstream s;
        BT::printTreeRecursively(root_node, s);
        return s.str();
      },
      py::arg("root_tree"));

  m.def(
       "input_port_int",
       [](BT::StringView name, BT::StringView description) {
         return BT::InputPort<int>(name, description);
       },
       py::arg("name"), py::arg("description") = "")
      .def(
          "input_port_double",
          [](BT::StringView name, BT::StringView description) {
            return BT::InputPort<double>(name, description);
          },
          py::arg("name"), py::arg("description") = "")
      .def(
          "input_port_string",
          [](BT::StringView name, BT::StringView description) {
            return BT::InputPort<std::string>(name, description);
          },
          py::arg("name"), py::arg("description") = "")
      .def(
          "output_port_int",
          [](BT::StringView name, BT::StringView description) {
            return BT::OutputPort<int>(name, description);
          },
          py::arg("name"), py::arg("description") = "")
      .def(
          "output_port_double",
          [](BT::StringView name, BT::StringView description) {
            return BT::OutputPort<double>(name, description);
          },
          py::arg("name"), py::arg("description") = "")
      .def(
          "output_port_string",
          [](BT::StringView name, BT::StringView description) {
            return BT::OutputPort<std::string>(name, description);
          },
          py::arg("name"), py::arg("description") = "");
}

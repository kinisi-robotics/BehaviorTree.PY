#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <fmt/format.h>
#include <pybind11/pybind11.h>

#include <behaviortree_py/behaviortree_py.hpp>

struct ExampleTask {
  std::string name;
};

class SetTaskName : public BT::SyncActionNode {
 public:
  SetTaskName(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {BT::OutputPort<ExampleTask>("task")}; }

  BT::NodeStatus tick() override {
    auto task = ExampleTask();
    task.name = "My Task C++";
    setOutput("task", task);
    return BT::NodeStatus::SUCCESS;
  }
};

class PrintTaskName : public BT::SyncActionNode {
 public:
  PrintTaskName(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {BT::InputPort<ExampleTask>("task")}; }

  BT::NodeStatus tick() override {
    auto task_maybe = getInput<ExampleTask>("task");
    if (!task_maybe) {
      log_error("Missing required input [task]");
      return BT::NodeStatus::FAILURE;
    }
    log_info(fmt::format("Task name: {}", task_maybe.value().name));
    return BT::NodeStatus::SUCCESS;
  }
};

namespace py = pybind11;
PYBIND11_MODULE(example, m) {
  py::class_<ExampleTask>(m, "ExampleTask").def(py::init<>()).def_readwrite("name", &ExampleTask::name);
}

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<SetTaskName>("SetTaskName");
  factory.registerNodeType<PrintTaskName>("PrintTaskName");
}
BT_REGISTER_TYPE_CONVERTER(ExampleTask);

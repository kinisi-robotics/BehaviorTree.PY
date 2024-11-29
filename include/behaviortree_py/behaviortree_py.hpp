#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <fmt/compile.h>
#include <pybind11/pybind11.h>

template <typename T>
void bind_port_methods(pybind11::module &m, const char *type_suffix) {
  const std::string input_port =
      fmt::format(FMT_COMPILE("input_port_{}"), type_suffix);
  const std::string get_input =
      fmt::format(FMT_COMPILE("get_input_{}"), type_suffix);
  const std::string output_port =
      fmt::format(FMT_COMPILE("output_port_{}"), type_suffix);

  m.def(
      input_port.c_str(),
      [](const std::string &name, const std::string &description) {
        return BT::InputPort<T>(name, description);
      },
      pybind11::arg("name"), pybind11::arg("description") = "");
  m.def(
      output_port.c_str(),
      [](const std::string &name, const std::string &description) {
        return BT::OutputPort<T>(name, description);
      },
      pybind11::arg("name"), pybind11::arg("description") = "");
  m.def("set_output",
        [](BT::TreeNode &tree, const std::string &key, const T &value) {
          return tree.setOutput(key, value);
        });
  m.def(get_input.c_str(),
        [](const BT::TreeNode &tree, const std::string &key) {
          return tree.getInput<T>(key).value();
        });
}

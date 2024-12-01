import time

import behaviortree_py as bt
import behaviortree_py.example as bt_example  # Needed to laod the ExampleTask pybind11

xml_text = """
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <SetTaskName task="{task}" />
            <ModifyTaskName task="{task}" />
            <PrintTaskName task="{task}" />
            <PySetTaskName task="{task}" />
            <PrintTaskName task="{task}" />
        </Sequence>
    </BehaviorTree>
</root>
"""


def modify_task_name(tree_node: bt.TreeNode) -> bt.NodeStatus:
    task = tree_node.get_input("task")
    print(f"Modifying task name: {task.name}")
    task.name = "My Task Python"
    tree_node.set_output("task", task)
    return bt.NodeStatus.SUCCESS


def py_set_task_name(tree_node: bt.TreeNode) -> bt.NodeStatus:
    task = bt_example.ExampleTask()
    task.name = "New Task Name Python"
    tree_node.set_output("task", task)
    return bt.NodeStatus.SUCCESS


factory = bt.BehaviorTreeFactory()
factory.register_from_plugin(bt_example.__file__)
bt.load_type_converters(bt_example.__file__)
factory.register_simple_action(
    "ModifyTaskName", modify_task_name, dict([bt.bidirectional_port("task")])
)
factory.register_simple_action(
    "PySetTaskName", py_set_task_name, dict([bt.output_port("task")])
)
factory.register_behavior_tree_from_text(xml_text)

tree = factory.create_tree("MainTree")

logger = bt.StdCoutLogger(tree)

print(bt.get_tree_recursively(tree.root_node()))

tree.tick_while_running()

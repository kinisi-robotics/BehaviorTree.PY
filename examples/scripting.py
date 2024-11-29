import behaviortree_py as bt

xml_text = """
<root BTCPP_format="4">
    <BehaviorTree>
       <Sequence>
           <Script code=" msg:='hello world' " />
           <Script code=" A:=THE_ANSWER; B:=3.14; color:=RED " />
           <Precondition if="A>B && color != BLUE" else="FAILURE">
               <Sequence>
                 <SetValue value="{double_value}"/>
                 <SaySomething message="{A}"/>
                 <SaySomething message="{B}"/>
                 <SaySomething message="{msg}"/>
                 <SaySomething message="{color}"/>
                 <SaySomethingDouble message="{double_value}"/>
               </Sequence>
           </Precondition>
       </Sequence>
    </BehaviorTree>
</root>
"""


def say_something_double(
    tree_node: bt.TreeNode,
) -> bt.NodeStatus:
    print(bt.get_input_double(tree_node, "message"))
    return bt.NodeStatus.SUCCESS


def say_something(tree_node: bt.TreeNode) -> bt.NodeStatus:
    print(bt.get_input_string(tree_node, "message"))
    return bt.NodeStatus.SUCCESS


def set_value(tree_node: bt.TreeNode) -> bt.NodeStatus:
    if not (result := bt.set_output(tree_node, "value", 24.9)):
        return bt.NodeStatus.FAILURE
    return bt.NodeStatus.SUCCESS


factory = bt.BehaviorTreeFactory()
factory.register_simple_action(
    "SetValue", set_value, dict([bt.output_port_double("value")])
)
factory.register_simple_action(
    "SaySomething", say_something, dict([bt.input_port_string("message")])
)
factory.register_simple_action(
    "SaySomethingDouble", say_something_double, dict([bt.input_port_double("message")])
)
factory.register_scripting_enum("RED", 1)
factory.register_scripting_enum("BLUE", 2)
factory.register_scripting_enum("GREEN", 3)
factory.register_scripting_enum("THE_ANSWER", 42)
tree = factory.create_tree_from_text(xml_text)
logger = bt.StdCoutLogger(tree)

print(bt.get_tree_recursively(tree.root_node()))

tree.tick_while_running()

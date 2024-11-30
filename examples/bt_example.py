import time

import behaviortree_py

xml_text = """
<root BTCPP_format="4">

    <BehaviorTree ID="MainTree">
        <Sequence>
            <Fallback>
                <Inverter>
                    <IsDoorClosed/>
                </Inverter>
                <SubTree ID="DoorClosed"/>
            </Fallback>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="DoorClosed">
        <Fallback>
            <OpenDoor/>
            <RetryUntilSuccessful num_attempts="5">
                <PickLock/>
            </RetryUntilSuccessful>
            <SmashDoor/>
        </Fallback>
    </BehaviorTree>

</root>
"""


class CrossDoor:
    def __init__(self):
        self.door_open = False
        self.door_locked = True
        self.pick_attempts = 0

    def register_nodes(self, factory: behaviortree_py.BehaviorTreeFactory):
        factory.register_simple_condition("IsDoorClosed", self.is_door_closed)
        factory.register_simple_action("PassThroughDoor", self.pass_through_door)
        factory.register_simple_action("OpenDoor", self.open_door)
        factory.register_simple_action("PickLock", self.pick_lock)
        factory.register_simple_action("SmashDoor", self.smash_door)

    def is_door_closed(
        self,
        tree_node: behaviortree_py.TreeNode,
    ) -> behaviortree_py.NodeStatus:
        time.sleep(0.2)
        return (
            behaviortree_py.NodeStatus.SUCCESS
            if not self.door_open
            else behaviortree_py.NodeStatus.FAILURE
        )

    def pass_through_door(
        self,
        tree_node: behaviortree_py.TreeNode,
    ) -> behaviortree_py.NodeStatus:
        time.sleep(0.5)
        return (
            behaviortree_py.NodeStatus.SUCCESS
            if self.door_open
            else behaviortree_py.NodeStatus.FAILURE
        )

    def open_door(
        self,
        tree_node: behaviortree_py.TreeNode,
    ) -> behaviortree_py.NodeStatus:
        time.sleep(0.5)
        if self.door_locked:
            return behaviortree_py.NodeStatus.FAILURE
        self.door_open = True
        return behaviortree_py.NodeStatus.SUCCESS

    def pick_lock(
        self,
        tree_node: behaviortree_py.TreeNode,
    ) -> behaviortree_py.NodeStatus:
        time.sleep(0.5)
        self.pick_attempts += 1
        if self.pick_attempts > 3:
            self.door_locked = False
            self.door_open = True
        return (
            behaviortree_py.NodeStatus.SUCCESS
            if self.door_open
            else behaviortree_py.NodeStatus.FAILURE
        )

    def smash_door(self) -> behaviortree_py.NodeStatus:
        self.door_locked = False
        self.door_open = True
        return behaviortree_py.NodeStatus.SUCCESS


factory = behaviortree_py.BehaviorTreeFactory()

cross_door = CrossDoor()
cross_door.register_nodes(factory)

factory.register_behavior_tree_from_text(xml_text)

tree = factory.create_tree("MainTree")

logger = behaviortree_py.StdCoutLogger(tree)
# logger = behaviortree_py.Groot2Publisher(tree)

print(behaviortree_py.get_tree_recursively(tree.root_node()))

tree.tick_while_running()

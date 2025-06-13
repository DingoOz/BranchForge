# Behavior Tree XML Documentation for Claude Code

## Introduction to Behavior Trees

Behavior Trees are a modular and flexible way to model complex behaviors, commonly used in AI for games and robotics. They consist of a tree structure where each node represents a task or a condition. The tree is traversed from the root to the leaves, and the behavior is determined by the success or failure of the nodes.

## XML Structure

The Behavior Tree is defined in an XML file with the following structure:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <!-- nodes go here -->
  </BehaviorTree>
</root>
```

- `<root>`: The top-level element, optionally with a `BTCPP_format` attribute specifying the version (e.g., "4").
- `<BehaviorTree>`: Defines a tree with a unique `ID`. Multiple trees can exist within `<root>`.
- Nodes are nested inside `<BehaviorTree>` as XML tags.

Execution nodes can have **input and output ports**, specified as attributes using the blackboard syntax. For example:

```xml
<NodeName input_key="${blackboard_key}" output_key="${blackboard_key}"/>
```

- `${blackboard_key}`: Refers to a key in the blackboard, a shared memory space for passing data between nodes.

## Node Types

### Control Nodes

Control nodes manage the flow of the tree:

- **Sequence**: Executes children in order until one fails or all succeed.
  ```xml
  <Sequence>
    <ChildNode1/>
    <ChildNode2/>
  </Sequence>
  ```
- **Fallback (Selector)**: Executes children in order until one succeeds or all fail.
  ```xml
  <Fallback>
    <ChildNode1/>
    <ChildNode2/>
  </Fallback>
  ```
- **Parallel**: Executes all children simultaneously, with configurable success/failure policies.
  ```xml
  <Parallel success_count="2" failure_count="1">
    <ChildNode1/>
    <ChildNode2/>
  </Parallel>
  ```

### Execution Nodes

Execution nodes perform tasks or check conditions:

- **Action Nodes**: Perform a task (e.g., move, compute), returning success or failure.
  ```xml
  <ActionNodeName param="${value}"/>
  ```
- **Condition Nodes**: Check a condition (e.g., is goal reached?), returning success or failure.
  ```xml
  <ConditionNodeName check="${value}"/>
  ```

### Nav2-Specific Nodes

For ROS2 Nav2, custom nodes handle navigation tasks. Key examples include:

| Node Name          | Type      | Description                          |
|--------------------|-----------|--------------------------------------|
| `ComputePathToPose` | Action    | Computes a path to a given pose.     |
| `FollowPath`       | Action    | Follows a computed path.             |
| `GoalReached`      | Condition | Checks if the goal has been reached. |
| `ClearCostmap`     | Action    | Clears the costmap.                  |
| `Wait`             | Action    | Waits for a specified duration.      |

- Example usage:
  ```xml
  <ComputePathToPose goal="${goal_pose}" path="${path}"/>
  ```
- For a full list, see the [Nav2 Behavior Tree XML Nodes documentation](https://docs.nav2.org/plugin_tutorials/docs/nav2_behavior_tree_nodes.html).

## Examples

### Simple Example

A basic navigator that computes and follows a path:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="SimpleNavigator">
    <Sequence>
      <ComputePathToPose goal="${goal_pose}" path="${path}"/>
      <FollowPath path="${path}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- **Explanation**: The `Sequence` ensures that `ComputePathToPose` succeeds before `FollowPath` executes. Ports connect the path output to the follow action.

### Complex Example

A navigator with recovery behavior:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="ComplexNavigator">
    <Fallback>
      <Sequence>
        <ComputePathToPose goal="${goal_pose}" path="${path}"/>
        <FollowPath path="${path}"/>
      </Sequence>
      <ClearCostmap service_name="local_costmap/clear_costmap"/>
    </Fallback>
  </BehaviorTree>
</root>
```

- **Explanation**: The `Fallback` tries the `Sequence` first. If it fails (e.g., path computation or following fails), it attempts `ClearCostmap` as a recovery action.

Subtrees can also be used for modularity:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="Navigate"/>
      <SubTree ID="Recover"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Navigate">
    <Sequence>
      <ComputePathToPose goal="${goal_pose}" path="${path}"/>
      <FollowPath path="${path}"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Recover">
    <ClearCostmap service_name="local_costmap/clear_costmap"/>
  </BehaviorTree>
</root>
```

## Best Practices

- **Naming**: Use meaningful `ID`s and node names (e.g., "Navigate" instead of "Tree1").
- **Modularity**: Split complex trees into subtrees for reusability.
- **Blackboard**: Use ports (`${key}`) to pass data dynamically, avoiding hardcoded values.
- **Completeness**: Ensure all required ports are specified in the XML.
- **Debugging**: Add logging nodes or use visualization tools like Groot.

## Validation Tools

Use [Groot](https://github.com/BehaviorTree/Groot) to visualize and validate your XML. Load `nav2_tree_nodes.xml` to support Nav2-specific nodes.
## Setting up `<arm>_moveit_config`

Two *YAML* files must be present in `config/`. If you see they are already present, still **make sure the contents match my example files**; except for the `cartesian_limits` values, put whatever suits you there.

### `<arm>_moveit_config/config/pilz_cartesian_limits.yaml`

```yaml
# Cartesian limits for the Pilz planner
cartesian_limits:
  max_trans_vel: 1.0 # m/s
  max_trans_acc: 4.25 # m/s^2
  max_trans_dec: -5.0 # m/s^2
  max_rot_vel: 1.57 # rad/s
```

### `<arm>_moveit_config/config/pilz_industrial_motion_planner_planning.yaml`

```yaml
planning_plugin: pilz_industrial_motion_planner/CommandPlanner
request_adapters: >default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
default_planner_config: LIN
capabilities: >pilz_industrial_motion_planner/MoveGroupSequenceAction
  pilz_industrial_motion_planner/MoveGroupSequenceService
```

## Launch File

### `MoveItConfigsBuilder()`

The `pilz_industrial_motion_planner` must be included in the planning pipelines.

```python
.planning_pipelines(
	pipelines=["ompl", "pilz_industrial_motion_planner"]
)
```

> [!tip]
> If you plan to launch your `move_group` client outside of the launch file, you can enable the publishing of the robot description as topics so that you do not require `MoveItConfigsBuilder` again
>
> ```python
> .planning_scene_monitor(
> 	publish_robot_description=True, 
> 	publish_robot_description_semantic=True
> )
> ```

### `move_group` Advertising Multi-Segment Capabilities [Bug #1248](https://github.com/moveit/moveit2/issues/1248#issuecomment-1350773802)

> [!warning]
> This only applies to *Humble* and earlier distributions ().

> [!NOTE]
> This is only required for multi-segment planning

Pass the following parameter to your `move_group` *Node* in your launch file.

```python
move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

move_group_node = Node(
	package="moveit_ros_move_group",
	executable="move_group",
	output="screen",
	parameters=[moveit_config.to_dict(), move_group_capabilities],
)
```

## Example Launch File

In this repository ([here]([url](https://github.com/gaspatxo/ros2_guides/blob/main/moveit2/panda_demo_with_pilz.launch.py))) you can find is the modified version of [`demo.launch.py`](https://github.com/moveit/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py) from `panda_moveit_config` (humble).

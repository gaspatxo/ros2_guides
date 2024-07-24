Instead of writing a guide, I made a [minimal example](https://github.com/gaspatxo/ros2_guides/blob/main/moveit2/minimal_pilz_sequence_client.cpp) you can use as a template. Even though the `move_group` client cannot be used to interact with the service, it is still useful to ease the crafting of the message; this is something I have not seen anywhere else being done, but it really simplifies things. The action request is generated using using `move_group_->constructMotionPlanRequest()`.

References:

- [Pilz Industrial Motion Planner — MoveIt Documentation: Rolling documentation](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#sequence-of-multiple-segments)
- [Pilz Industrial Motion Planner blend generates points with same time\_from\_start · Issue #2741 · moveit/moveit2 · GitHub](https://github.com/moveit/moveit2/issues/2741)

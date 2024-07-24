This guide is aimed at developers with experience using vanilla move_group interface. It will cover how to:

- [Setup and Configure the Pilz Planner](https://github.com/gaspatxo/ros2_guides/blob/main/moveit2/setup_and_configuration_for_pilz_planner.md)
- [Plan a Sequence of moves with Pilz - aka Multi-Segment Planning](https://github.com/gaspatxo/ros2_guides/blob/main/moveit2/multi-segment_planning_with_pilz.md)

The panda robot arm will be used for exemplification, though it should apply to any robot arm with standard moveit2 config packages.

I wrote this guide because I could not find anything similar on the internet and had to figure a lot of things out myself by reverse-engineering, navigating issues and very old forum posts. Hopefully this will spare some headaches. This guide is not only a how-to but also compiles many of the bugs/nuances that one must know of to use the Pilz planner.

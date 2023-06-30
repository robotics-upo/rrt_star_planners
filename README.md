# rrt_star_planners
RRT Star Planner adapted to get path for tied marsupial robotic configuration

## Testing:

### Steps for UGV/UAV:
```
roslaunch rrt_planners random_planner_marsupial.launch map:=stage_2 use_parable:=true
roslaunch rrt_planners test.launch map_file:=stage_2
roslaunch marsupial_optimizer behavior_tree_node_marsupial.launch
roscd marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_2
```

### Steps for UAV with UGV fixed:

```
roslaunch rrt_planners random_planner_uav.launch map:=stage_2 use_parable:=true
roslaunch rrt_planners test_uav.launch map_file:=stage_2
roslaunch marsupial_optimizer behavior_tree_node_marsupial.launch
roscd marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_2
```


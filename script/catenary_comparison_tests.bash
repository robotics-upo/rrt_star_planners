#! /bin/bash

# Check for the correct number of arguments (two: the number of stage and the number of initial condition)

if [ $# -ne 2 ]; then
    echo "Usage: $0 <stage number> <initial cond. number>"
    exit 1
fi

# Setup the environment

roslaunch rrt_planners test_uav.launch map_file:=stage_${1} &
roslaunch rrt_planners random_planner_uav.launch map:=stage_$1 use_parable:=true &
roslaunch marsupial_optimizer behavior_tree_node_marsupial.launch &


# Add a number of tests --> this is not necessary: 
CONTADOR=1
until [ $CONTADOR -gt 1 ]; do
    roscd marsupial_optimizer/trees/script/ && ./execute_tree.sh $HOME/rrt_stage_$1
    ostopic echo -n 1 /Make_Plan/result  # Wait for the response before the next
    let CONTADOR+=1
done

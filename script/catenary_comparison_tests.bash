#! /bin/bash

# Check for the correct number of arguments (two: the number of stage and the number of initial condition)

if [ $# -ne 1 ]; then
    echo "Usage: $0 <final stage number>"
    exit 1
fi

# Setup the environment

# Add a number of tests --> this is not necessary: 
CONTADOR=1
until [ $CONTADOR -gt 1 ]; do
    roslaunch rrt_planners test_uav.launch map_file:=stage_${CONTADOR} &
    roslaunch rrt_planners random_planner_uav.launch map:=stage_$CONTADOR &
    sleep 7
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 3 /behavior_tree/execution_status   # Wait for the response before the next


    rosnode kill /random_planner_node
    sleep 5
    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_parable:=true &
    sleep 5
    roscd marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 3 /behavior_tree/execution_status

    rosnode kill /random_planner_node
    sleep 5
    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_both:=true &
    sleep 5
    roscd marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 3 /behavior_tree/execution_status

    # TODO: add the _2 information
    
    echo Performed Tests Contador = $CONTADOR
    rosnode kill -a
    sleep 20
    let CONTADOR+=1
done

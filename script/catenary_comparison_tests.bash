#! /bin/bash

# Check for the correct number of arguments (two: the number of stage and the number of initial condition)

if [ $# -ne 2 ]; then
    echo "Usage: $0 <init stage numebr> <final stage number>"
    exit 1
fi

# Setup the environment
TARGET=$HOME/catenary_stats
mkdir -p $TARGET

# Add a number of tests --> this is not necessary: 
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
    roslaunch rrt_planners test_uav.launch map_file:=stage_${CONTADOR} &
    roslaunch rrt_planners random_planner_uav.launch map:=stage_$CONTADOR &
    sleep 7
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 3 /behavior_tree/execution_status   # Wait for the response before the next
    rosnode kill /random_planner_node
    sleep 5
    mv $HOME/.ros/rrt_stats.txt $TARGET/rrt_stats_bisection_$CONTADOR.txt

    # Second problem, bisection
    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR &
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}_2.xml
    rostopic echo -n 3 /behavior_tree/execution_status
    rosnode kill /random_planner_node
    sleep 5
    mv $HOME/.ros/rrt_stats.txt $TARGET/rrt_stats_bisection_${CONTADOR}_2.txt

    # Parable approximation
    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_parable:=true &
    sleep 7
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 3 /behavior_tree/execution_status
    rosnode kill /random_planner_node
    sleep 5
    mv $HOME/.ros/rrt_stats.txt $TARGET/rrt_stats_parable_$CONTADOR.txt
    mv $HOME/.ros/catenary_stats.txt $TARGET/precomputing_parable_catenary_stats_${CONTADOR}.txt

    #Second problem parable
    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_parable:=true &
    sleep 7
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}_2.xml
    rostopic echo -n 3 /behavior_tree/execution_status
    rosnode kill /random_planner_node
    sleep 5
    mv $HOME/.ros/rrt_stats.txt $TARGET/rrt_stats_parable_${CONTADOR}_2.txt
    mv $HOME/.ros/catenary_stats.txt $TARGET/precomputing_parable_catenary_stats_${CONTADOR}_2.txt

    # Comparison parable/
    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_both:=true &
    sleep 7
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 3 /behavior_tree/execution_status
    rosnode kill /random_planner_node
    mv $HOME/.ros/catenary_stats.txt $HOME/catenary_stats_${CONTADOR}.txt

    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_both:=true &
    sleep 5
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}_2.xml
    rostopic echo -n 3 /behavior_tree/execution_status
    rosnode kill /random_planner_node
    mv $HOME/.ros/catenary_stats.txt $TARGET/catenary_stats_${CONTADOR}_2.txt

    echo Performed Tests Contador = $CONTADOR
    rosnode kill -a
    sleep 20

    let CONTADOR+=1
done

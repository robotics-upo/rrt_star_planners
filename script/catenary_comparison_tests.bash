#! /bin/bash

# Check for the correct number of arguments (two: the number of stage and the number of initial condition)

# TODO: Use the same tests as Simon for the comparison and the marsupial whole planner
# TODO: The goal are the same for each scenario --> marsupial_optimizer/trees/,,,,
# TODO: The initial conditions vary --> marsupial_optimizer/cfg/stage_....


if [ $# -ne 2 ]; then
    echo "Usage: $0 <init stage number> <final stage number>"
    exit 1
fi

# Add a number of tests --> this is not necessary: 
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
    roslaunch rrt_planners random_planner_marsupial.launch map:=stage_$CONTADOR &
    sleep 20
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}.xml
    rostopic echo -n 1 /behavior_tree/load_tree_action/result  # Wait for the response before the next
    rosnode kill /random_planner_node
    sleep 15
     
    # # # Second problem, bisection
    roslaunch rrt_planners random_planner_marsupial.launch map:=stage_$CONTADOR num_pos_initial:="2" &
    sleep 20
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}_2.xml
    rostopic echo /behavior_tree/load_tree_action/result -n 1
    rosnode kill /random_planner_node
    sleep 15

    # # Parabola approximation
    roslaunch rrt_planners random_planner_marsupial.launch map:=stage_$CONTADOR use_parable:=true &
    sleep 20
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}.xml
    rostopic echo -n 1 /behavior_tree/load_tree_action/result
    rosnode kill /random_planner_node
    sleep 15

    #Second problem parabola
    roslaunch rrt_planners random_planner_marsupial.launch map:=stage_$CONTADOR use_parable:=true num_pos_initial:="2" &
    sleep 20
    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
    rostopic echo -n 1 /behavior_tree/load_tree_action/result
    rosnode kill /random_planner_node
    sleep 15
 
    # Comparison parable/catenary runtime TODO: integrate!
#    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_both:=true &
    #sleep 7
#    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
#    rostopic echo -n 3 /behavior_tree/execution_status
#    rosnode kill /random_planner_node
#    sleep 5
#    mv $HOME/.ros/catenary_stats.txt $TARGET/catenary_stats_${CONTADOR}.txt

#    roslaunch rrt_planners random_global_uav_planner.launch map_file:=stage_$CONTADOR use_both:=true &
#    sleep 5
#    cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_${CONTADOR}_2.xml
#    rostopic echo -n 3 /behavior_tree/execution_status
#    rosnode kill /random_planner_node
#    sleep 5
#    mv $HOME/.ros/catenary_stats.txt $TARGET/catenary_stats_${CONTADOR}_2.txt

    echo Performed Tests of Stage = $CONTADOR
    rosnode kill -a
    sleep 2

    let CONTADOR+=1
done

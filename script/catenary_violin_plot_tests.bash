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
TARGET=$HOME
until [ $CONTADOR -gt $2 ]; do 
    # Comparison parable/catenary runtime TODO: integrate!
   roslaunch rrt_planners random_planner_marsupial.launch map:=stage_$CONTADOR use_both:=true &
   sleep 20
   cd $HOME/marsupial_ws/src/marsupial_optimizer/trees/script/ && ./execute_tree.sh stage_$CONTADOR.xml
   rostopic echo -n 3 /behavior_tree/execution_status
   rosnode kill /random_planner_node
   sleep 20
   mv $HOME/.ros/catenary_stats.txt $TARGET/catenary_stats_${CONTADOR}.txt


    echo Performed Tests of Stage = $CONTADOR
    rosnode kill -a
    sleep 2

    let CONTADOR+=1
done

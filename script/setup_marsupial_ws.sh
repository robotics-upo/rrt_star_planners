read -p "Installing g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash



# ARE MISSING ALL G2O DEPENDENCIES   (only that)   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



#---------------Dependences ------------------
sudo apt-get install ros-melodic-costmap-2d 

sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-dbgsym ros-melodic-octomap-msgs ros-melodic-octomap-ros 
sudo apt-get install ros-melodic-octomap-ros-dbgsym ros-melodic-octomap-ros-dbgsym ros-melodic-octomap-rviz-plugins ros-melodic-octomap-rviz-plugins-dbgsym 
sudo apt-get install ros-melodic-octomap-server ros-melodic-octomap-server-dbgsym

sudo apt-get install ros-melodic-timed-roslaunch

sudo apt-get intall libpcl-dev
sudo apt-get install libpcl
sudo apt-get install libpcl-kdtree1.8

#-------------- Packages --------------------
git clone -b ceres_solver https://github.com/robotics-upo/marsupial_optimizer.git

#To install planner lazy-theta*
git clone -b marsupial https://github.com/robotics-upo/lazy_theta_star_planners.git

#To get action for actionlib
git clone https://github.com/robotics-upo/upo_actions.git

#To get a marker in the desired frame_link
git clone  https://github.com/robotics-upo/upo_markers.git

#To install Behavior tree
sudo apt-get install ros-$ROS_DISTRO-ros-type-introspection
sudo apt-get install libqt5svg5-dev qtbase5-dev  
sudo apt install libdw-dev
git clone -b v2 https://github.com/robotics-upo/Groot.git
git clone -b mbzirc https://github.com/robotics-upo/behavior_tree_ros.git
git clone -b mbzirc https://github.com/robotics-upo/behavior_tree_plugins.git
# git clone -b v2 https://github.com/robotics-upo/BehaviorTree.CPP.git #git checkout 05126fd0bfa46cfad1e3ad5e0261f59e76934009
git clone -b mbzirc https://github.com/robotics-upo/BehaviorTree.CPP.git 

#-------------- For CERES SOLVER -----------------
cd ~
git clone https://ceres-solver.googlesource.com/ceres-solver

# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev

#tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
#cmake ../ceres-solver-2.0.0
cmake ../ceres-solver
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
sudo make install

# cd ceres-bin/bin 
# ./simple_bundle_adjuster ~/ceres-solver/data/problem-16-22106-pre.txt


#-------------- For GAZEBO 9 SIMULATOR -----------------
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo9
sudo apt-get install libgazebo9-dev
sudo apt upgrade libignition-math2

git clone -b melodic https://github.com/robotics-upo/rotors_simulator.git
read -p "Installing g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash



# ARE MISSING ALL G2O DEPENDENCIES   (only that)   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



#---------------Dependences ------------------
sudo apt-get install ros-melodic-costmap-2d 

sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-dbgsym ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-ros-dbgsym ros-melodic-octomap-ros-dbgsym ros-melodic-octomap-rviz-plugins ros-melodic-octomap-rviz-plugins-dbgsym ros-melodic-octomap-server ros-melodic-octomap-server-dbgsym

sudo apt-get install ros-melodic-timed-roslaunch

sudo apt-get intall libpcl-dev
sudo apt-get install libpcl
sudo apt-get install libpcl-kdtree1.8

#-------------- Packages --------------------
git clone https://github.com/robotics-upo/marsupial_g2o.git
git checkout ceres_solver

#To install planner lazy-theta*
https://github.com/robotics-upo/lazy_theta_star_planners.git
git checkout marsupial 

#To get action for actionlib
git clone https://github.com/robotics-upo/upo_actions.git

#To get a marker in the desired frame_link
git clone  https://github.com/robotics-upo/upo_markers.git

#To install Behavior tree
sudo apt-get install libqt5svg5-dev ros-$ROS_DISTRO-ros-type-introspection
git clone https://github.com/robotics-upo/Groot.git
git clone -b develop https://github.com/robotics-upo/behavior_tree_ros.git
git clone -b mbzirc https://github.com/robotics-upo/behavior_tree_plugins.git
git clone https://github.com/robotics-upo/BehaviorTree.CPP.git

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

cd ceres-bin/bin 
./simple_bundle_adjuster ~/ceres-solver/data/problem-16-22106-pre.txt


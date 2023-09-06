#include <rrt_planners/check_collision_path_planner.h>


checkCollisionPathPlanner::checkCollisionPathPlanner(std::string node_name_, Grid3d *grid_3D_, sensor_msgs::PointCloud2::ConstPtr pc_, geometry_msgs::Vector3 p_reel_ugv_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    node_name = node_name_;
	grid_3D = grid_3D_;
	pc_obs_ugv = pc_;
	nn_obs_ugv.setInput(*pc_obs_ugv);
	p_reel_ugv = p_reel_ugv_;
	nh.reset(new ros::NodeHandle("~"));

    nh->param("distance_obstacle_ugv", distance_obstacle_ugv, (double)1.0);
    nh->param("distance_obstacle_uav", distance_obstacle_uav, (double)1.0);
    nh->param("distance_catenary_obstacle", distance_catenary_obstacle, (double)0.1);
}

void checkCollisionPathPlanner::CheckStatus(trajectory_msgs::MultiDOFJointTrajectory mt_, std::vector<double> ct_)
{
	geometry_msgs::Vector3 p_ugv_, p_uav_, p_reel_; 
	std::vector<geometry_msgs::Vector3> points_catenary_;
    double len_cat_, dist_;
	bisectionCatenary bc;
	int count_ugv, count_uav, count_cat;
	count_ugv = count_uav = count_cat = 0;

    std::cout << std::endl << "	checkCollisionPathPlanner started: analizing collision status for the marsupial agents" << std::endl; 
	
	for (size_t i= 0 ; i <  mt_.points.size(); i++){
		p_ugv_.x = mt_.points.at(i).transforms[0].translation.x;
		p_ugv_.y = mt_.points.at(i).transforms[0].translation.y;
		p_ugv_.z = mt_.points.at(i).transforms[0].translation.z;
		p_uav_.x = mt_.points.at(i).transforms[1].translation.x;
		p_uav_.y = mt_.points.at(i).transforms[1].translation.y;
		p_uav_.z = mt_.points.at(i).transforms[1].translation.z;

        dist_ = getPointDistanceFullMap(true, p_ugv_,i,"UGV");
        if (dist_ < distance_obstacle_ugv){
			count_ugv++;
            std::cout << "      The agent UGV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
        }

        dist_ = getPointDistanceFullMap(false, p_uav_,i,"UAV");
        if (dist_ < distance_obstacle_uav){
            count_uav++;
		    std::cout << "      The agent UAV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
		}
		geometry_msgs::Vector3 p_;
		geometry_msgs::Quaternion q_;
		p_.x = p_ugv_.x;
		p_.y = p_ugv_.y;
		p_.z = p_ugv_.z+0.1;	//Move in Z to see the point over the map surface
		q_.x = mt_.points.at(i).transforms[0].rotation.x;
		q_.y = mt_.points.at(i).transforms[0].rotation.y;
		q_.z = mt_.points.at(i).transforms[0].rotation.z;
		q_.w = mt_.points.at(i).transforms[0].rotation.w;
		p_reel_ = getReelNode(p_,q_);


		// Catenary 
		len_cat_ = ct_[i];

		points_catenary_.clear();
		bool just_one_axe = bc.configBisection(len_cat_, p_reel_.x, p_reel_.y, p_reel_.z, p_uav_.x, p_uav_.y, p_uav_.z);
		bc.getPointCatenary3D(points_catenary_, false);

		for (size_t j = 0 ; j < points_catenary_.size() ; j++ ) {
            dist_ = getPointDistanceFullMap(false, points_catenary_[j],i,"CATENARY") ;
            if( dist_ < distance_catenary_obstacle){
            	count_cat++;
            	std::cout << " 		The agent CATENARY in the state = " << i << " position [" << j <<"]is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
			}
		}
	}

	if (count_ugv > 0 || count_uav > 0 || count_cat > 0)
		ROS_INFO_COND(true, PRINTF_RED "checkCollisionPathPlanner: Marsupial system in collision for RRT solution [ugv=%i  uav=%i  catenary=%i]",count_ugv, count_uav,count_cat);
	else
		ROS_INFO_COND(true, PRINTF_GREEN "checkCollisionPathPlanner: Marsupial system in collision for RRT solution [ugv=%i  uav=%i  catenary=%i]",count_ugv, count_uav,count_cat);


    std::cout << "	checkCollisionPathPlanner finished" << std::endl << std::endl; 

}

double checkCollisionPathPlanner::getPointDistanceFullMap(bool ugv_obstacle_, geometry_msgs::Vector3 p_, int pose_, string msg_)
{
	double dist;

	if(!ugv_obstacle_){
		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
		if(is_into_)
			dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
		else{
            std::cout << "  The agent " << msg_ << "in the state = " << pose_ << " is out of the GRID" << std::endl; 
			dist = -1.0;
        }
    }
	else{
		Eigen::Vector3d pos_, obs_;
		pos_.x() = p_.x;
		pos_.y() = p_.y; 
		pos_.z() = p_.z; 
		obs_= nn_obs_ugv.nearestObstacleMarsupial(nn_obs_ugv.kdtree, pos_, nn_obs_ugv.obs_points);

		dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) + pow(obs_.z()-pos_.z(),2));
	}

	return dist;
}

geometry_msgs::Vector3 checkCollisionPathPlanner::getReelNode(const geometry_msgs::Vector3 p_, const geometry_msgs::Quaternion q_)
{
	geometry_msgs::Vector3 pos_reel;
	double yaw_ugv;

	yaw_ugv = getYawFromQuaternion(q_.x, q_.y, q_.z, q_.w);
	double lengt_vec =  sqrt(p_reel_ugv.x*p_reel_ugv.x + p_reel_ugv.y*p_reel_ugv.y);
	pos_reel.x = p_.x + lengt_vec *cos(yaw_ugv); 
	pos_reel.y = p_.y + lengt_vec *sin(yaw_ugv);
	pos_reel.z = p_.z + p_reel_ugv.z ; 
	return pos_reel;
}

double checkCollisionPathPlanner::getYawFromQuaternion(double x_, double y_, double z_, double w_)
{
	double roll_, pitch_, yaw_;

	tf::Quaternion q( x_, y_, z_, w_);
	tf::Matrix3x3 M(q);	
	M.getRPY(roll_, pitch_, yaw_);

	return yaw_;
}
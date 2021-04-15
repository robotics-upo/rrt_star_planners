#include <rrt_planners/rrt_graph_markers.h>


RRTGraphMarkers::RRTGraphMarkers()
{

}

void RRTGraphMarkers::configGraphMarkers(std::string frame_id_, float step_, bool is_coupled_, int n_iter_, geometry_msgs::Vector3 pos_reel_ugv_)
{
    frame_id = frame_id_;
    step = step_;
    is_coupled = is_coupled_;
    n_iter = n_iter_;
	pos_reel_ugv = pos_reel_ugv_;
}


void RRTGraphMarkers::getGraphMarker(std::list<RRTNode*> nodes_tree_, ros::Publisher tree_rrt_star_ugv_pub_, ros::Publisher tree_rrt_star_uav_pub_)
{
    pointTreeMarkerUGV.markers.resize(nodes_tree_.size());

	int count = 0; 
    for (auto nt_:nodes_tree_) {
        pointTreeMarkerUGV.markers[count].header.frame_id = frame_id;
        pointTreeMarkerUGV.markers[count].header.stamp = ros::Time::now();
        pointTreeMarkerUGV.markers[count].ns = "tree_RRTStar_ugv";
        pointTreeMarkerUGV.markers[count].id = nt_->id;
        pointTreeMarkerUGV.markers[count].action = visualization_msgs::Marker::ADD;
        pointTreeMarkerUGV.markers[count].type = visualization_msgs::Marker::CYLINDER;
        pointTreeMarkerUGV.markers[count].lifetime = ros::Duration(180);
        pointTreeMarkerUGV.markers[count].pose.position.x = nt_->point.x * step; 
        pointTreeMarkerUGV.markers[count].pose.position.y = nt_->point.y * step; 
        pointTreeMarkerUGV.markers[count].pose.position.z = nt_->point.z * step ; //Move in Z to see the point over the map surface
        pointTreeMarkerUGV.markers[count].pose.orientation.x = 0.0;
        pointTreeMarkerUGV.markers[count].pose.orientation.y = 0.0;
        pointTreeMarkerUGV.markers[count].pose.orientation.z = 0.0;
        pointTreeMarkerUGV.markers[count].pose.orientation.w = 1.0;
        pointTreeMarkerUGV.markers[count].scale.x = 0.08;
        pointTreeMarkerUGV.markers[count].scale.y = 0.08;
        pointTreeMarkerUGV.markers[count].scale.z = 0.28;
        pointTreeMarkerUGV.markers[count].color.r=1.0;
        pointTreeMarkerUGV.markers[count].color.g=1.0;
        pointTreeMarkerUGV.markers[count].color.b=1.0;
        pointTreeMarkerUGV.markers[count].color.a=1.0; 
		count++;
    }	
    tree_rrt_star_ugv_pub_.publish(pointTreeMarkerUGV);

	if(!is_coupled){
		pointTreeMarkerUAV.markers.resize(nodes_tree_.size());

		int count = 0; 
		for (auto nt_:nodes_tree_) {
			pointTreeMarkerUAV.markers[count].header.frame_id = frame_id;
			pointTreeMarkerUAV.markers[count].header.stamp = ros::Time::now();
			pointTreeMarkerUAV.markers[count].ns = "tree_RRTStar_uav";
			pointTreeMarkerUAV.markers[count].id = nt_->id_uav;
			pointTreeMarkerUAV.markers[count].action = visualization_msgs::Marker::ADD;
			pointTreeMarkerUAV.markers[count].type = visualization_msgs::Marker::SPHERE;
			pointTreeMarkerUAV.markers[count].lifetime = ros::Duration(180);
			pointTreeMarkerUAV.markers[count].pose.position.x = nt_->point_uav.x * step; 
			pointTreeMarkerUAV.markers[count].pose.position.y = nt_->point_uav.y * step; 
			pointTreeMarkerUAV.markers[count].pose.position.z = nt_->point_uav.z * step;
			pointTreeMarkerUAV.markers[count].pose.orientation.x = 0.0;
			pointTreeMarkerUAV.markers[count].pose.orientation.y = 0.0;
			pointTreeMarkerUAV.markers[count].pose.orientation.z = 0.0;
			pointTreeMarkerUAV.markers[count].pose.orientation.w = 1.0;
			pointTreeMarkerUAV.markers[count].scale.x = 0.1;
			pointTreeMarkerUAV.markers[count].scale.y = 0.1;
			pointTreeMarkerUAV.markers[count].scale.z = 0.1;
			pointTreeMarkerUAV.markers[count].color.r=0.2;
			pointTreeMarkerUAV.markers[count].color.g=0.2;
			pointTreeMarkerUAV.markers[count].color.b=1.0;
			pointTreeMarkerUAV.markers[count].color.a=1.0; 
			count++;
		}	
		tree_rrt_star_uav_pub_.publish(pointTreeMarkerUAV);
	}
}

void RRTGraphMarkers::getTakeOffNodesMarker(std::list<RRTNode*> take_off_nodes_, ros::Publisher take_off_nodes_pub_)
{ 
	pointTakeOffMarker.markers.resize(take_off_nodes_.size());

	int count_ = 0; 
	for (auto nt_:take_off_nodes_) {
		pointTakeOffMarker.markers[count_].header.frame_id = frame_id;
		pointTakeOffMarker.markers[count_].header.stamp = ros::Time::now();
		pointTakeOffMarker.markers[count_].ns = "take_off_nodes_RRTStar";
		pointTakeOffMarker.markers[count_].id = nt_->id;
		pointTakeOffMarker.markers[count_].action = visualization_msgs::Marker::ADD;
		pointTakeOffMarker.markers[count_].type = visualization_msgs::Marker::SPHERE;
		pointTakeOffMarker.markers[count_].lifetime = ros::Duration(180);
		pointTakeOffMarker.markers[count_].pose.position.x = nt_->point.x * step; 
		pointTakeOffMarker.markers[count_].pose.position.y = nt_->point.y * step; 
		pointTakeOffMarker.markers[count_].pose.position.z = nt_->point.z * step;
		pointTakeOffMarker.markers[count_].pose.orientation.x = 0.0;
		pointTakeOffMarker.markers[count_].pose.orientation.y = 0.0;
		pointTakeOffMarker.markers[count_].pose.orientation.z = 0.0;
		pointTakeOffMarker.markers[count_].pose.orientation.w = 1.0;
		pointTakeOffMarker.markers[count_].scale.x = 0.1;
		pointTakeOffMarker.markers[count_].scale.y = 0.1;
		pointTakeOffMarker.markers[count_].scale.z = 0.1;
		pointTakeOffMarker.markers[count_].color.r=1.0;
		pointTakeOffMarker.markers[count_].color.g=0.0;
		pointTakeOffMarker.markers[count_].color.b=0.0;
		pointTakeOffMarker.markers[count_].color.a=0.5; 
		count_++;
	}	
	take_off_nodes_pub_.publish(pointTakeOffMarker);
}

void RRTGraphMarkers::getPathMarker(std::list<RRTNode*> pt_, ros::Publisher lines_ugv_marker_pub_, ros::Publisher lines_uav_marker_pub_)
{
	geometry_msgs::Point _p1, _p2; 

	lines_ugv_marker.markers.resize(pt_.size()-1);
	
	int i_ = 0;
	for (auto p_:pt_){
		_p2.x = p_->point.x*step;
		_p2.y = p_->point.y*step;
		_p2.z = p_->point.z*step+0.1;
		if (i_ > 0){
			lines_ugv_marker.markers[i_-1].header.frame_id = frame_id;
			lines_ugv_marker.markers[i_-1].header.stamp = ros::Time::now();
			lines_ugv_marker.markers[i_-1].ns = "Line_ugv_RRTStar_Path";
			lines_ugv_marker.markers[i_-1].id = i_ + pt_.size();
			lines_ugv_marker.markers[i_-1].action = visualization_msgs::Marker::ADD;
			lines_ugv_marker.markers[i_-1].type = visualization_msgs::Marker::LINE_STRIP;
			lines_ugv_marker.markers[i_-1].lifetime = ros::Duration(0);
			lines_ugv_marker.markers[i_-1].points.push_back(_p1);
			lines_ugv_marker.markers[i_-1].points.push_back(_p2);
			lines_ugv_marker.markers[i_-1].pose.orientation.x = 0.0;
			lines_ugv_marker.markers[i_-1].pose.orientation.y = 0.0;
			lines_ugv_marker.markers[i_-1].pose.orientation.z = 0.0;
			lines_ugv_marker.markers[i_-1].pose.orientation.w = 1.0;
			lines_ugv_marker.markers[i_-1].scale.x = 0.1;
			// lines_ugv_marker.markers[i].scale.y = 0.3;
			// lines_ugv_marker.markers[i].scale.z = 0.1;
			lines_ugv_marker.markers[i_-1].color.a = 1.0;
			lines_ugv_marker.markers[i_-1].color.r = 0.0;
			lines_ugv_marker.markers[i_-1].color.g = 0.0;
			lines_ugv_marker.markers[i_-1].color.b = 1.0;
		}
		_p1.x = p_->point.x*step;
		_p1.y = p_->point.y*step;
		_p1.z = p_->point.z*step+0.1;	//Move in Z to see the point over the map surface
		i_++;
	}
	lines_ugv_marker_pub_.publish(lines_ugv_marker);

	if(!is_coupled){
		lines_uav_marker.markers.resize(pt_.size()-1);
		i_ = 0;

		for (auto p_:pt_){
			_p2.x = p_->point_uav.x*step;
			_p2.y = p_->point_uav.y*step;
			_p2.z = p_->point_uav.z*step;
			if (i_ > 0){
				lines_uav_marker.markers[i_-1].header.frame_id = frame_id;
				lines_uav_marker.markers[i_-1].header.stamp = ros::Time::now();
				lines_uav_marker.markers[i_-1].ns = "Line_uav_RRTStar_Path";
				lines_uav_marker.markers[i_-1].id = i_ + pt_.size();
				lines_uav_marker.markers[i_-1].action = visualization_msgs::Marker::ADD;
				lines_uav_marker.markers[i_-1].type = visualization_msgs::Marker::LINE_STRIP;
				lines_uav_marker.markers[i_-1].lifetime = ros::Duration(0);
				lines_uav_marker.markers[i_-1].points.push_back(_p1);
				lines_uav_marker.markers[i_-1].points.push_back(_p2);
				lines_uav_marker.markers[i_-1].pose.orientation.x = 0.0;
				lines_uav_marker.markers[i_-1].pose.orientation.y = 0.0;
				lines_uav_marker.markers[i_-1].pose.orientation.z = 0.0;
				lines_uav_marker.markers[i_-1].pose.orientation.w = 1.0;
				lines_uav_marker.markers[i_-1].scale.x = 0.1;
				// lines_uav_marker.markers[i].scale.y = 0.3;
				// lines_uav_marker.markers[i].scale.z = 0.1;
				lines_uav_marker.markers[i_-1].color.a = 1.0;
				lines_uav_marker.markers[i_-1].color.r = 1.0;
				lines_uav_marker.markers[i_-1].color.g = 1.0;
				lines_uav_marker.markers[i_-1].color.b = 1.0;
			}
			_p1.x = p_->point_uav.x*step;
			_p1.y = p_->point_uav.y*step;
			_p1.z = p_->point_uav.z*step;
			i_++;
		}
		lines_uav_marker_pub_.publish(lines_uav_marker);
	}
}

void RRTGraphMarkers::getCatenaryMarker(vector<geometry_msgs::Point> points_catenary_, ros::Publisher one_catenary_marker_pub_){
	std::string string_marker;
    std::string ns_marker;

	double c_color1, c_color2, c_color3;
	visualization_msgs::MarkerArray oneCatenaryMarker;
	
	oneCatenaryMarker.markers.clear();
	oneCatenaryMarker.markers.resize(points_catenary_.size());

	for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
		c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
		c_color2 = ((double)i / (double)points_catenary_.size())*0.5;
		if (i%2 == 0)
		c_color3 = 0.5;
		else
		c_color3 = 0.0;
		oneCatenaryMarker.markers[i].header.frame_id = frame_id;
		oneCatenaryMarker.markers[i].header.stamp = ros::Time::now();
		oneCatenaryMarker.markers[i].ns = "one_catenary";
		oneCatenaryMarker.markers[i].id = 1 + i*10.0;
		oneCatenaryMarker.markers[i].action = visualization_msgs::Marker::ADD;
		oneCatenaryMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
		oneCatenaryMarker.markers[i].lifetime = ros::Duration(180);
		oneCatenaryMarker.markers[i].pose.position.x = points_catenary_[i].x; 
		oneCatenaryMarker.markers[i].pose.position.y = points_catenary_[i].y; 
		oneCatenaryMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
		oneCatenaryMarker.markers[i].pose.orientation.x = 0.0;
		oneCatenaryMarker.markers[i].pose.orientation.y = 0.0;
		oneCatenaryMarker.markers[i].pose.orientation.z = 0.0;
		oneCatenaryMarker.markers[i].pose.orientation.w = 1.0;
		oneCatenaryMarker.markers[i].scale.x = 0.06;
		oneCatenaryMarker.markers[i].scale.y = 0.06;
		oneCatenaryMarker.markers[i].scale.z = 0.06;
		oneCatenaryMarker.markers[i].color.r = 1.0 - c_color1;
		oneCatenaryMarker.markers[i].color.g = c_color2;
		oneCatenaryMarker.markers[i].color.b = c_color3;
		oneCatenaryMarker.markers[i].color.a = 1.0; 
	}
	one_catenary_marker_pub_.publish(oneCatenaryMarker);
}

void RRTGraphMarkers::getCatenaryPathMarker(std::list<RRTNode*> ct_, ros::Publisher catenary_marker_pub_)
{
    std::string string_marker;
    std::string ns_marker;
	double c_color1, c_color2, c_color3;
	std::vector<geometry_msgs::Point> points_catenary_;
	geometry_msgs::Point p_reel_;
	
	int count = 0; 
    for (auto nt_:ct_) {
		p_reel_ = getReelNode(*nt_);
		points_catenary_.clear();

		CatenarySolver cS_;
		cS_.setMaxNumIterations(100);
		
		double x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_, len_cat_;
		x_ugv_ = p_reel_.x; 
		y_ugv_ = p_reel_.y; 
		z_ugv_ = p_reel_.z; 
		x_uav_ = nt_->point_uav.x*step; 
		y_uav_ = nt_->point_uav.y*step; 
		z_uav_ = nt_->point_uav.z*step; 
		len_cat_ = nt_->length_cat;
		// printf("Values to Compute Catenary: ugv[%f %f %f]  uav[%f %f %f] len_cat[%f]\n",x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_, len_cat_);
		cS_.solve(x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_, len_cat_, points_catenary_);

		int id_ = nt_->id;
		
		if (count%2 == 0)
			c_color3 = 0.5;
		else
			c_color3 = 0.0;
		string_marker = std::to_string(count);
		ns_marker = "catenary_"+ string_marker;
	
		catenaryMarker.markers.clear();
		catenaryMarker.markers.resize(points_catenary_.size());

		for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
			c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
			c_color2 = ((double)i / (double)points_catenary_.size())*0.5;

			catenaryMarker.markers[i].header.frame_id = frame_id;
			catenaryMarker.markers[i].header.stamp = ros::Time::now();
			catenaryMarker.markers[i].ns = ns_marker;
			catenaryMarker.markers[i].id = id_ + i*10.0;
			catenaryMarker.markers[i].action = visualization_msgs::Marker::ADD;
			catenaryMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
			catenaryMarker.markers[i].lifetime = ros::Duration(180);
			catenaryMarker.markers[i].pose.position.x = points_catenary_[i].x; 
			catenaryMarker.markers[i].pose.position.y = points_catenary_[i].y; 
			catenaryMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
			catenaryMarker.markers[i].pose.orientation.x = 0.0;
			catenaryMarker.markers[i].pose.orientation.y = 0.0;
			catenaryMarker.markers[i].pose.orientation.z = 0.0;
			catenaryMarker.markers[i].pose.orientation.w = 1.0;
			catenaryMarker.markers[i].scale.x = 0.06;
			catenaryMarker.markers[i].scale.y = 0.06;
			catenaryMarker.markers[i].scale.z = 0.06;
			catenaryMarker.markers[i].color.r = 1.0 - c_color1;
			catenaryMarker.markers[i].color.g = c_color2;
			catenaryMarker.markers[i].color.b = c_color3;
			catenaryMarker.markers[i].color.a = 1.0; 
		}
	count++;
	catenary_marker_pub_.publish(catenaryMarker);
	}	
}

void RRTGraphMarkers::getAllCatenaryMarker(std::list<RRTNode*> nodes_tree_, ros::Publisher all_catenary_marker_pub_)
{
	std::string string_marker;
    std::string ns_marker;

	double c_color1, c_color2, c_color3;
	
	int count = 0; 

	for (auto nt_:nodes_tree_) {		
		std::vector<geometry_msgs::Point> points_catenary_;
		geometry_msgs::Point p_reel_, p_uav_;

		CatenarySolver cSolver_;
		cSolver_.setMaxNumIterations(100);

		p_reel_ = getReelNode(*nt_);

		p_uav_.x = nt_->point_uav.x*step; 
		p_uav_.y = nt_->point_uav.y*step; 
		p_uav_.z = nt_->point_uav.z*step;
		double l_cat_ = nt_->length_cat;

		cSolver_.solve(p_reel_.x, p_reel_.y, p_reel_.z, p_uav_.x, p_uav_.y, p_uav_.z, l_cat_, points_catenary_);

		int id_ = nt_->id;


		if (count%2 == 0)
			c_color3 = 0.5;
		else
			c_color3 = 0.0;

		string_marker = std::to_string(count);
		ns_marker = "catenary_"+ string_marker;
	
		allCatenaryMarker.markers.clear();
		allCatenaryMarker.markers.resize(points_catenary_.size());

		for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
			c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
			c_color2 = ((double)i / (double)points_catenary_.size())*0.5;

			allCatenaryMarker.markers[i].header.frame_id = frame_id;
			allCatenaryMarker.markers[i].header.stamp = ros::Time::now();
			allCatenaryMarker.markers[i].ns = ns_marker;
			allCatenaryMarker.markers[i].id = id_ + i*10.0;
			allCatenaryMarker.markers[i].action = visualization_msgs::Marker::ADD;
			allCatenaryMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
			allCatenaryMarker.markers[i].lifetime = ros::Duration(180);
			allCatenaryMarker.markers[i].pose.position.x = points_catenary_[i].x; 
			allCatenaryMarker.markers[i].pose.position.y = points_catenary_[i].y; 
			allCatenaryMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
			allCatenaryMarker.markers[i].pose.orientation.x = 0.0;
			allCatenaryMarker.markers[i].pose.orientation.y = 0.0;
			allCatenaryMarker.markers[i].pose.orientation.z = 0.0;
			allCatenaryMarker.markers[i].pose.orientation.w = 1.0;
			allCatenaryMarker.markers[i].scale.x = 0.05;
			allCatenaryMarker.markers[i].scale.y = 0.05;
			allCatenaryMarker.markers[i].scale.z = 0.05;
			allCatenaryMarker.markers[i].color.r = 1.0 - c_color1;
			allCatenaryMarker.markers[i].color.g = c_color2;
			allCatenaryMarker.markers[i].color.b = c_color3;
			allCatenaryMarker.markers[i].color.a = 1.0; 
		}
	count++;
	all_catenary_marker_pub_.publish(allCatenaryMarker);
		
	}
}

void RRTGraphMarkers::goalPointMarker(geometry_msgs::Vector3 final_position_, ros::Publisher goal_point_pub_)
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = frame_id;
	marker_.header.stamp = ros::Time();
	marker_.ns = "goal_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(0);
	marker_.pose.position.x = final_position_.x;
	marker_.pose.position.y = final_position_.y;
	marker_.pose.position.z = final_position_.z;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 0.4;
	marker_.scale.y = 0.4;
	marker_.scale.z = 0.4;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.color.a = 1.0; 
	
	goal_point_pub_.publish(marker_);
}

void RRTGraphMarkers::reelPointMarker1(geometry_msgs::Point p_, ros::Publisher reel1_point_pub_)
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = frame_id;
	marker_.header.stamp = ros::Time();
	marker_.ns = "reel1_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::CUBE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(40);
	marker_.pose.position.x = p_.x;
	marker_.pose.position.y = p_.y;
	marker_.pose.position.z = p_.z;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 0.06;
	marker_.scale.y = 0.06;
	marker_.scale.z = 0.06;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.color.a = 1.0; 
	
	reel1_point_pub_.publish(marker_);
}

void RRTGraphMarkers::reelPointMarker2(geometry_msgs::Point p_, ros::Publisher reel2_point_pub_)
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = frame_id;
	marker_.header.stamp = ros::Time();
	marker_.ns = "reel2_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::CUBE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(40);
	marker_.pose.position.x = p_.x;
	marker_.pose.position.y = p_.y;
	marker_.pose.position.z = p_.z;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 0.06;
	marker_.scale.y = 0.06;
	marker_.scale.z = 0.06;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.color.a = 1.0; 
	
	reel2_point_pub_.publish(marker_);
}

void RRTGraphMarkers::randNodeMarker(RRTNode rn_, ros::Publisher rand_point_pub_)
{
	visualization_msgs::MarkerArray marker_rand;
	marker_rand.markers.resize(2);
	marker_rand.markers[0].header.frame_id = frame_id;
	marker_rand.markers[0].header.stamp = ros::Time();
	marker_rand.markers[0].ns = "rand_uav_point";
	marker_rand.markers[0].id = 0;
	marker_rand.markers[0].type = visualization_msgs::Marker::SPHERE;
	marker_rand.markers[0].action = visualization_msgs::Marker::ADD;
	marker_rand.markers[0].lifetime = ros::Duration(20);
	marker_rand.markers[0].pose.position.x = rn_.point.x*step;
	marker_rand.markers[0].pose.position.y = rn_.point.y*step;
	marker_rand.markers[0].pose.position.z = rn_.point.z*step+0.1;
	marker_rand.markers[0].pose.orientation.x = 0.0;
	marker_rand.markers[0].pose.orientation.y = 0.0;
	marker_rand.markers[0].pose.orientation.z = 0.0;
	marker_rand.markers[0].pose.orientation.w = 1.0;
	marker_rand.markers[0].scale.x = 0.4;
	marker_rand.markers[0].scale.y = 0.4;
	marker_rand.markers[0].scale.z = 0.4;
	marker_rand.markers[0].color.r = 0.0;
	marker_rand.markers[0].color.g = 1.0;
	marker_rand.markers[0].color.b = 0.0;
	marker_rand.markers[0].color.a = 1.0; 

	marker_rand.markers[1].header.frame_id = frame_id;
	marker_rand.markers[1].header.stamp = ros::Time();
	marker_rand.markers[1].ns = "rand_uav_point";
	marker_rand.markers[1].id = 1;
	marker_rand.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker_rand.markers[1].action = visualization_msgs::Marker::ADD;
	marker_rand.markers[1].lifetime = ros::Duration(20);
	marker_rand.markers[1].pose.position.x = rn_.point_uav.x*step;
	marker_rand.markers[1].pose.position.y = rn_.point_uav.y*step;
	marker_rand.markers[1].pose.position.z = rn_.point_uav.z*step;
	marker_rand.markers[1].pose.orientation.x = 0.0;
	marker_rand.markers[1].pose.orientation.y = 0.0;
	marker_rand.markers[1].pose.orientation.z = 0.0;
	marker_rand.markers[1].pose.orientation.w = 1.0;
	marker_rand.markers[1].scale.x = 0.4;
	marker_rand.markers[1].scale.y = 0.4;
	marker_rand.markers[1].scale.z = 0.4;
	marker_rand.markers[1].color.r = 0.4;
	marker_rand.markers[1].color.g = 1.0;
	marker_rand.markers[1].color.b = 0.4;
	marker_rand.markers[1].color.a = 1.0; 
	
	rand_point_pub_.publish(marker_rand);
}

void RRTGraphMarkers::newNodeMarker(RRTNode rn_, ros::Publisher new_point_pub_)
{
	visualization_msgs::MarkerArray marker_new;
	marker_new.markers.resize(2);
	marker_new.markers[0].header.frame_id = frame_id;
	marker_new.markers[0].header.stamp = ros::Time();
	marker_new.markers[0].ns = "only_new_node_ugv_point";
	marker_new.markers[0].id = 0;
	marker_new.markers[0].type = visualization_msgs::Marker::CUBE;
	marker_new.markers[0].action = visualization_msgs::Marker::ADD;
	marker_new.markers[0].lifetime = ros::Duration(40);
	marker_new.markers[0].pose.position.x = rn_.point.x*step;
	marker_new.markers[0].pose.position.y = rn_.point.y*step;
	marker_new.markers[0].pose.position.z = rn_.point.z*step + 0.15;
	marker_new.markers[0].pose.orientation.x = rn_.rot_ugv.x;
	marker_new.markers[0].pose.orientation.y = rn_.rot_ugv.y;
	marker_new.markers[0].pose.orientation.z = rn_.rot_ugv.z;
	marker_new.markers[0].pose.orientation.w = rn_.rot_ugv.w;
	marker_new.markers[0].scale.x = 0.8;
	marker_new.markers[0].scale.y = 0.12;
	marker_new.markers[0].scale.z = 0.10;
	marker_new.markers[0].color.r = 125.0/255.0;
	marker_new.markers[0].color.g = 125.0/255.0;
	marker_new.markers[0].color.b = 125.0/255.0;
	marker_new.markers[0].color.a = 1.0; 

	marker_new.markers[1].header.frame_id = frame_id;
	marker_new.markers[1].header.stamp = ros::Time();
	marker_new.markers[1].ns = "only_new_node_uav_point";
	marker_new.markers[1].id = 1;
	marker_new.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker_new.markers[1].action = visualization_msgs::Marker::ADD;
	marker_new.markers[1].lifetime = ros::Duration(40);
	marker_new.markers[1].pose.position.x = rn_.point_uav.x*step;
	marker_new.markers[1].pose.position.y = rn_.point_uav.y*step;
	marker_new.markers[1].pose.position.z = rn_.point_uav.z*step;
	marker_new.markers[1].pose.orientation.x = 0.0;
	marker_new.markers[1].pose.orientation.y = 0.0;
	marker_new.markers[1].pose.orientation.z = 0.0;
	marker_new.markers[1].pose.orientation.w = 1.0;
	marker_new.markers[1].scale.x = 0.2;
	marker_new.markers[1].scale.y = 0.2;
	marker_new.markers[1].scale.z = 0.2;
	marker_new.markers[1].color.r = 1.0;
	marker_new.markers[1].color.g = 0.4;
	marker_new.markers[1].color.b = 0.4;
	marker_new.markers[1].color.a = 1.0; 
	
	new_point_pub_.publish(marker_new);
}

void RRTGraphMarkers::nearestNodeMarker(RRTNode rn_, ros::Publisher nearest_point_pub_)
{
	visualization_msgs::MarkerArray marker_nearest;
	marker_nearest.markers.resize(2);
	marker_nearest.markers[0].header.frame_id = frame_id;
	marker_nearest.markers[0].header.stamp = ros::Time();
	marker_nearest.markers[0].ns = "nearest_uav_point";
	marker_nearest.markers[0].id = 0;
	marker_nearest.markers[0].type = visualization_msgs::Marker::CYLINDER;
	marker_nearest.markers[0].action = visualization_msgs::Marker::ADD;
	marker_nearest.markers[0].lifetime = ros::Duration(20);
	marker_nearest.markers[0].pose.position.x = rn_.point.x*step;
	marker_nearest.markers[0].pose.position.y = rn_.point.y*step;
	marker_nearest.markers[0].pose.position.z = rn_.point.z*step+0.1;
	marker_nearest.markers[0].pose.orientation.x = 0.0;
	marker_nearest.markers[0].pose.orientation.y = 0.0;
	marker_nearest.markers[0].pose.orientation.z = 0.0;
	marker_nearest.markers[0].pose.orientation.w = 1.0;
	marker_nearest.markers[0].scale.x = 0.08;
	marker_nearest.markers[0].scale.y = 0.08;
	marker_nearest.markers[0].scale.z = 0.28;
	marker_nearest.markers[0].color.r = 0.2;
	marker_nearest.markers[0].color.g = 1.0;
	marker_nearest.markers[0].color.b = 0.2;
	marker_nearest.markers[0].color.a = 1.0; 

	marker_nearest.markers[1].header.frame_id = frame_id;
	marker_nearest.markers[1].header.stamp = ros::Time();
	marker_nearest.markers[1].ns = "nearest_uav_point";
	marker_nearest.markers[1].id = 1;
	marker_nearest.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker_nearest.markers[1].action = visualization_msgs::Marker::ADD;
	marker_nearest.markers[1].lifetime = ros::Duration(20);
	marker_nearest.markers[1].pose.position.x = rn_.point_uav.x*step;
	marker_nearest.markers[1].pose.position.y = rn_.point_uav.y*step;
	marker_nearest.markers[1].pose.position.z = rn_.point_uav.z*step;
	marker_nearest.markers[1].pose.orientation.x = 0.0;
	marker_nearest.markers[1].pose.orientation.y = 0.0;
	marker_nearest.markers[1].pose.orientation.z = 0.0;
	marker_nearest.markers[1].pose.orientation.w = 1.0;
	marker_nearest.markers[1].scale.x = 0.1;
	marker_nearest.markers[1].scale.y = 0.1;
	marker_nearest.markers[1].scale.z = 0.1;
	marker_nearest.markers[1].color.r = 0.2;
	marker_nearest.markers[1].color.g = 1.0;
	marker_nearest.markers[1].color.b = 0.2;
	marker_nearest.markers[1].color.a = 1.0; 
	
	nearest_point_pub_.publish(marker_nearest);
}

void RRTGraphMarkers::getPointsObsMarker(std::vector<geometry_msgs::Point> points_catenary_, ros::Publisher points_marker_pub_){
	std::string string_marker;
    std::string ns_marker;

	double c_color1, c_color2, c_color3;
	visualization_msgs::MarkerArray pointsMarker;
	
	pointsMarker.markers.clear();
	pointsMarker.markers.resize(points_catenary_.size());

	for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
		c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
		c_color2 = ((double)i / (double)points_catenary_.size())*0.5;
		if (i%2 == 0)
		c_color3 = 0.5;
		else
		c_color3 = 0.0;
		pointsMarker.markers[i].header.frame_id = frame_id;
		pointsMarker.markers[i].header.stamp = ros::Time::now();
		pointsMarker.markers[i].ns = "points_marker";
		pointsMarker.markers[i].id = 1 + i*10.0;
		pointsMarker.markers[i].action = visualization_msgs::Marker::ADD;
		pointsMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
		pointsMarker.markers[i].lifetime = ros::Duration(180);
		pointsMarker.markers[i].pose.position.x = points_catenary_[i].x; 
		pointsMarker.markers[i].pose.position.y = points_catenary_[i].y; 
		pointsMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
		pointsMarker.markers[i].pose.orientation.x = 0.0;
		pointsMarker.markers[i].pose.orientation.y = 0.0;
		pointsMarker.markers[i].pose.orientation.z = 0.0;
		pointsMarker.markers[i].pose.orientation.w = 1.0;
		pointsMarker.markers[i].scale.x = 0.06;
		pointsMarker.markers[i].scale.y = 0.06;
		pointsMarker.markers[i].scale.z = 0.06;
		pointsMarker.markers[i].color.r = 1.0 - c_color1;
		pointsMarker.markers[i].color.g = c_color2;
		pointsMarker.markers[i].color.b = c_color3;
		pointsMarker.markers[i].color.a = 1.0; 
	}
	points_marker_pub_.publish(pointsMarker);
}

void RRTGraphMarkers::clearMarkers(ros::Publisher tree_rrt_star_ugv_pub_, ros::Publisher tree_rrt_star_uav_pub_, ros::Publisher take_off_nodes_pub_, ros::Publisher lines_ugv_marker_pub_, ros::Publisher lines_uav_marker_pub_)
{
	auto size_ = pointTreeMarkerUGV.markers.size();
	pointTreeMarkerUGV.markers.clear();
	pointTreeMarkerUGV.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        pointTreeMarkerUGV.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    tree_rrt_star_ugv_pub_.publish(pointTreeMarkerUGV);

	pointTreeMarkerUAV.markers.clear();
	pointTreeMarkerUAV.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        pointTreeMarkerUAV.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    tree_rrt_star_uav_pub_.publish(pointTreeMarkerUAV);

	size_ = pointTakeOffMarker.markers.size();
	pointTakeOffMarker.markers.clear();
	pointTakeOffMarker.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        pointTakeOffMarker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    take_off_nodes_pub_.publish(pointTakeOffMarker);


	size_ = n_iter;
    lines_ugv_marker.markers.clear();
	lines_ugv_marker.markers.resize(size_);
	for (auto i = 0 ; i < size_; i++){
        lines_ugv_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    lines_ugv_marker_pub_.publish(lines_ugv_marker);

	size_ = n_iter;
    lines_uav_marker.markers.clear();
	lines_uav_marker.markers.resize(size_);
	for (auto i = 0 ; i < size_; i++){
        lines_uav_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    lines_uav_marker_pub_.publish(lines_uav_marker);
}

void RRTGraphMarkers::clearCatenaryMarker(ros::Publisher c_m_pub_)
{
	auto size_ = catenaryMarker.markers.size();
	catenaryMarker.markers.clear();
	catenaryMarker.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        catenaryMarker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    c_m_pub_.publish(catenaryMarker);
}

geometry_msgs::Point RRTGraphMarkers::getReelNode(const RRTNode node_)
{
	geometry_msgs::Point pos_reel;
	float yaw_ugv;

	yaw_ugv = getYawFromQuaternion(node_,false);
	double lengt_vec =  sqrt(pos_reel_ugv.x*pos_reel_ugv.x + pos_reel_ugv.y*pos_reel_ugv.y);
	pos_reel.x = node_.point.x*step + lengt_vec *cos(yaw_ugv); 
	pos_reel.y = node_.point.y*step + lengt_vec *sin(yaw_ugv);
	pos_reel.z = node_.point.z*step + pos_reel_ugv.z ;

	// printf("pos_reel = [%f %f %f] yaw = %f\n",pos_reel.x,pos_reel.y,pos_reel.z,yaw_ugv);

	return pos_reel;
}

float RRTGraphMarkers::getYawFromQuaternion(RRTNode n_, bool is_uav_)
{
	double r_, p_, y_;

	if (!is_uav_){
		tf::Quaternion q(n_.rot_ugv.x, n_.rot_ugv.y, n_.rot_ugv.z, n_.rot_ugv.w);
		tf::Matrix3x3 M(q);	
		M.getRPY(r_, p_, y_);
	}
	else{
		tf::Quaternion q(n_.rot_uav.x, n_.rot_uav.y, n_.rot_uav.z, n_.rot_uav.w);
		tf::Matrix3x3 M(q);	
		M.getRPY(r_, p_, y_);
	}

	return y_;
}
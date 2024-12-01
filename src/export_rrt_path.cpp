#include "rrt_planners/export_rrt_path.hpp"

ExportPath::ExportPath()
{}

ExportPath::ExportPath(upo_actions::ExecutePathGoal p_, std::string path_)
{
    time_t ttime = time(0);
    tm *local_time = localtime(&ttime);
    
    string year_, month_, day_, hour_, min_, sec_;
    year_ = to_string(1900 + local_time->tm_year);
    month_ = to_string(1 + local_time->tm_mon);
    day_ = to_string(local_time->tm_mday);
    hour_ = to_string(local_time->tm_hour); 
    min_ = to_string(local_time->tm_min); 
    sec_ = to_string(1 + local_time->tm_sec);

	double d_to_interp_, d1_, d2_, cat_inter_;
	int r_;
	bool interpol_;
	vector<geometry_msgs::Vector3> vec_pose_ugv, vec_pose_uav;
	vector<geometry_msgs::Quaternion> vec_rot_ugv, vec_rot_uav;
	vector<float> vec_len_cat;
	geometry_msgs::Vector3 p_int_ugv_, p_int_uav_;

    vec_pose_ugv.clear();
	vec_rot_ugv.clear();
	vec_pose_uav.clear();
	vec_rot_uav.clear();
	vec_len_cat.clear();

	// Interpolate vector
	printf("vec_pose_path.size()=%lu , vec_len_cat.size()=%lu\n",p_.path.points.size(),p_.length_catenary.size());
	for (int i=0 ; i < p_.path.points.size()-1; i++){
		d_to_interp_ = 0.2;
		interpol_ = false;
		d1_ = sqrt ( pow(p_.path.points.at(i+1).transforms[0].translation.x - p_.path.points.at(i).transforms[0].translation.x,2) + 
                     pow(p_.path.points.at(i+1).transforms[0].translation.y - p_.path.points.at(i).transforms[0].translation.y,2) + 
                     pow(p_.path.points.at(i+1).transforms[0].translation.z - p_.path.points.at(i).transforms[0].translation.z,2) );
		d2_ = sqrt ( pow(p_.path.points.at(i+1).transforms[1].translation.x - p_.path.points.at(i).transforms[1].translation.x,2) + 
                     pow(p_.path.points.at(i+1).transforms[1].translation.y - p_.path.points.at(i).transforms[1].translation.y,2) + 
                     pow(p_.path.points.at(i+1).transforms[1].translation.z - p_.path.points.at(i).transforms[1].translation.z,2) );
		if (d1_ > d_to_interp_ ){
			if (d1_ >= d2_)
				r_ = floor(d1_/d_to_interp_);
			interpol_ = true;
		}
		if (d2_ > d_to_interp_ ){
			if (d2_ > d1_)
				r_ = floor(d2_/d_to_interp_);
			interpol_ = true;
		}
		if (interpol_){
			for (int j=0 ; j < r_ ; j++){
				p_int_ugv_.x = p_.path.points.at(i+1).transforms[0].translation.x + (j+1)*(p_.path.points.at(i+1).transforms[0].translation.x - p_.path.points.at(i).transforms[0].translation.x)/(r_+1);
				p_int_ugv_.y = p_.path.points.at(i+1).transforms[0].translation.y + (j+1)*(p_.path.points.at(i+1).transforms[0].translation.y - p_.path.points.at(i).transforms[0].translation.y)/(r_+1);
				p_int_ugv_.z = p_.path.points.at(i+1).transforms[0].translation.z + (j+1)*(p_.path.points.at(i+1).transforms[0].translation.z - p_.path.points.at(i).transforms[0].translation.z)/(r_+1);
				p_int_uav_.x = p_.path.points.at(i+1).transforms[1].translation.x + (j+1)*(p_.path.points.at(i+1).transforms[1].translation.x - p_.path.points.at(i).transforms[1].translation.x)/(r_+1);
				p_int_uav_.y = p_.path.points.at(i+1).transforms[1].translation.y + (j+1)*(p_.path.points.at(i+1).transforms[1].translation.y - p_.path.points.at(i).transforms[1].translation.y)/(r_+1);
				p_int_uav_.z = p_.path.points.at(i+1).transforms[1].translation.z + (j+1)*(p_.path.points.at(i+1).transforms[1].translation.z - p_.path.points.at(i).transforms[1].translation.z)/(r_+1);
				cat_inter_ = p_.length_catenary[i] + (j+1)*(p_.length_catenary[i+1] - p_.length_catenary[i])/(r_+1);
				vec_pose_ugv.push_back(p_int_ugv_);
				vec_rot_ugv.push_back(p_.path.points.at(i).transforms[0].rotation);
				vec_pose_uav.push_back(p_int_uav_);
				vec_rot_uav.push_back(p_.path.points.at(i).transforms[1].rotation);
				vec_len_cat.push_back(cat_inter_);
			}
		}else{
			vec_pose_ugv.push_back(p_.path.points.at(i).transforms[0].translation);
			vec_pose_uav.push_back(p_.path.points.at(i).transforms[1].translation);
			vec_rot_ugv.push_back(p_.path.points.at(i).transforms[0].rotation);
			vec_rot_uav.push_back(p_.path.points.at(i).transforms[1].rotation);
			vec_len_cat.push_back(p_.length_catenary[i]);
		}
	}
	
	printf("vec_pose_ugv.size()=%lu , vec_pose_uav.size()=%lu vec_len_cat=%lu\n", 
	vec_pose_ugv.size(),vec_pose_uav.size(),vec_len_cat.size());

    // Root of our file
    YAML::Node root_ugv, root_uav, root_tether;

    // Populate emitter
    YAML::Emitter emitter;

    // // Create a node listing some values
    root_ugv["marsupial_ugv"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    YAML::Node node = root_ugv["marsupial_ugv"];
    // YAML::Node node;
    // Write some values
    int size_ = vec_pose_ugv.size();

    node["header"] = "marsupial_ugv";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_;
    node["frame_id"] = "ugv";
    node["size"] = size_;
    
    for(int i=0 ; i < vec_pose_ugv.size(); i++){
		node["poses"+to_string(i)]["header"] = "ugv"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        node["poses"+to_string(i)]["frame_id"] = "ugv";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = vec_pose_ugv[i].x;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = vec_pose_ugv[i].y;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = vec_pose_ugv[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = vec_rot_ugv[i].x;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = vec_rot_ugv[i].y;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = vec_rot_ugv[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = vec_rot_ugv[i].w;
    }

    emitter << root_ugv;

    // // Create a node listing some values
    root_uav["marsupial_uav"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    node = root_uav["marsupial_uav"];
	size_ = vec_pose_uav.size();
    // YAML::Node node;
    // Write some values
    node["header"] = "marsupial_uav";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "base_link_uav";
    node["size"] = size_;
    for(int i=0 ; i < vec_pose_uav.size(); i++){
        node["poses"+to_string(i)]["header"] = "uav"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        node["poses"+to_string(i)]["frame_id"] = "uav";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = vec_pose_uav[i].x;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = vec_pose_uav[i].y;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = vec_pose_uav[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = vec_rot_uav[i].x;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = vec_rot_uav[i].y;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = vec_rot_uav[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = vec_rot_uav[i].w;
    }

    // Populate emitter
    emitter << root_uav;

	    // // Create a node listing some values
    root_tether["tether"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    node = root_tether["tether"];
	size_ = vec_len_cat.size();
    // YAML::Node node;
    // Write some values
    node["header"] = "tether";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "frame_tether";
    node["size"] = size_;
    for(int i=0 ; i < vec_len_cat.size(); i++){
        node["length"+to_string(i)]["header"] = "tether"+to_string(i);
        node["length"+to_string(i)]["seq"] = i;
        node["length"+to_string(i)]["frame_id"] = "tether_length";  
        node["length"+to_string(i)]["length"] = vec_len_cat[i];
    }

    // Populate emitter
    emitter << root_tether;

    // Write to file
    std::ofstream fout;
	fout.open(path_+"rrt_path_"+year_+"_"+month_+"_"+day_+"_"+hour_+min_+sec_ +".yaml", std::ofstream::app);
    fout << emitter.c_str();

	std::cout << "Saved Path Optimized" << std::endl << std::endl;
}

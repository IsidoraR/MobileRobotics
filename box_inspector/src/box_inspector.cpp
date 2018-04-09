//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> observed_models_wrt_world, 
       vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  //WRITE ME!!!
  //ROS_WARN("NEED TO WRITE update_inspection() ");
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();

  //THIS  IS WRONG...but an example of how to get models from image and sort into category vectors
  //for (int i=0;i<num_parts_seen;i++) {
  //   orphan_models_wrt_world.push_back(box_inspector_image_.models[i]);
  //}

	int n_observed_models = observed_models_wrt_world.size();
	int n_desired_models = desired_models_wrt_world.size();
	ROS_WARN(" number of observed models = %d", n_observed_models);
	int num_gear_part = 0;
	int num_desired_gear_part = 0;
	int num_piston_part = 0;
	int num_desired_piston_part = 0;
	int num_gasket_part = 0;
	int num_desired_gasket_part = 0;
	int num_disk_part = 0;
	int num_desired_disk_part = 0;
	int num_pulley_part = 0;
	int num_desired_pulley_part = 0;
	for(int i=0; i<n_observed_models; i++){
		int num_match = 0;
		int pos_match = 0;
		int ort_match = 0;
		int j = 0;
		double pos_x_diff = 0;
		double pos_y_diff = 0;
		double pos_z_diff = 0;
		double ort_x_diff = 0;
		double ort_y_diff = 0;
		double ort_z_diff = 0;
		double ort_w_diff = 0;
		if(observed_models_wrt_world[i].type == "gear_part"){
			num_gear_part++;
		}
		else if(observed_models_wrt_world[i].type == "piston_rod_part"){
			num_piston_part++;
		}
		else if(observed_models_wrt_world[i].type == "gasket_part"){
			num_gasket_part++;
		}
		else if(observed_models_wrt_world[i].type == "disk_part"){
			num_disk_part++;
		}
		else if(observed_models_wrt_world[i].type == "pulley_part"){
			num_pulley_part++;
		}
		while((num_match==0) && (j<n_desired_models)){
			if(observed_models_wrt_world[i].type == desired_models_wrt_world[j].type){
				num_match=1;
				pos_x_diff = abs(desired_models_wrt_world[j].pose.position.x - observed_models_wrt_world[i].pose.position.x);
				pos_y_diff = abs(desired_models_wrt_world[j].pose.position.y - observed_models_wrt_world[i].pose.position.y);
				pos_z_diff = abs(desired_models_wrt_world[j].pose.position.z - observed_models_wrt_world[i].pose.position.z);
				ort_x_diff = abs(desired_models_wrt_world[j].pose.orientation.x - observed_models_wrt_world[i].pose.orientation.x);
				ort_y_diff = abs(desired_models_wrt_world[j].pose.orientation.y - observed_models_wrt_world[i].pose.orientation.y);
				ort_z_diff = abs(desired_models_wrt_world[j].pose.orientation.z - observed_models_wrt_world[i].pose.orientation.z);
				ort_w_diff = abs(desired_models_wrt_world[j].pose.orientation.w - observed_models_wrt_world[i].pose.orientation.w);
				if((pos_x_diff <= 3)&&(pos_y_diff <= 3)&&(pos_z_diff <= 3)){
					pos_match = 1;
				}
				if((ort_x_diff <= 0.1)&&(ort_y_diff <= 0.1)&&(ort_z_diff <= 0.1)&&(ort_w_diff <= 0.1)){
					ort_match = 1;
				}
			}
			j++;
		}
		if((num_match==1)&&(pos_match==1)&&(ort_match==1)){
			satisfied_models_wrt_world.push_back(observed_models_wrt_world[i]);	
		}
		//else if(num_match==0){
		//	orphan_models_wrt_world.push_back(observed_models_wrt_world[i]);

		//}
		else if((num_match==1)&&((pos_match==0)||(ort_match==0))){
			misplaced_models_actual_coords_wrt_world.push_back(observed_models_wrt_world[i]);
			misplaced_models_desired_coords_wrt_world.push_back(desired_models_wrt_world[j-1]);

		}
		//ROS_INFO_STREAM("desired part poses w/rt world: " << desired_models_wrt_world[i] << endl);
	}
	for(int i=0; i<n_desired_models; i++){
		//int part_match = 0;
		//int j = 0;
		if(desired_models_wrt_world[i].type == "gear_part"){
			num_desired_gear_part++;
			if(num_desired_gear_part > num_gear_part)
				missing_models_wrt_world.push_back(desired_models_wrt_world[i]);
		}
		else if(desired_models_wrt_world[i].type == "piston_rod_part"){
			num_desired_piston_part++;
			if(num_desired_piston_part > num_piston_part)
				missing_models_wrt_world.push_back(desired_models_wrt_world[i]);
		}
		else if(desired_models_wrt_world[i].type == "gasket_part"){
			num_desired_gasket_part++;
			if(num_desired_gasket_part > num_gasket_part)
				missing_models_wrt_world.push_back(desired_models_wrt_world[i]);
		}
		else if(desired_models_wrt_world[i].type == "disk_part"){
			num_desired_disk_part++;
			if(num_desired_disk_part > num_disk_part)
				missing_models_wrt_world.push_back(desired_models_wrt_world[i]);
		}
		else if(desired_models_wrt_world[i].type == "pulley_part"){
			num_desired_pulley_part++;
			if(num_desired_pulley_part > num_pulley_part)
				missing_models_wrt_world.push_back(desired_models_wrt_world[i]);
		}
	}
	
	num_gear_part = 0;
	num_piston_part = 0;
	num_gasket_part = 0;
	num_disk_part = 0;
	num_pulley_part = 0;
	for(int i=0; i<n_observed_models; i++){
		//int part_match = 0;
		//int j = 0;
		if(observed_models_wrt_world[i].type == "gear_part"){
			num_gear_part++;
			if(num_gear_part > num_desired_gear_part)
				orphan_models_wrt_world.push_back(observed_models_wrt_world[i]);
		}
		else if(observed_models_wrt_world[i].type == "piston_rod_part"){
			num_piston_part++;
			if(num_piston_part > num_desired_piston_part)
				orphan_models_wrt_world.push_back(observed_models_wrt_world[i]);
		}
		else if(observed_models_wrt_world[i].type == "gasket_part"){
			num_gasket_part++;
			if(num_gasket_part > num_desired_gasket_part)
				orphan_models_wrt_world.push_back(observed_models_wrt_world[i]); 
		}
		else if(observed_models_wrt_world[i].type == "disk_part"){
			num_disk_part++;
			if(num_disk_part > num_desired_disk_part)
				orphan_models_wrt_world.push_back(observed_models_wrt_world[i]);
		}
		else if(observed_models_wrt_world[i].type == "pulley_part"){
			num_pulley_part++;
			if(num_pulley_part > num_desired_pulley_part)
				orphan_models_wrt_world.push_back(observed_models_wrt_world[i]);
		}
	}
  

}

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/

#include "../include/underwater_inspection/map_carving.h"
#include <tf/transform_datatypes.h>

#include <string>

namespace map_carving_ns {

MapCarving::MapCarving() {
}

bool MapCarving::initialize() {

  ROS_INFO("[map_carving] Node launched");
  if (!readParameters()) {
    ROS_ERROR("readParameters failed!");
    return false;
  }

  ros::NodeHandle nh("");

  // Initialize subscribers
  map_sub_ = nh.subscribe(map_topic_, 1, &MapCarving::collisonMapCallback, this);
  odometry_sub_ = nh.subscribe(odometry_topic_, 1, &MapCarving::odometryCallback, this);
  viewpoint_sub_ = nh.subscribe(viewpoint_topic_, 1, &MapCarving::viewpointCallback, this);

  // Initialize publishers
  viewpoint_map_pub_ = nh.advertise<inspection_planner_msgs::ViewpointList>(viewpoint_map_topic_, 2);
  
  // Initialize tf listener
  tf_listener_ = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

  ROS_INFO("[map_carving][initialize] Successfully launched map_carving node");
  std::cout << "[map_carving][initialize] Successfully launched map_carving node" << std::endl;
  
  return true;
}

bool MapCarving::readParameters() {

  ros::NodeHandle nh("~");

  if (!nh.getParam("viewpoint_map_topic_", viewpoint_map_topic_)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: viewpoint_map_topic_");
    return false;
  }
  if (!nh.getParam("odometry_topic_", odometry_topic_)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: odometry_topic_");
    return false;
  }
  if (!nh.getParam("viewpoint_topic_", viewpoint_topic_)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: viewpoint_topic_");
    return false;
  }
  if (!nh.getParam("map_topic_", map_topic_)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: map_topic_");
    return false;
  }
  if (!nh.getParam("ravenLength", ravenLength)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: ravenLength");
    return false;
  }
  if (!nh.getParam("ravenWidth", ravenWidth)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: ravenWidth");
    return false;
  }
  if (!nh.getParam("ravenHeight", ravenHeight)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: ravenHeight");
    return false;
  }
  if (!nh.getParam("ravenSphereRad", ravenSphereRad)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: ravenSphereRad");
    return false;
  }
  if (!nh.getParam("unknownThresh", unknownThresh)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: unknownThresh");
    return false;
  }
  if (!nh.getParam("freeThresh", freeThresh)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: freeThresh");
    return false;
  }
  if (!nh.getParam("occThresh", occThresh)) {
    ROS_ERROR("Node: map_carving. Cannot read parameter: occThresh");
    return false;
  }
  return true;
}

void MapCarving::viewpointCallback(const inspection_planner_msgs::ViewpointList::ConstPtr &viewpointlist_msg) {
  // ROS_INFO("[map_carving][viewpointCallback] viewpoint received");
  // std::cout << "[map_carving][viewpointCallback] viewpoint received" << std::endl;
  inspection_planner_msgs::ViewpointList viewpoints_map;
  for (auto& viewpoint : viewpointlist_msg->viewpoints){
    double unexplored_area = obtainViewpointInfo(viewpoint.x, viewpoint.y, viewpoint.z, viewpoint.yaw);
    if(unexplored_area<-0.99 || unexplored_area>-0.2){
      continue;
    }
    else{
      inspection_planner_msgs::Viewpoint viewpoint_modified;
      viewpoint_modified.x = viewpoint.x;
      viewpoint_modified.y = viewpoint.y;
      viewpoint_modified.z = viewpoint.z;
      viewpoint_modified.yaw = viewpoint.yaw;
      viewpoint_modified.cost = viewpoint.cost;
      viewpoint_modified.reward = unexplored_area;
      viewpoints_map.viewpoints.push_back(viewpoint_modified);
    }
  }
  publishViewpointInfo(viewpoints_map);
}

void MapCarving::collisonMapCallback(const octomap_msgs::Octomap::ConstPtr &map_msg) {
  octomap::AbstractOcTree* abstract_tree_ = octomap_msgs::msgToMap(*map_msg);
  // octomap::OcTree* octree = NULL;
  // ROS_INFO("[map_carving][collisonMapCallback] Received Octomap");
  // std::cout << "[map_carving][collisonMapCallback] Received Octomap" << std::endl;
  
  if (abstract_tree_){
    // octree = dynamic_cast<octomap::OcTree*>(abstract_tree_);
    tree_initialized_ = true;
    collision_tree_ = dynamic_cast<octomap::OcTree*>(abstract_tree_);
    node_count_ = collision_tree_->calcNumNodes();
    // std::cout << "[map_carving][collisonMapCallback] Tree node count: " << node_count_ << std::endl;
  }

}

void MapCarving::odometryCallback(const nav_msgs::Odometry &odom_msg) {
  odom_ = odom_msg.pose.pose;
  // ROS_INFO("[map_carving][odometryCallback] odom received");
  // std::cout << "[map_carving][odometryCallback] odom received" << std::endl;
  geo_quat_ = odom_.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat_.x, geo_quat_.y, geo_quat_.z, geo_quat_.w)).getRPY(raven_roll, raven_pitch, raven_yaw);
}

//////////////////////////////////
// Publishers

void MapCarving::publishViewpointInfo(inspection_planner_msgs::ViewpointList viewpointlist) {
  if(tree_initialized_){
    ROS_INFO("[map_carving][publishViewpointInfo] publishing viewpoint map info");
    std::cout << "[map_carving][publishViewpointInfo] publishing viewpoint map info" << std::endl;
    viewpoint_map_pub_.publish(viewpointlist);
  }
}


//////////////////////////////////
// Main functions

double MapCarving::obtainViewpointInfo(double vp_x, double vp_y, double vp_z, double vp_yaw){
  // ROS_INFO("[map_carving][obtainViewpointInfo] Map carving started.....");
  // std::cout << "[map_carving][obtainViewpointInfo] Map carving started....." << std::endl;
  int unknown_cells = 0;
  int free_cells = 0;
  int occ_cells = 0;
  double total_points = 0;
  
  if(tree_initialized_){
    double step_size = 0.25;

    octomap::point3d ray_origin_pose (vp_x, vp_y, vp_z);
    
    double nod_fov = M_PI_2;
    double nod_angle_inc = 0.05;
    double h_fov = 130 * (M_PI/180);
    // double v_fov = 20 * (M_PI/180);
    // int ray_skip = 10;
    double n_beams = 512;
    double angle_inc = h_fov / n_beams;
    
    for(double pitch_direction = -nod_fov/2; pitch_direction <= nod_fov/2; pitch_direction += nod_angle_inc){
      for(double yaw_direction = wrapAngle(vp_yaw-h_fov/2); yaw_direction <= wrapAngle(vp_yaw+h_fov/2); yaw_direction += angle_inc){
        for(double xx=0.1; xx<=20.0; xx+=0.1){
          total_points += 1;
          double d_x = xx;
          double d_y = 0.0;
          double d_z = 0.0;

          // Rotation about y-axis
          d_x = d_x * std::cos(pitch_direction) - d_z * std::sin(pitch_direction);
          d_z = d_x * std::sin(pitch_direction) + d_z * std::cos(pitch_direction);
          
          // Rotation about z-axis
          d_x = d_x * std::cos(yaw_direction) - d_y * std::sin(yaw_direction);
          d_y = d_y * std::sin(yaw_direction) + d_y * std::cos(yaw_direction);

          d_x += vp_x;
          d_y += vp_y;
          d_z += vp_z;

          float tree_occ_thresh = collision_tree_->getOccupancyThresLog();
          uint tree_depth = collision_tree_->getTreeDepth();
          octomap::OcTreeNode *oct_query_node = collision_tree_->search(d_x, d_y, d_z, 9);
        
          if(oct_query_node != NULL){
            float query_node_value = oct_query_node->getLogOdds();
            // std::cout<<"[MapCarving][obtainViewpointInfo] node value: "<<query_node_value_<<std::endl;
            if(query_node_value >= tree_occ_thresh){
              // std::cout<<"[MapCarving][obtainViewpointInfo] node value: "<<query_node_value_<<std::endl;
              occ_cells += 1;
            }
            else{
              // std::cout<<"[MapCarving][obtainViewpointInfo] node value free: "<<query_node_value_<<std::endl;
              free_cells += 1;
            }
          }
          else{
            unknown_cells += 1;
            // std::cout<<"[MapCarving][obtainViewpointInfo] query node: "<<oct_query_node_<<std::endl;
          }
        }
      }
    }
    

    // for(double pitch_direction = -nod_fov/2; pitch_direction <= nod_fov/2; pitch_direction += nod_angle_inc){
    //   for(double yaw_direction = wrapAngle(vp_yaw-h_fov/2); yaw_direction <= wrapAngle(vp_yaw+h_fov/2); yaw_direction += angle_inc){
        
    //     double d_x = 1.0;
    //     double d_y = 0.0;
    //     double d_z = 0.0;

    //     // Rotation about y-axis
    //     d_x = d_x * std::cos(pitch_direction) - d_z * std::sin(pitch_direction);
    //     d_z = d_x * std::sin(pitch_direction) + d_z * std::cos(pitch_direction);
        
    //     // Rotation about z-axis
    //     d_x = d_x * std::cos(yaw_direction) - d_y * std::sin(yaw_direction);
    //     d_y = d_y * std::sin(yaw_direction) + d_y * std::cos(yaw_direction);

    //     // d_x += vp_x;
    //     // d_y += vp_y;
    //     // d_z += vp_z;

    //     octomap::point3d ray_direction (d_x, d_y, d_z);
    //     octomap::point3d ray_end_pt;
    //     // std::cout<<ray_direction;
    //     collision_tree_->setResolution(0.64);
    //     bool ray_hit_success = collision_tree_->castRay(ray_origin_pose, ray_direction, ray_end_pt, false, 30.0);
    //     // std::cout<<pitch_direction<<", "<<yaw_direction<<", "<<ray_origin_pose<<", "<<ray_direction<<", "<<ray_end_pt<<", "<<ray_hit_success<<std::endl;
    //     if(ray_hit_success)
    //       occ_cells_ += 1;
    //   }
    // }
    
    // double unknown_dist_ = (double)unknown_cells_ / (double)total_points;
    // double free_dist_ = (double)free_cells_ / (double)total_points;
    // double occ_dist_ = (double)occ_cells_ / (double)total_points;
    // std::cout<<"[MapCarving][obtainViewpointInfo] Total Points: "<<total_points<<std::endl;
    // std::cout<<"[MapCarving][obtainViewpointInfo] Unknown space: "<<unknown_dist_<<"; Number: "<<unknown_cells_<<std::endl;
    // std::cout<<"[MapCarving][obtainViewpointInfo] Occupied cells: "<<occ_cells<<std::endl;
    // std::cout<<"[MapCarving][obtainViewpointInfo] Free space: "<<free_dist_<<"; Number: "<<free_cells_<<std::endl;
  }
  else{
    std::cout<<"[MapCarving][obtainViewpointInfo] Tree not initialized"<<std::endl;
  }
  double unexplored_area= -(double)(node_count_ - (occ_cells + free_cells))/(double)node_count_;
  return unexplored_area;
}

//////////////////////////////////
// Auxiliary functions

double MapCarving::wrapAngle(double angle){
  return std::fmod((angle + M_PI), (2 * M_PI)) - M_PI;
}
}

int main(int argc, char** argv){

  ROS_INFO("[map_carving][main] Creating node...");

  ros::init(argc, argv, "map_carving");

  map_carving_ns::MapCarving map_carving;

  // Setup
  bool result = map_carving.initialize();

  ros::Duration(0.5).sleep(); // sleep added to flush log buffer (not working?)

  // Loop indefinitely
  if(result) {
    ROS_INFO("[map_carving][main] Spinning indefinitely...");
    ros::spin();
  }

  ROS_INFO("[map_carving][main] Exiting...");

  return 0;
}


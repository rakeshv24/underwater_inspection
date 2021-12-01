#ifndef MAP_CARVING_H
#define MAP_CARVING_H

#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <inspection_planner_msgs/Viewpoint.h>
#include <inspection_planner_msgs/ViewpointList.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

namespace map_carving_ns {

class MapCarving;

}  // namespace sonar_processor_ns

class map_carving_ns::MapCarving {
 private:

  // ROS subscribers
  ros::Subscriber viewpoint_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber map_sub_;

  // ROS publishers
  ros::Publisher viewpoint_map_pub_;
  
  // String constants
  std::string map_topic_;
  std::string viewpoint_topic_;
  std::string viewpoint_map_topic_;
  std::string odometry_topic_;
  
  // Environment constants
  double ravenLength;
  double ravenWidth;
  double ravenHeight;
  double ravenSphereRad;
  double unknownThresh;
  double freeThresh;
  double occThresh;

  // Other variables
  geometry_msgs::Pose odom_;
  geometry_msgs::Quaternion geo_quat_;
  visualization_msgs::Marker debug_marker_;
  bool robot_pos_received_ = false;
  double raven_roll, raven_pitch, raven_yaw;
  double raven_diagonal_;
  std::unique_ptr<tf::TransformListener> tf_listener_;
  octomap::OcTree* collision_tree_;
  bool tree_initialized_ = false;

  // Functions
  bool readParameters();
  void viewpointCallback(const inspection_planner_msgs::ViewpointList::ConstPtr &viewpointlist_msg);
  void collisonMapCallback(const octomap_msgs::Octomap::ConstPtr &map_msg);
  void odometryCallback(const nav_msgs::Odometry &odom_msg);
  void publishViewpointInfo(inspection_planner_msgs::ViewpointList viewpointlist);
  int obtainViewpointInfo(double vp_x, double vp_y, double vp_z, double vp_yaw);
  double wrapAngle(double angle);
  
 public:
  explicit MapCarving();
  bool initialize();
  virtual ~MapCarving() = default;
};

#endif  // MAP_CARVING_H
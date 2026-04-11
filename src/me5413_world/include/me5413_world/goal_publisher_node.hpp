/* goal_publisher_node.hpp

 * Copyright (C) 2023 SS47816

 * Declarations for GoalPublisherNode class
 
 deprecated for 2526

**/

#ifndef GOAL_PUBLISHER_NODE_H_
#define GOAL_PUBLISHER_NODE_H_

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

namespace me5413_world 
{

class GoalPublisherNode
{
 public:
  GoalPublisherNode();
  virtual ~GoalPublisherNode() {};

 private:
  enum AutoSequenceState
  {
    IDLE = 0,
    WAITING_FOR_RESPAWN = 1,
    NAVIGATING_TO_FIRST = 2,
    NAVIGATING_TO_SECOND = 3,
    NAVIGATING_TO_INTERMEDIATE = 4,
    NAVIGATING_TO_PRE_FINAL = 5,
    NAVIGATING_TO_THIRD = 6,
  };

  void timerCallback(const ros::TimerEvent&);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void goalNameCallback(const std_msgs::String::ConstPtr& name);
  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  void boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers);
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& model_states);
  void respawnObjectsCallback(const std_msgs::Int16::ConstPtr& respawn_msg);
  void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
  
  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);
  geometry_msgs::PoseStamped getGoalPoseFromConfig(const std::string& name);
  std::pair<double, double> calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal);
  bool updateRobotPoseInMap();
  bool isPointOccupiedInMap(const geometry_msgs::Point& point) const;
  bool selectPreFinalGoal(geometry_msgs::PoseStamped* selected_goal, std::string* selected_label) const;
  void publishAutoGoal(const geometry_msgs::PoseStamped& goal, const std::string& label);
  double calculatePlanarDistance(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal) const;

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Publisher pub_goal_;
  ros::Publisher pub_unblock_;
  ros::Publisher pub_absolute_position_error_;
  ros::Publisher pub_absolute_heading_error_;
  ros::Publisher pub_relative_position_error_;
  ros::Publisher pub_relative_heading_error_;

  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_goal_name_;
  ros::Subscriber sub_goal_pose_;
  ros::Subscriber sub_box_markers_;
  ros::Subscriber sub_model_states_;
  ros::Subscriber sub_respawn_objects_;
  ros::Subscriber sub_occupancy_grid_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;

  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string goal_type_;

  geometry_msgs::Pose pose_world_robot_;
  geometry_msgs::Pose pose_world_goal_;
  geometry_msgs::Pose pose_map_robot_;
  geometry_msgs::Pose pose_map_goal_;
  std::vector<geometry_msgs::PoseStamped> box_poses_;
  geometry_msgs::PoseStamped auto_goal_1_;
  geometry_msgs::PoseStamped auto_goal_2_;
  geometry_msgs::PoseStamped auto_goal_intermediate_;
  geometry_msgs::PoseStamped auto_goal_pre_final_candidate_1_;
  geometry_msgs::PoseStamped auto_goal_pre_final_candidate_2_;
  geometry_msgs::PoseStamped auto_goal_pre_final_selected_;
  geometry_msgs::PoseStamped auto_goal_3_;
  nav_msgs::OccupancyGrid latest_map_;

  std_msgs::Float32 absolute_position_error_;
  std_msgs::Float32 absolute_heading_error_;
  std_msgs::Float32 relative_position_error_;
  std_msgs::Float32 relative_heading_error_;
  ros::Time auto_sequence_start_time_;
  AutoSequenceState auto_sequence_state_;
  double auto_goal_tolerance_;
  double auto_goal_heading_tolerance_deg_;
  bool respawn_models_ready_;
  bool has_cone_model_;
  bool has_random_cone_model_;
  int spawned_number_model_count_;
  bool has_latest_map_;
};

} // namespace me5413_world

#endif // GOAL_PUBLISHER_NODE_H_

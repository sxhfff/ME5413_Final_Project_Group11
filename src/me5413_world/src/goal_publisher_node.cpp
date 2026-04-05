/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses

 deprecated for 2526
 
**/

#include "me5413_world/goal_publisher_node.hpp"

namespace me5413_world 
{

GoalPublisherNode::GoalPublisherNode() : tf2_listener_(tf2_buffer_)
{
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  this->pub_absolute_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/position_error", 1);
  this->pub_absolute_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/heading_error", 1);
  this->pub_relative_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/position_error", 1);
  this->pub_relative_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/heading_error", 1);
  this->pub_cmd_unblock_ = nh_.advertise<std_msgs::Bool>("/cmd_unblock", 1);

  this->timer_ = nh_.createTimer(ros::Duration(0.2), &GoalPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &GoalPublisherNode::robotOdomCallback, this);
  this->sub_goal_name_ = nh_.subscribe("/rviz_panel/goal_name", 1, &GoalPublisherNode::goalNameCallback, this);
  this->sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalPublisherNode::goalPoseCallback, this);
  this->sub_box_markers_ = nh_.subscribe("/gazebo/ground_truth/box_markers", 1, &GoalPublisherNode::boxMarkersCallback, this);
  this->sub_planning_ready_ = nh_.subscribe("/planning_ready", 1, &GoalPublisherNode::planningReadyCallback, this);
  
  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";
  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;

  this->planning_ready_ = false;
  this->cone_open_sent_ = false;
  this->robot_pose_received_ = false;
  this->mission_stage_ = 0;
  this->goal_reached_tolerance_ = 0.5;
};

void GoalPublisherNode::timerCallback(const ros::TimerEvent&)
{
  if (!this->robot_pose_received_)
  {
    ROS_WARN_THROTTLE(5.0, "Waiting for /gazebo/ground_truth/state before computing mission progress.");
    return;
  }

  geometry_msgs::TransformStamped transform_map_world;
  if (!this->tryLookupMapToWorld(transform_map_world))
  {
    return;
  }

  tf2::doTransform(this->pose_world_robot_, this->pose_map_robot_, transform_map_world);


  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> error_absolute = calculatePoseError(this->pose_world_robot_, this->pose_world_goal_);
  // Calculate relative errors (wrt to map frame)
  const std::pair<double, double> error_relative = calculatePoseError(this->pose_map_robot_, this->pose_map_goal_);

  
  this->absolute_position_error_.data = error_absolute.first;
  this->absolute_heading_error_.data = error_absolute.second;
  this->relative_position_error_.data = error_relative.first;
  this->relative_heading_error_.data = error_relative.second;

  if (this->goal_type_ == "box")
  {
    this->absolute_heading_error_.data = 0.0;
    this->relative_heading_error_.data = 0.0;
  }

  // Publish errors
  this->pub_absolute_position_error_.publish(this->absolute_position_error_);
  this->pub_absolute_heading_error_.publish(this->absolute_heading_error_);
  this->pub_relative_position_error_.publish(this->relative_position_error_);
  this->pub_relative_heading_error_.publish(this->relative_heading_error_);

    // ===== mission logic =====
  if (!this->planning_ready_)
  {
    return;
  }

  const double current_error = this->relative_position_error_.data;

  ROS_INFO_STREAM("mission_stage = " << this->mission_stage_
              << ", current_error = " << current_error
              << ", tolerance = " << this->goal_reached_tolerance_);

  // 到达第一个点后：发清除 cone 命令，然后去第二个点
  if (this->mission_stage_ == 1 && current_error < this->goal_reached_tolerance_)
  {
    if (!this->cone_open_sent_)
    {
      std_msgs::Bool unblock_msg;
      unblock_msg.data = true;
      this->pub_cmd_unblock_.publish(unblock_msg);
      this->cone_open_sent_ = true;

      ROS_INFO_STREAM("Reached point 1. Sent cone unblock command.");
    }

    // 立刻去第二个点
    this->publishGoal(8.493532180786133, -2.0953121185302734,0.0);
    this->mission_stage_ = 2;

    ROS_INFO_STREAM("Mission stage 2 started.");
  }
  else if (this->mission_stage_ == 2 && current_error < this->goal_reached_tolerance_)
  {
    this->mission_stage_ = 3;
    ROS_INFO_STREAM("Mission completed: reached point 2.");
  }

};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;
  this->robot_pose_received_ = true;

  const tf2::Transform T_world_robot = convertPoseToTransform(this->pose_world_robot_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();
  
  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

void GoalPublisherNode::goalNameCallback(const std_msgs::String::ConstPtr& name)
{ 

    if (!this->planning_ready_)
  {
    ROS_WARN_STREAM("Planning is not ready yet. Please click spawn first.");
    return;
  }
  
  const std::string goal_name = name->data;
  const int end = goal_name.find_last_of("_");
  this->goal_type_ = goal_name.substr(1, end-1);
  const int goal_box_id = stoi(goal_name.substr(end+1, 1));

  geometry_msgs::PoseStamped P_world_goal;
  if (this->goal_type_ == "box")
  {
    if (box_poses_.empty())
    {
      ROS_ERROR_STREAM("Box poses unknown, please spawn boxes first!");
      return;
    }
    else if (goal_box_id >= box_poses_.size())
    {
      ROS_ERROR_STREAM("Box id is outside the available range, please select a smaller id!");
      return;
    }
    
    P_world_goal = box_poses_[goal_box_id - 1];
  }
  else
  {
    // Get the Pose of the goal in world frame
    P_world_goal = getGoalPoseFromConfig(goal_name);
  }

  this->pose_world_goal_ = P_world_goal.pose;
  // Get the Transform from world to map from the tf_listener
  geometry_msgs::TransformStamped transform_map_world;
  if (!this->tryLookupMapToWorld(transform_map_world))
  {
    return;
  }

  // Transform the goal pose to map frame
  geometry_msgs::PoseStamped P_map_goal;
  tf2::doTransform(P_world_goal, P_map_goal, transform_map_world);
  P_map_goal.header.stamp = ros::Time::now();
  P_map_goal.header.frame_id = map_frame_;

  // Transform the robot pose to map frame
  tf2::doTransform(this->pose_world_robot_, this->pose_map_robot_, transform_map_world);

  // Publish goal pose in map frame 
  if (this->goal_type_ != "box")
  {
    this->pub_goal_.publish(P_map_goal);
  }

  return;
};

void GoalPublisherNode::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
{
  this->pose_map_goal_ = goal_pose->pose;
}

tf2::Transform GoalPublisherNode::convertPoseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

void GoalPublisherNode::planningReadyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  this->planning_ready_ = msg->data;
  ROS_INFO_STREAM("planning_ready = " << this->planning_ready_);

  if (!this->planning_ready_)
  {
    this->cone_open_sent_ = false;
    this->mission_stage_ = 0;
    return;
  }

  // 只有在任务尚未开始时，才发第一个点
  if (this->mission_stage_ != 0)
  {
    return;
  }

  // Stage 1: 先去第一个点
  this->publishGoal(7.515092849731445, 0.5537302494049072, -M_PI/2);

  this->mission_stage_ = 1;
  this->cone_open_sent_ = false;

  ROS_INFO_STREAM("Mission stage 1 started.");
}

void GoalPublisherNode::boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers)
{
  this->box_poses_.clear();
  for (const auto& box : box_markers->markers)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = box.pose;
    this->box_poses_.emplace_back(pose);
  }

  return;
};

bool GoalPublisherNode::tryLookupMapToWorld(geometry_msgs::TransformStamped& transform_map_world)
{
  if (!this->tf2_buffer_.canTransform(this->map_frame_, this->world_frame_, ros::Time(0), ros::Duration(0.05)))
  {
    ROS_WARN_THROTTLE(
      5.0,
      "Waiting for TF transform from %s to %s. Set an initial pose in RViz if AMCL is running.",
      this->map_frame_.c_str(),
      this->world_frame_.c_str());
    return false;
  }

  try
  {
    transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
    return false;
  }
}

geometry_msgs::PoseStamped GoalPublisherNode::getGoalPoseFromConfig(const std::string& name)
{
  /** 
   * Get the Transform from goal to world from the file
   */

  double x, y, yaw;
  nh_.getParam("/me5413_world" + name + "/x", x);
  nh_.getParam("/me5413_world" + name + "/y", y);
  nh_.getParam("/me5413_world" + name + "/yaw", yaw);
  nh_.getParam("/me5413_world/frame_id", this->world_frame_);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  geometry_msgs::PoseStamped P_world_goal;
  P_world_goal.pose.position.x = x;
  P_world_goal.pose.position.y = y;
  P_world_goal.pose.orientation = tf2::toMsg(q);

  return P_world_goal;
};

void GoalPublisherNode::publishGoal(double x, double y, double yaw)
{
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  this->pub_goal_.publish(goal);

  ROS_INFO_STREAM("Published goal: x=" << x << ", y=" << y << ", yaw=" << yaw);
}

std::pair<double, double> GoalPublisherNode::calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal)
{
  // Positional Error
  const double position_error = std::sqrt(
    std::pow(pose_robot.position.x - pose_goal.position.x, 2) + 
    std::pow(pose_robot.position.y - pose_goal.position.y, 2)
  );

  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = (yaw_robot - yaw_goal)/M_PI*180.0;

  return std::pair<double, double>(position_error, heading_error);
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  me5413_world::GoalPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}

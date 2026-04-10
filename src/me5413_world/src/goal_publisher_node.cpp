/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses

 deprecated for 2526
 
**/

#include "me5413_world/goal_publisher_node.hpp"

namespace me5413_world 
{

namespace
{
constexpr int kMinNumberModelsAfterRespawn = 9;
}

GoalPublisherNode::GoalPublisherNode() : tf2_listener_(tf2_buffer_)
{
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  this->pub_unblock_ = nh_.advertise<std_msgs::Bool>("/cmd_unblock", 1);
  this->pub_absolute_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/position_error", 1);
  this->pub_absolute_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/heading_error", 1);
  this->pub_relative_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/position_error", 1);
  this->pub_relative_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/heading_error", 1);

  this->timer_ = nh_.createTimer(ros::Duration(0.2), &GoalPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &GoalPublisherNode::robotOdomCallback, this);
  this->sub_goal_name_ = nh_.subscribe("/rviz_panel/goal_name", 1, &GoalPublisherNode::goalNameCallback, this);
  this->sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalPublisherNode::goalPoseCallback, this);
  this->sub_box_markers_ = nh_.subscribe("/gazebo/ground_truth/box_markers", 1, &GoalPublisherNode::boxMarkersCallback, this);
  this->sub_model_states_ = nh_.subscribe("/gazebo/model_states", 1, &GoalPublisherNode::modelStatesCallback, this);
  this->sub_respawn_objects_ = nh_.subscribe("/rviz_panel/respawn_objects", 1, &GoalPublisherNode::respawnObjectsCallback, this);
  
  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";
  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;
  this->auto_sequence_state_ = IDLE;
  this->auto_goal_tolerance_ = 0.75;
  this->auto_goal_heading_tolerance_deg_ = 15.0;
  this->respawn_models_ready_ = false;
  this->has_cone_model_ = false;
  this->has_random_cone_model_ = false;
  this->spawned_number_model_count_ = 0;

  this->auto_goal_1_.header.frame_id = this->map_frame_;
  this->auto_goal_1_.pose.position.x = 1.7331511974334717;
  this->auto_goal_1_.pose.position.y = 7.418707370758057;
  this->auto_goal_1_.pose.position.z = 0.0;
  this->auto_goal_1_.pose.orientation.w = 1.0;

  this->auto_goal_2_.header.frame_id = this->map_frame_;
  this->auto_goal_2_.pose.position.x = 4.177798271179199;
  this->auto_goal_2_.pose.position.y = 40.7452507019043;
  this->auto_goal_2_.pose.position.z = 0.002017974853515625;
  this->auto_goal_2_.pose.orientation.z = 1.0;

  this->auto_goal_intermediate_.header.frame_id = this->map_frame_;
  this->auto_goal_intermediate_.pose.position.x = -15.925942420959473;
  this->auto_goal_intermediate_.pose.position.y = 40.745750427246094;
  this->auto_goal_intermediate_.pose.position.z = 0.0074310302734375;
  this->auto_goal_intermediate_.pose.orientation.z = -0.70710678;
  this->auto_goal_intermediate_.pose.orientation.w = 0.70710678;

  this->auto_goal_3_.header.frame_id = this->map_frame_;
  this->auto_goal_3_.pose.position.x = -7.284735679626465;
  this->auto_goal_3_.pose.position.y = 30.505300521850586;
  this->auto_goal_3_.pose.position.z = 0.001087188720703125;
  this->auto_goal_3_.pose.orientation.z = -0.70710678;
  this->auto_goal_3_.pose.orientation.w = 0.70710678;
};

void GoalPublisherNode::timerCallback(const ros::TimerEvent&)
{
  const bool has_map_pose = updateRobotPoseInMap();

  if (this->auto_sequence_state_ == WAITING_FOR_RESPAWN)
  {
    if (this->respawn_models_ready_)
    {
      publishAutoGoal(this->auto_goal_1_, "first");
      this->auto_sequence_state_ = NAVIGATING_TO_FIRST;
    }
  }
  else if (this->auto_sequence_state_ == NAVIGATING_TO_FIRST && has_map_pose)
  {
    const double position_error = calculatePlanarDistance(this->pose_map_robot_, this->auto_goal_1_.pose);
    const double heading_error_deg = std::abs(calculatePoseError(this->pose_map_robot_, this->auto_goal_1_.pose).second);
    if (position_error <= this->auto_goal_tolerance_ &&
        heading_error_deg <= this->auto_goal_heading_tolerance_deg_)
    {
      std_msgs::Bool unblock_msg;
      unblock_msg.data = true;
      this->pub_unblock_.publish(unblock_msg);
      ROS_INFO_STREAM("Reached first auto goal with heading aligned. Published obstacle removal command.");
      publishAutoGoal(this->auto_goal_2_, "second");
      this->auto_sequence_state_ = NAVIGATING_TO_SECOND;
    }
  }
  else if (this->auto_sequence_state_ == NAVIGATING_TO_SECOND && has_map_pose)
  {
    if (calculatePlanarDistance(this->pose_map_robot_, this->auto_goal_2_.pose) <= this->auto_goal_tolerance_)
    {
      publishAutoGoal(this->auto_goal_intermediate_, "intermediate");
      this->auto_sequence_state_ = NAVIGATING_TO_INTERMEDIATE;
    }
  }
  else if (this->auto_sequence_state_ == NAVIGATING_TO_INTERMEDIATE && has_map_pose)
  {
    if (calculatePlanarDistance(this->pose_map_robot_, this->auto_goal_intermediate_.pose) <= this->auto_goal_tolerance_)
    {
      publishAutoGoal(this->auto_goal_3_, "third");
      this->auto_sequence_state_ = NAVIGATING_TO_THIRD;
    }
  }
  else if (this->auto_sequence_state_ == NAVIGATING_TO_THIRD && has_map_pose)
  {
    if (calculatePlanarDistance(this->pose_map_robot_, this->auto_goal_3_.pose) <= this->auto_goal_tolerance_)
    {
      this->auto_sequence_state_ = IDLE;
      ROS_INFO_STREAM("Reached third auto goal. Auto sequence complete.");
    }
  }

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

  return;
};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

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
  try
  {
    transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
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

void GoalPublisherNode::respawnObjectsCallback(const std_msgs::Int16::ConstPtr& respawn_msg)
{
  if (respawn_msg->data == 1)
  {
    this->auto_sequence_start_time_ = ros::Time::now();
    this->auto_sequence_state_ = WAITING_FOR_RESPAWN;
    this->respawn_models_ready_ = false;
    ROS_INFO_STREAM("Respawn requested. Waiting until Gazebo reports the respawned objects are ready.");
  }
  else
  {
    this->auto_sequence_state_ = IDLE;
    this->respawn_models_ready_ = false;
  }
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

void GoalPublisherNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& model_states)
{
  this->has_cone_model_ = false;
  this->has_random_cone_model_ = false;
  this->spawned_number_model_count_ = 0;

  for (const auto& model_name : model_states->name)
  {
    if (model_name == "Construction Barrel")
    {
      this->has_cone_model_ = true;
    }
    else if (model_name == "Construction Cone")
    {
      this->has_random_cone_model_ = true;
    }
    else if (model_name.rfind("number", 0) == 0)
    {
      ++this->spawned_number_model_count_;
    }
  }

  this->respawn_models_ready_ =
    this->has_cone_model_ &&
    this->has_random_cone_model_ &&
    this->spawned_number_model_count_ >= kMinNumberModelsAfterRespawn;
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

bool GoalPublisherNode::updateRobotPoseInMap()
{
  geometry_msgs::TransformStamped transform_map_world;
  try
  {
    transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "%s", ex.what());
    return false;
  }

  tf2::doTransform(this->pose_world_robot_, this->pose_map_robot_, transform_map_world);
  return true;
}

void GoalPublisherNode::publishAutoGoal(const geometry_msgs::PoseStamped& goal, const std::string& label)
{
  geometry_msgs::PoseStamped goal_msg = goal;
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.header.frame_id = this->map_frame_;
  this->pub_goal_.publish(goal_msg);
  this->pose_map_goal_ = goal_msg.pose;
  ROS_INFO_STREAM("Published " << label << " auto goal at ("
                  << goal_msg.pose.position.x << ", "
                  << goal_msg.pose.position.y << ").");
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

double GoalPublisherNode::calculatePlanarDistance(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal) const
{
  return std::sqrt(
    std::pow(pose_robot.position.x - pose_goal.position.x, 2) +
    std::pow(pose_robot.position.y - pose_goal.position.y, 2)
  );
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  me5413_world::GoalPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}

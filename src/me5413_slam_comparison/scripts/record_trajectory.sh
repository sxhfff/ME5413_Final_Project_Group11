#!/bin/bash

rosbag record \
/front/scan \
/jackal_velocity_controller/odom \
/mid/points \
/tf \
/tf_static \
/gazebo/model_states \
-O ~/ME5413_Final_Project_Group11/src/me5413_slam_comparison/bags/slam_dataset_complete.bag


#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('publish_initial_pose_once')

    x = rospy.get_param('~x', 0.0)
    y = rospy.get_param('~y', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    rospy.sleep(2.0)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'

    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    q = quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.0685

    pub.publish(msg)
    rospy.loginfo(f"Published initial pose: x={x}, y={y}, yaw={yaw}")

    rospy.sleep(0.5)

if __name__ == '__main__':
    main()

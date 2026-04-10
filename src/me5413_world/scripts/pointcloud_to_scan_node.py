#!/usr/bin/env python3

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2


class PointCloudToScanNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/mid/points")
        self.output_topic = rospy.get_param("~output_topic", "/local_obstacle_scan")
        self.scan_frame = rospy.get_param("~scan_frame", "velodyne")

        self.min_height = rospy.get_param("~min_height", -0.45)
        self.max_height = rospy.get_param("~max_height", 0.35)
        self.min_range = rospy.get_param("~min_range", 0.6)
        self.max_range = rospy.get_param("~max_range", 12.0)
        self.robot_exclusion_radius = rospy.get_param("~robot_exclusion_radius", 0.55)

        self.angle_min = rospy.get_param("~angle_min", -math.pi)
        self.angle_max = rospy.get_param("~angle_max", math.pi)
        self.angle_increment = rospy.get_param("~angle_increment", math.radians(0.5))
        self.scan_time = rospy.get_param("~scan_time", 0.1)

        self.range_count = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.handle_cloud, queue_size=1)

    def handle_cloud(self, msg):
        ranges = [float("inf")] * self.range_count

        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < self.min_height or z > self.max_height:
                continue

            distance = math.hypot(x, y)
            if distance < self.min_range or distance > self.max_range:
                continue
            if distance < self.robot_exclusion_radius:
                continue

            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue

            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.range_count and distance < ranges[index]:
                ranges[index] = distance

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.scan_frame or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.angle_increment * self.range_count
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = ranges

        self.pub.publish(scan)


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_scan_node")
    PointCloudToScanNode()
    rospy.spin()

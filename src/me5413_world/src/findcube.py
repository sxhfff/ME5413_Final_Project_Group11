#!/usr/bin/env python
import rospy
import numpy as np
import math
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from sklearn.cluster import DBSCAN
import tf2_ros
import tf2_geometry_msgs

# =================== Adjustable Parameters ===================
# Default map file used when no ROS parameter is provided.
DEFAULT_MAP_YAML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "maps", "my_map.yaml")
)

# DBSCAN clustering parameters: 
# 'CLUSTER_EPS' is the maximum distance between two samples for them to be considered neighbors.
# 'CLUSTER_MIN_SAMPLES' is the minimum number of points required to form a cluster.
CLUSTER_EPS = 0.2        
CLUSTER_MIN_SAMPLES = 10 

# Frame definitions:
# 'DEFAULT_INPUT_FRAME' is only used when the scan message does not carry a frame id.
# 'OUTPUT_FRAME' is the target frame to which the point cloud data will be transformed.
DEFAULT_INPUT_FRAME = "tim551"
OUTPUT_FRAME = "map"         

# Visualization marker settings:
# 'MARKER_TOPIC' is the topic to which the cluster markers will be published.
# 'MARKER_SCALE' controls the size of the sphere markers representing cluster centers.
MARKER_TOPIC = "/cluster_markers_map"  
MARKER_SCALE = 0.4                     

# Valid bounding rectangle size range (in meters) for a cluster to be considered as a valid block.
VALID_RECT_MIN_SIZE = 0.6  
VALID_RECT_MAX_SIZE = 0.9  

# When fusing clusters with previously detected blocks, merge them if their centers are within this threshold.
MERGE_DISTANCE_THRESHOLD = 1.3 

# Fusion weights for blending the previously found block with the new cluster data.
FUSION_OLD_WEIGHT = 0.4
FUSION_NEW_WEIGHT = 0.6
# =================================================================

def _parse_simple_map_yaml(map_yaml_path):
    """
    Parse the small subset of map_server YAML fields used by this node.
    """
    data = {}
    with open(map_yaml_path, 'r') as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, value = line.split(':', 1)
            data[key.strip()] = value.strip()

    image_rel = data.get('image')
    resolution = float(data['resolution'])
    origin_text = data['origin'].strip()[1:-1]
    origin = [float(v.strip()) for v in origin_text.split(',')]

    if image_rel is None:
        raise ValueError("Missing 'image' field in map yaml: {}".format(map_yaml_path))

    image_path = image_rel if os.path.isabs(image_rel) else os.path.join(os.path.dirname(map_yaml_path), image_rel)
    return image_path, resolution, origin

def _read_map_image_size(image_path):
    """
    Read image dimensions without pulling in extra dependencies.
    Supports .pgm and .png map files.
    """
    ext = os.path.splitext(image_path)[1].lower()

    if ext == '.pgm':
        with open(image_path, 'rb') as f:
            magic = f.readline().strip()
            if magic not in [b'P2', b'P5']:
                raise ValueError("Unsupported PGM format: {}".format(magic))

            tokens = []
            while len(tokens) < 3:
                line = f.readline()
                if not line:
                    break
                line = line.split(b'#', 1)[0].strip()
                if not line:
                    continue
                tokens.extend(line.split())

            if len(tokens) < 3:
                raise ValueError("Invalid PGM header: {}".format(image_path))

            width = int(tokens[0])
            height = int(tokens[1])
            return width, height

    if ext == '.png':
        with open(image_path, 'rb') as f:
            header = f.read(24)
            if len(header) < 24 or header[:8] != b'\x89PNG\r\n\x1a\n':
                raise ValueError("Invalid PNG header: {}".format(image_path))
            width = int.from_bytes(header[16:20], byteorder='big')
            height = int.from_bytes(header[20:24], byteorder='big')
            return width, height

    raise ValueError("Unsupported map image format: {}".format(image_path))

def load_region_bounds_from_map(map_yaml_path):
    """
    Compute the valid map rectangle from the map_server YAML metadata.
    """
    image_path, resolution, origin = _parse_simple_map_yaml(map_yaml_path)
    width_px, height_px = _read_map_image_size(image_path)

    min_x = origin[0]
    min_y = origin[1]
    max_x = origin[0] + width_px * resolution
    max_y = origin[1] + height_px * resolution

    return {
        'map_yaml_path': map_yaml_path,
        'image_path': image_path,
        'resolution': resolution,
        'origin': origin,
        'width_px': width_px,
        'height_px': height_px,
        'min_x': min_x,
        'max_x': max_x,
        'min_y': min_y,
        'max_y': max_y,
    }

def clamp_region_to_map(map_region, min_x=None, max_x=None, min_y=None, max_y=None):
    """
    Apply optional user-provided region bounds and keep them inside the map.
    """
    region_min_x = map_region['min_x'] if min_x is None else max(map_region['min_x'], min_x)
    region_max_x = map_region['max_x'] if max_x is None else min(map_region['max_x'], max_x)
    region_min_y = map_region['min_y'] if min_y is None else max(map_region['min_y'], min_y)
    region_max_y = map_region['max_y'] if max_y is None else min(map_region['max_y'], max_y)

    if region_min_x >= region_max_x or region_min_y >= region_max_y:
        raise ValueError(
            "Invalid active region after clamping: x=[{}, {}], y=[{}, {}]".format(
                region_min_x, region_max_x, region_min_y, region_max_y
            )
        )

    return {
        'min_x': region_min_x,
        'max_x': region_max_x,
        'min_y': region_min_y,
        'max_y': region_max_y,
    }

class LocalClusterVisualizer:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('local_cluster_visualizer')
        self.last_scan = None
        self.last_clusters = []
        self.found_blocks = []
        self.input_frame = DEFAULT_INPUT_FRAME

        map_yaml_path = rospy.get_param('~map_yaml', DEFAULT_MAP_YAML)
        map_region = load_region_bounds_from_map(map_yaml_path)
        active_region = clamp_region_to_map(
            map_region,
            rospy.get_param('~region_min_x', None),
            rospy.get_param('~region_max_x', None),
            rospy.get_param('~region_min_y', None),
            rospy.get_param('~region_max_y', None),
        )
        self.region_min_x = active_region['min_x']
        self.region_max_x = active_region['max_x']
        self.region_min_y = active_region['min_y']
        self.region_max_y = active_region['max_y']

        # Initialize the TF buffer and listener to handle coordinate frame transformations.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the laser scan topic and set up publishers for visualization markers and block information.
        rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)
        self.info_pub = rospy.Publisher('/found_blocks_info', String, queue_size=10)

        # Process at a moderate rate so navigation costmaps keep enough CPU budget.
        rospy.Timer(rospy.Duration(0.5), self.process_callback)
        rospy.loginfo(
            "Loaded map bounds from %s: full_map x=[%.2f, %.2f], y=[%.2f, %.2f]",
            map_yaml_path,
            map_region['min_x'],
            map_region['max_x'],
            map_region['min_y'],
            map_region['max_y'],
        )
        rospy.loginfo(
            "Using active detection region: x=[%.2f, %.2f], y=[%.2f, %.2f]",
            self.region_min_x,
            self.region_max_x,
            self.region_min_y,
            self.region_max_y,
        )
        rospy.loginfo("LocalClusterVisualizer started. Processing clustering and fusion every 0.5 seconds.")

    def scan_callback(self, msg):
        # Callback to update the latest laser scan data.
        self.last_scan = msg
        if msg.header.frame_id:
            self.input_frame = msg.header.frame_id

    def process_callback(self, event):
        try:
            # Step 1: Prepare the point cloud from the laser scan data.
            scan = self.last_scan
            if scan is None:
                return
            pts = []
            ang = scan.angle_min
            for r in scan.ranges:
                # Only consider finite range values.
                if math.isfinite(r):
                    # Convert polar coordinates (range and angle) to Cartesian coordinates.
                    pts.append([r * math.cos(ang), r * math.sin(ang)])
                ang += scan.angle_increment
            pts = np.array(pts)
            if len(pts) < CLUSTER_MIN_SAMPLES:
                self.last_clusters = []
            else:
                # Step 2: Apply TF transformation.
                # Look up the latest transform from the input frame (laser) to the output frame (map).
                try:
                    trans = self.tf_buffer.lookup_transform(
                        OUTPUT_FRAME, self.input_frame, rospy.Time(0), rospy.Duration(1.0))
                except Exception as e:
                    rospy.logwarn("TF lookup failed: {}".format(e))
                    return

                # Step 3: Perform DBSCAN clustering on the point cloud.
                labels = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit_predict(pts)
                clusters = []
                # Process each detected cluster (ignore label -1 which indicates noise).
                for lbl in set(labels):
                    if lbl == -1:
                        continue
                    cpts = pts[labels == lbl]
                    mpts = []
                    # Transform each point in the cluster from the input frame to the output frame.
                    for x, y in cpts:
                        ps = PoseStamped()
                        ps.header.frame_id = self.input_frame
                        ps.header.stamp = rospy.Time(0)
                        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, 0
                        ps.pose.orientation.w = 1.0
                        try:
                            pm = tf2_geometry_msgs.do_transform_pose(ps, trans)
                            mpts.append([pm.pose.position.x, pm.pose.position.y])
                        except:
                            continue
                    if not mpts:
                        continue
                    arr = np.array(mpts)
                    # Calculate the bounding rectangle of the transformed points.
                    min_x, max_x = arr[:, 0].min(), arr[:, 0].max()
                    min_y, max_y = arr[:, 1].min(), arr[:, 1].max()
                    # Compute the center of the bounding rectangle.
                    cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
                    # Only consider clusters whose centers lie within the predefined valid region.
                    if not (self.region_min_x <= cx <= self.region_max_x and self.region_min_y <= cy <= self.region_max_y):
                        continue
                    clusters.append({
                        'center': (cx, cy),
                        'envelope': [[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]],
                        'width': max_x - min_x,
                        'height': max_y - min_y
                    })
                self.last_clusters = clusters

            # Step 4: Fusion update - merge new valid clusters with previously detected blocks.
            for cl in self.last_clusters:
                w, h = cl['width'], cl['height']
                # Consider only clusters with rectangle size within the valid range.
                if VALID_RECT_MIN_SIZE <= w <= VALID_RECT_MAX_SIZE and VALID_RECT_MIN_SIZE <= h <= VALID_RECT_MAX_SIZE:
                    nc = np.array(cl['center'])
                    merged = False
                    for fb in self.found_blocks:
                        fc = np.array(fb['center'])
                        if np.linalg.norm(nc - fc) < MERGE_DISTANCE_THRESHOLD:
                            # Fuse the new measurement with the existing block using weighted averaging.
                            uc = fc * FUSION_OLD_WEIGHT + nc * FUSION_NEW_WEIGHT
                            fb['center'] = (uc[0], uc[1])
                            fb['width']  = fb['width'] * FUSION_OLD_WEIGHT + w * FUSION_NEW_WEIGHT
                            fb['height'] = fb['height'] * FUSION_OLD_WEIGHT + h * FUSION_NEW_WEIGHT
                            # Update the bounding envelope based on the newly fused dimensions.
                            w2, h2 = fb['width'], fb['height']
                            x0, y0 = uc[0] - w2 / 2, uc[1] - h2 / 2
                            fb['envelope'] = [[x0, y0], [x0 + w2, y0], [x0 + w2, y0 + h2], [x0, y0 + h2]]
                            merged = True
                            break
                    if not merged:
                        # If no close block is found, append the new cluster as a new found block.
                        self.found_blocks.append(cl)

            # Step 5: Publish visualization markers.
            self.publish_markers()
            # Step 6: Publish information about the found blocks.
            self.publish_info()

        except Exception as e:
            rospy.logerr("Internal exception in process_callback: {}".format(e))

    def publish_markers(self):
        ma = MarkerArray()
        # Delete any old markers by sending delete actions for all namespaces.
        for ns in ["map_clusters", "map_cluster_rectangles", "found_blocks"]:
            for i in range(100):
                m = Marker()
                m.header.frame_id = OUTPUT_FRAME
                m.header.stamp = rospy.Time.now()
                m.ns = ns
                m.id = i
                m.action = Marker.DELETE
                ma.markers.append(m)

        # Add new markers: one for each detected cluster center and its bounding rectangle.
        for i, cl in enumerate(self.last_clusters):
            cx, cy = cl['center']
            env = cl['envelope']
            # Create a sphere marker for the cluster center.
            s = Marker()
            s.header.frame_id = OUTPUT_FRAME
            s.header.stamp = rospy.Time.now()
            s.ns = "map_clusters"
            s.id = i
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x, s.pose.position.y, s.pose.position.z = cx, cy, 0
            s.pose.orientation.w = 1
            s.scale.x = s.scale.y = s.scale.z = MARKER_SCALE
            s.color.r = 1
            s.color.a = 1
            ma.markers.append(s)

            # Create a line strip marker to represent the bounding rectangle of the cluster.
            r = Marker()
            r.header.frame_id = OUTPUT_FRAME
            r.header.stamp = rospy.Time.now()
            r.ns = "map_cluster_rectangles"
            r.id = i
            r.type = Marker.LINE_STRIP
            r.action = Marker.ADD
            r.scale.x = 0.1
            # Use green color if the rectangle size is valid; otherwise use an orange-like color.
            if VALID_RECT_MIN_SIZE <= cl['width'] <= VALID_RECT_MAX_SIZE and VALID_RECT_MIN_SIZE <= cl['height'] <= VALID_RECT_MAX_SIZE:
                r.color.g = 1
                r.color.a = 1
            else:
                r.color.r = 1
                r.color.g = 0.65
                r.color.a = 1
            # Append the rectangle vertices (closing the loop by adding the first point at the end).
            for x, y in env + [env[0]]:
                p = Point(x=x, y=y, z=0)
                r.points.append(p)
            ma.markers.append(r)

        # Add markers for the found blocks after fusion.
        for i, fb in enumerate(self.found_blocks):
            r = Marker()
            r.header.frame_id = OUTPUT_FRAME
            r.header.stamp = rospy.Time.now()
            r.ns = "found_blocks"
            r.id = i
            r.type = Marker.LINE_STRIP
            r.action = Marker.ADD
            r.scale.x = 0.1
            r.color.b = 1
            r.color.a = 1
            # Draw the envelope for the found block, closing the loop.
            for x, y in fb['envelope'] + [fb['envelope'][0]]:
                r.points.append(Point(x=x, y=y, z=0))
            ma.markers.append(r)

        # Publish the array of markers.
        self.marker_pub.publish(ma)

    def publish_info(self):
        # Build a string with information about each found block.
        rows = []
        for idx, fb in enumerate(self.found_blocks, 1):
            x, y = fb['center']
            rows.append(f"({idx},{x:.5f},{y:.5f},{fb.get('label',0)})")
        s = ", ".join(rows)
        if s:
            rospy.loginfo("Found blocks: " + s)
        # Publish the found blocks' information as a string message.
        self.info_pub.publish(String(data=s))

if __name__ == '__main__':
    try:
        LocalClusterVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

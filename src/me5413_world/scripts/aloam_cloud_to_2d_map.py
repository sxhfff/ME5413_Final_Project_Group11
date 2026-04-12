#!/usr/bin/env python3
import math
import os

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import yaml
from PIL import Image
from sensor_msgs.msg import PointCloud2


UNKNOWN = 205
FREE = 254
OCCUPIED = 0


def optional_float(value):
    if value is None:
        return None
    if isinstance(value, str) and value.strip() == "":
        return None
    return float(value)


def normalize_regions(value):
    if value in (None, ""):
        return []
    if isinstance(value, str):
        value = yaml.safe_load(value)
    if not value:
        return []
    regions = []
    for idx, region in enumerate(value):
        if not isinstance(region, dict):
            raise RuntimeError("region %d must be a mapping" % idx)
        try:
            regions.append(
                {
                    "min_x": float(region["min_x"]),
                    "max_x": float(region["max_x"]),
                    "min_y": float(region["min_y"]),
                    "max_y": float(region["max_y"]),
                }
            )
        except KeyError as exc:
            raise RuntimeError("region %d missing key %s" % (idx, exc))
    return regions


def normalize_slice_regions(value):
    if value in (None, ""):
        return []
    if isinstance(value, str):
        value = yaml.safe_load(value)
    if not value:
        return []
    regions = []
    for idx, region in enumerate(value):
        if not isinstance(region, dict):
            raise RuntimeError("slice region %d must be a mapping" % idx)
        try:
            item = {
                "min_x": float(region["min_x"]),
                "max_x": float(region["max_x"]),
                "min_y": float(region["min_y"]),
                "max_y": float(region["max_y"]),
                "min_z": float(region["min_z"]),
                "max_z": float(region["max_z"]),
            }
        except KeyError as exc:
            raise RuntimeError("slice region %d missing key %s" % (idx, exc))
        regions.append(item)
    return regions


def normalize_relative_regions(value):
    if value in (None, ""):
        return []
    if isinstance(value, str):
        value = yaml.safe_load(value)
    if not value:
        return []
    regions = []
    for idx, region in enumerate(value):
        if not isinstance(region, dict):
            raise RuntimeError("relative region %d must be a mapping" % idx)
        try:
            item = {
                "min_x": float(region["min_x"]),
                "max_x": float(region["max_x"]),
                "min_y": float(region["min_y"]),
                "max_y": float(region["max_y"]),
                "min_rel_height": float(region["min_rel_height"]),
                "max_rel_height": float(region["max_rel_height"]),
            }
        except KeyError as exc:
            raise RuntimeError("relative region %d missing key %s" % (idx, exc))
        regions.append(item)
    return regions


def normalize_merge_layers(value):
    if value in (None, ""):
        return []
    if isinstance(value, str):
        value = yaml.safe_load(value)
    if not value:
        return []
    layers = []
    for idx, layer in enumerate(value):
        if not isinstance(layer, dict):
            raise RuntimeError("merge layer %d must be a mapping" % idx)
        try:
            item = {
                "name": str(layer.get("name", "layer_%d" % idx)),
                "min_x": float(layer["min_x"]),
                "max_x": float(layer["max_x"]),
                "min_y": float(layer["min_y"]),
                "max_y": float(layer["max_y"]),
                "mode": str(layer.get("mode", "slice")),
                "min_points_per_cell": int(layer.get("min_points_per_cell", 1)),
            }
        except KeyError as exc:
            raise RuntimeError("merge layer %d missing key %s" % (idx, exc))

        if item["mode"] == "slice":
            item["min_z"] = float(layer["min_z"])
            item["max_z"] = float(layer["max_z"])
        elif item["mode"] == "relative_ground":
            item["min_rel_height"] = float(layer["min_rel_height"])
            item["max_rel_height"] = float(layer["max_rel_height"])
        else:
            raise RuntimeError("merge layer %d has unsupported mode %s" % (idx, item["mode"]))
        layers.append(item)
    return layers


def points_in_regions(x, y, regions):
    if not regions:
        return np.zeros_like(x, dtype=bool)
    mask = np.zeros_like(x, dtype=bool)
    for region in regions:
        mask |= (
            (x >= region["min_x"])
            & (x <= region["max_x"])
            & (y >= region["min_y"])
            & (y <= region["max_y"])
        )
    return mask


def points_in_slice_regions(x, y, z, regions):
    if not regions:
        return np.zeros_like(x, dtype=bool)
    mask = np.zeros_like(x, dtype=bool)
    for region in regions:
        mask |= (
            (x >= region["min_x"])
            & (x <= region["max_x"])
            & (y >= region["min_y"])
            & (y <= region["max_y"])
            & (z >= region["min_z"])
            & (z <= region["max_z"])
        )
    return mask


def bool_dilate(grid, radius):
    if radius <= 0:
        return grid.copy()
    padded = np.pad(grid, radius, mode="constant", constant_values=False)
    out = np.zeros_like(grid, dtype=bool)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y0 = radius + dy
            x0 = radius + dx
            out |= padded[y0:y0 + grid.shape[0], x0:x0 + grid.shape[1]]
    return out


def bool_erode(grid, radius):
    if radius <= 0:
        return grid.copy()
    padded = np.pad(grid, radius, mode="constant", constant_values=True)
    out = np.ones_like(grid, dtype=bool)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y0 = radius + dy
            x0 = radius + dx
            out &= padded[y0:y0 + grid.shape[0], x0:x0 + grid.shape[1]]
    return out


def bool_open(grid, radius):
    if radius <= 0:
        return grid.copy()
    return bool_dilate(bool_erode(grid, radius), radius)


def render_counts_to_grid(points_xy, min_x, min_y, width, height, resolution):
    counts = np.zeros((height, width), dtype=np.uint16)
    if points_xy.shape[0] == 0:
        return counts
    ix = np.floor((points_xy[:, 0] - min_x) / resolution).astype(np.int32)
    iy = np.floor((points_xy[:, 1] - min_y) / resolution).astype(np.int32)
    ix = np.clip(ix, 0, width - 1)
    iy = np.clip(iy, 0, height - 1)
    np.add.at(counts, (iy, ix), 1)
    return counts


def save_debug_preview(path, occupied):
    preview = np.full(occupied.shape, 255, dtype=np.uint8)
    preview[occupied] = 0
    Image.fromarray(np.flipud(preview), mode="L").save(path)


def compute_local_ground_mask(
    points,
    common_mask,
    resolution,
    padding,
    rel_min_height,
    rel_max_height,
    min_points_per_cell,
    ground_quantile,
):
    candidates = points[common_mask]
    if candidates.shape[0] == 0:
        return np.zeros(points.shape[0], dtype=bool)

    min_x = math.floor((candidates[:, 0].min() - padding) / resolution) * resolution
    min_y = math.floor((candidates[:, 1].min() - padding) / resolution) * resolution
    max_x = math.ceil((candidates[:, 0].max() + padding) / resolution) * resolution
    max_y = math.ceil((candidates[:, 1].max() + padding) / resolution) * resolution
    width = int(round((max_x - min_x) / resolution)) + 1
    height = int(round((max_y - min_y) / resolution)) + 1

    ix = np.floor((candidates[:, 0] - min_x) / resolution).astype(np.int32)
    iy = np.floor((candidates[:, 1] - min_y) / resolution).astype(np.int32)
    ix = np.clip(ix, 0, width - 1)
    iy = np.clip(iy, 0, height - 1)

    buckets = {}
    for idx, (cx, cy) in enumerate(zip(ix.tolist(), iy.tolist())):
        buckets.setdefault((cy, cx), []).append(float(candidates[idx, 2]))

    ground = np.full((height, width), np.nan, dtype=np.float32)
    for (cy, cx), zs in buckets.items():
        if len(zs) < min_points_per_cell:
            continue
        ground[cy, cx] = np.quantile(zs, ground_quantile)

    full_ix = np.floor((points[:, 0] - min_x) / resolution).astype(np.int32)
    full_iy = np.floor((points[:, 1] - min_y) / resolution).astype(np.int32)
    valid_xy = (
        (full_ix >= 0)
        & (full_ix < width)
        & (full_iy >= 0)
        & (full_iy < height)
    )
    mask = np.zeros(points.shape[0], dtype=bool)
    valid_indices = np.where(common_mask & valid_xy)[0]
    if valid_indices.size == 0:
        return mask

    local_ground = ground[full_iy[valid_indices], full_ix[valid_indices]]
    valid_ground = np.isfinite(local_ground)
    rel_height = points[valid_indices, 2] - local_ground
    selected = valid_ground & (rel_height >= rel_min_height) & (rel_height <= rel_max_height)
    mask[valid_indices[selected]] = True
    return mask


def compute_local_ground_mask_for_regions(
    points,
    base_mask,
    regions,
    resolution,
    padding,
    min_points_per_cell,
    ground_quantile,
):
    if not regions:
        return np.zeros(points.shape[0], dtype=bool)
    x = points[:, 0]
    y = points[:, 1]
    combined = np.zeros(points.shape[0], dtype=bool)
    for region in regions:
        region_xy = (
            (x >= region["min_x"])
            & (x <= region["max_x"])
            & (y >= region["min_y"])
            & (y <= region["max_y"])
        )
        region_mask = compute_local_ground_mask(
            points,
            base_mask & region_xy,
            resolution,
            padding,
            region["min_rel_height"],
            region["max_rel_height"],
            min_points_per_cell,
            ground_quantile,
        )
        combined |= region_mask
    return combined


class AloamCloudTo2DMap:
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/laser_cloud_surround")
        self.cloud_file = rospy.get_param("~cloud_file", "")
        self.output_prefix = rospy.get_param("~output_prefix", "")
        self.resolution = float(rospy.get_param("~resolution", 0.05))
        self.min_height = float(rospy.get_param("~min_height", 0.05))
        self.max_height = float(rospy.get_param("~max_height", 0.30))
        self.min_range = float(rospy.get_param("~min_range", 0.30))
        self.max_range = float(rospy.get_param("~max_range", 10.0))
        self.padding = float(rospy.get_param("~padding", 0.50))
        self.min_points_per_cell = int(rospy.get_param("~min_points_per_cell", 2))
        self.noise_open_radius_cells = int(rospy.get_param("~noise_open_radius_cells", 1))
        self.inflate_radius = float(rospy.get_param("~inflate_radius", 0.25))
        self.free_space_radius = float(rospy.get_param("~free_space_radius", 0.18))
        self.crop_min_x = optional_float(rospy.get_param("~crop_min_x", None))
        self.crop_max_x = optional_float(rospy.get_param("~crop_max_x", None))
        self.crop_min_y = optional_float(rospy.get_param("~crop_min_y", None))
        self.crop_max_y = optional_float(rospy.get_param("~crop_max_y", None))
        self.relative_ground_mode = bool(rospy.get_param("~relative_ground_mode", False))
        self.relative_ground_resolution = float(rospy.get_param("~relative_ground_resolution", 0.30))
        self.relative_ground_quantile = float(rospy.get_param("~relative_ground_quantile", 0.10))
        self.relative_ground_min_height = float(rospy.get_param("~relative_ground_min_height", 0.10))
        self.relative_ground_max_height = float(rospy.get_param("~relative_ground_max_height", 0.80))
        self.relative_ground_min_points = int(rospy.get_param("~relative_ground_min_points", 3))
        self.relative_ground_regions = normalize_relative_regions(rospy.get_param("~relative_ground_regions", []))
        self.merge_layers = normalize_merge_layers(rospy.get_param("~merge_layers", []))
        self.auto_slice_scan = bool(rospy.get_param("~auto_slice_scan", False))
        self.auto_slice_step = float(rospy.get_param("~auto_slice_step", 0.10))
        self.auto_slice_thickness = float(rospy.get_param("~auto_slice_thickness", 0.20))
        self.auto_slice_select_count = int(rospy.get_param("~auto_slice_select_count", 2))
        self.auto_slice_min_separation = float(rospy.get_param("~auto_slice_min_separation", 1.00))
        self.auto_slice_min_points = int(rospy.get_param("~auto_slice_min_points", 500))
        self.auto_slice_preview_dir = rospy.get_param("~auto_slice_preview_dir", "")
        self.slice_regions = normalize_slice_regions(rospy.get_param("~slice_regions", []))
        self.projection_regions = normalize_regions(rospy.get_param("~projection_regions", []))
        self.projection_min_points_per_cell = int(rospy.get_param("~projection_min_points_per_cell", 3))
        self.projection_min_height = optional_float(rospy.get_param("~projection_min_height", None))
        self.projection_max_height = optional_float(rospy.get_param("~projection_max_height", None))
        self.occupied_thresh = float(rospy.get_param("~occupied_thresh", 0.65))
        self.free_thresh = float(rospy.get_param("~free_thresh", 0.196))
        self.negate = int(rospy.get_param("~negate", 0))
        self._done = False

        if not self.output_prefix:
            map_dir = os.path.join(os.path.dirname(__file__), "..", "maps")
            self.output_prefix = os.path.abspath(os.path.join(map_dir, "aloam_projected_nav_map"))
        else:
            self.output_prefix = os.path.abspath(self.output_prefix)

        if self.cloud_file:
            self.cloud_file = os.path.abspath(self.cloud_file)
            rospy.loginfo("aloam_cloud_to_2d_map: loading %s", self.cloud_file)
            self.export_points(load_point_cloud_file(self.cloud_file))
            rospy.signal_shutdown("map export complete")
            return

        rospy.loginfo("aloam_cloud_to_2d_map: waiting for %s", self.cloud_topic)
        self.sub = rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

    def cloud_cb(self, msg):
        if self._done:
            return
        self._done = True
        try:
            points = np.array(
                list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float32,
            )
            self.export_points(points)
        except Exception as exc:
            rospy.logerr("aloam_cloud_to_2d_map: failed: %s", exc)
        finally:
            rospy.signal_shutdown("map export complete")

    def export_points(self, points):
        if points.size == 0:
            raise RuntimeError("point cloud is empty")

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        ranges = np.hypot(x, y)

        common_mask = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & (ranges >= self.min_range)
            & (ranges <= self.max_range)
        )

        if self.crop_min_x is not None:
            common_mask &= x >= self.crop_min_x
        if self.crop_max_x is not None:
            common_mask &= x <= self.crop_max_x
        if self.crop_min_y is not None:
            common_mask &= y >= self.crop_min_y
        if self.crop_max_y is not None:
            common_mask &= y <= self.crop_max_y

        region_mask = points_in_regions(x, y, self.projection_regions)
        slice_region_mask = points_in_slice_regions(x, y, z, self.slice_regions)
        slice_regions_xy_mask = points_in_regions(
            x,
            y,
            [{"min_x": r["min_x"], "max_x": r["max_x"], "min_y": r["min_y"], "max_y": r["max_y"]} for r in self.slice_regions],
        )

        if self.relative_ground_mode:
            base_relative_mask = compute_local_ground_mask(
                points,
                common_mask & ~region_mask & ~slice_regions_xy_mask,
                self.relative_ground_resolution,
                self.padding,
                self.relative_ground_min_height,
                self.relative_ground_max_height,
                self.relative_ground_min_points,
                self.relative_ground_quantile,
            )
            region_relative_mask = compute_local_ground_mask_for_regions(
                points,
                common_mask & ~region_mask & ~slice_regions_xy_mask,
                self.relative_ground_regions,
                self.relative_ground_resolution,
                self.padding,
                self.relative_ground_min_points,
                self.relative_ground_quantile,
            )
            default_slice_mask = base_relative_mask | region_relative_mask
        else:
            default_slice_mask = (
                common_mask & ~region_mask & ~slice_regions_xy_mask & (z >= self.min_height) & (z <= self.max_height)
            )
        region_slice_mask = common_mask & ~region_mask & slice_region_mask
        slice_mask = default_slice_mask | region_slice_mask

        projection_mask = common_mask & region_mask
        if self.projection_min_height is not None:
            projection_mask &= z >= self.projection_min_height
        if self.projection_max_height is not None:
            projection_mask &= z <= self.projection_max_height

        filtered = points[slice_mask | projection_mask]
        if filtered.shape[0] == 0:
            raise RuntimeError("no points remain after filtering")

        min_x = math.floor((filtered[:, 0].min() - self.padding) / self.resolution) * self.resolution
        min_y = math.floor((filtered[:, 1].min() - self.padding) / self.resolution) * self.resolution
        max_x = math.ceil((filtered[:, 0].max() + self.padding) / self.resolution) * self.resolution
        max_y = math.ceil((filtered[:, 1].max() + self.padding) / self.resolution) * self.resolution

        width = int(round((max_x - min_x) / self.resolution)) + 1
        height = int(round((max_y - min_y) / self.resolution)) + 1
        if width <= 0 or height <= 0:
            raise RuntimeError("invalid map size")

        if self.merge_layers:
            occupied = self.build_merged_layers_occupancy(points, common_mask, min_x, min_y, width, height)
            occupied = bool_open(occupied, self.noise_open_radius_cells)
            inflate_cells = int(round(self.inflate_radius / self.resolution))
            free_cells = int(round(self.free_space_radius / self.resolution))
            occupied = bool_dilate(occupied, inflate_cells)
            known = bool_dilate(occupied, free_cells)

            image = np.full((height, width), UNKNOWN, dtype=np.uint8)
            image[known] = FREE
            image[occupied] = OCCUPIED
            image = np.flipud(image)

            pgm_path = self.output_prefix + ".pgm"
            yaml_path = self.output_prefix + ".yaml"
            os.makedirs(os.path.dirname(pgm_path), exist_ok=True)
            Image.fromarray(image, mode="L").save(pgm_path)
            metadata = {
                "image": pgm_path,
                "resolution": float(self.resolution),
                "origin": [float(min_x), float(min_y), 0.0],
                "negate": int(self.negate),
                "occupied_thresh": float(self.occupied_thresh),
                "free_thresh": float(self.free_thresh),
            }
            with open(yaml_path, "w", encoding="ascii") as handle:
                yaml.safe_dump(metadata, handle, sort_keys=False)
            rospy.loginfo(
                "aloam_cloud_to_2d_map: saved merged-layer map %s and %s using %d layers",
                pgm_path,
                yaml_path,
                len(self.merge_layers),
            )
            return

        if self.auto_slice_scan:
            slice_mask = self.build_auto_slice_mask(points, common_mask, region_mask, min_x, min_y, width, height)
            filtered = points[slice_mask | projection_mask]
            if filtered.shape[0] == 0:
                raise RuntimeError("no points remain after auto slice selection")
            min_x = math.floor((filtered[:, 0].min() - self.padding) / self.resolution) * self.resolution
            min_y = math.floor((filtered[:, 1].min() - self.padding) / self.resolution) * self.resolution
            max_x = math.ceil((filtered[:, 0].max() + self.padding) / self.resolution) * self.resolution
            max_y = math.ceil((filtered[:, 1].max() + self.padding) / self.resolution) * self.resolution
            width = int(round((max_x - min_x) / self.resolution)) + 1
            height = int(round((max_y - min_y) / self.resolution)) + 1

        slice_points = points[slice_mask]
        projection_points = points[projection_mask]

        slice_counts = render_counts_to_grid(slice_points[:, :2], min_x, min_y, width, height, self.resolution)

        projection_counts = render_counts_to_grid(
            projection_points[:, :2], min_x, min_y, width, height, self.resolution
        )

        occupied = (slice_counts >= self.min_points_per_cell) | (
            projection_counts >= self.projection_min_points_per_cell
        )
        occupied = bool_open(occupied, self.noise_open_radius_cells)

        inflate_cells = int(round(self.inflate_radius / self.resolution))
        free_cells = int(round(self.free_space_radius / self.resolution))
        occupied = bool_dilate(occupied, inflate_cells)
        known = bool_dilate(occupied, free_cells)

        image = np.full((height, width), UNKNOWN, dtype=np.uint8)
        image[known] = FREE
        image[occupied] = OCCUPIED
        image = np.flipud(image)

        pgm_path = self.output_prefix + ".pgm"
        yaml_path = self.output_prefix + ".yaml"
        os.makedirs(os.path.dirname(pgm_path), exist_ok=True)

        Image.fromarray(image, mode="L").save(pgm_path)

        metadata = {
            "image": pgm_path,
            "resolution": float(self.resolution),
            "origin": [float(min_x), float(min_y), 0.0],
            "negate": int(self.negate),
            "occupied_thresh": float(self.occupied_thresh),
            "free_thresh": float(self.free_thresh),
        }
        with open(yaml_path, "w", encoding="ascii") as handle:
            yaml.safe_dump(metadata, handle, sort_keys=False)

        rospy.loginfo(
            "aloam_cloud_to_2d_map: saved %s and %s from %d filtered points (%d slice, %d projection)",
            pgm_path,
            yaml_path,
            filtered.shape[0],
            slice_points.shape[0],
            projection_points.shape[0],
        )

    def build_merged_layers_occupancy(self, points, common_mask, min_x, min_y, width, height):
        x = points[:, 0]
        y = points[:, 1]
        occupied = np.zeros((height, width), dtype=bool)
        for layer in self.merge_layers:
            xy_mask = (
                common_mask
                & (x >= layer["min_x"])
                & (x <= layer["max_x"])
                & (y >= layer["min_y"])
                & (y <= layer["max_y"])
            )
            if layer["mode"] == "slice":
                layer_mask = xy_mask & (points[:, 2] >= layer["min_z"]) & (points[:, 2] <= layer["max_z"])
            else:
                layer_mask = compute_local_ground_mask(
                    points,
                    xy_mask,
                    self.relative_ground_resolution,
                    self.padding,
                    layer["min_rel_height"],
                    layer["max_rel_height"],
                    self.relative_ground_min_points,
                    self.relative_ground_quantile,
                )
            layer_points = points[layer_mask]
            counts = render_counts_to_grid(layer_points[:, :2], min_x, min_y, width, height, self.resolution)
            layer_occ = counts >= layer["min_points_per_cell"]
            occupied |= layer_occ
        return occupied

    def build_auto_slice_mask(self, points, common_mask, region_mask, min_x, min_y, width, height):
        candidates = points[common_mask & ~region_mask]
        if candidates.shape[0] == 0:
            raise RuntimeError("no points available for auto slice scan")

        z_values = candidates[:, 2]
        z_min = float(z_values.min())
        z_max = float(z_values.max())
        centers = np.arange(z_min, z_max + self.auto_slice_step, self.auto_slice_step, dtype=np.float32)
        previews_dir = self.auto_slice_preview_dir
        if previews_dir:
            previews_dir = os.path.abspath(previews_dir)
            os.makedirs(previews_dir, exist_ok=True)

        scored = []
        half = 0.5 * self.auto_slice_thickness
        for center in centers:
            mask = common_mask & ~region_mask & (points[:, 2] >= center - half) & (points[:, 2] <= center + half)
            slice_points = points[mask]
            if slice_points.shape[0] < self.auto_slice_min_points:
                continue

            counts = render_counts_to_grid(slice_points[:, :2], min_x, min_y, width, height, self.resolution)
            occupied = counts >= self.min_points_per_cell
            if self.noise_open_radius_cells > 0:
                occupied = bool_open(occupied, self.noise_open_radius_cells)

            occupied_cells = int(occupied.sum())
            if occupied_cells == 0:
                continue
            boundary = occupied & ~bool_erode(occupied, 1)
            boundary_cells = int(boundary.sum())
            boundary_ratio = boundary_cells / float(occupied_cells)
            score = occupied_cells * boundary_ratio
            scored.append(
                {
                    "center": float(center),
                    "score": float(score),
                    "occupied_cells": occupied_cells,
                    "boundary_ratio": float(boundary_ratio),
                    "mask": mask.copy(),
                    "occupied": occupied,
                }
            )

        if not scored:
            raise RuntimeError("auto slice scan found no valid z bands")

        scored.sort(key=lambda item: item["score"], reverse=True)
        selected = []
        for item in scored:
            if all(abs(item["center"] - keep["center"]) >= self.auto_slice_min_separation for keep in selected):
                selected.append(item)
            if len(selected) >= self.auto_slice_select_count:
                break

        if previews_dir:
            for idx, item in enumerate(selected):
                preview_name = "slice_%02d_z_%0.2f_score_%0.0f.png" % (idx, item["center"], item["score"])
                save_debug_preview(os.path.join(previews_dir, preview_name), item["occupied"])

        combined_mask = np.zeros(points.shape[0], dtype=bool)
        for item in selected:
            combined_mask |= item["mask"]

        rospy.loginfo(
            "aloam_cloud_to_2d_map: auto-slice selected z bands: %s",
            ", ".join("%0.2f" % item["center"] for item in selected),
        )
        return combined_mask


def parse_pcd_header(path):
    header = {}
    data_offset = 0
    header_lines = 0
    with open(path, "rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("invalid PCD header")
            data_offset += len(line)
            header_lines += 1
            text = line.decode("ascii", errors="ignore").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == "DATA":
                break
    return header, data_offset, header_lines


def load_ascii_pcd(path, fields, header_lines):
    usecols = [fields.index("x"), fields.index("y"), fields.index("z")]
    points = np.loadtxt(path, comments="#", skiprows=header_lines, usecols=usecols, dtype=np.float32)
    return np.atleast_2d(points)


def load_pcd_file(path):
    header, data_offset, header_lines = parse_pcd_header(path)
    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(set(fields)):
        raise RuntimeError("PCD file must contain x y z fields")

    data_type = header.get("DATA", [""])[0].lower()
    if data_type == "ascii":
        return load_ascii_pcd(path, fields, header_lines)

    if data_type != "binary":
        raise RuntimeError("unsupported PCD DATA mode: %s" % data_type)

    sizes = list(map(int, header["SIZE"]))
    types = header["TYPE"]
    counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
    points_count = int(header["POINTS"][0])

    dtype_fields = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        if typ == "F" and size == 4:
            dt = np.float32
        elif typ == "F" and size == 8:
            dt = np.float64
        elif typ == "U" and size == 1:
            dt = np.uint8
        elif typ == "U" and size == 2:
            dt = np.uint16
        elif typ == "U" and size == 4:
            dt = np.uint32
        elif typ == "I" and size == 1:
            dt = np.int8
        elif typ == "I" and size == 2:
            dt = np.int16
        elif typ == "I" and size == 4:
            dt = np.int32
        else:
            raise RuntimeError("unsupported PCD field type: %s %s" % (typ, size))
        dtype_fields.append((name, dt) if count == 1 else (name, dt, (count,)))

    with open(path, "rb") as handle:
        handle.seek(data_offset)
        raw = handle.read()

    data = np.frombuffer(raw, dtype=np.dtype(dtype_fields), count=points_count)
    return np.column_stack((data["x"], data["y"], data["z"])).astype(np.float32, copy=False)


def load_ascii_ply(path):
    vertex_count = None
    property_names = []
    header_lines = 0
    with open(path, "r", encoding="ascii", errors="ignore") as handle:
        for line in handle:
            header_lines += 1
            text = line.strip()
            if text.startswith("element vertex"):
                vertex_count = int(text.split()[2])
            elif text.startswith("property"):
                parts = text.split()
                if len(parts) >= 3:
                    property_names.append(parts[-1])
            elif text == "end_header":
                break
    if vertex_count is None:
        raise RuntimeError("PLY file missing vertex count")
    if not {"x", "y", "z"}.issubset(set(property_names)):
        raise RuntimeError("PLY file must contain x y z properties")
    usecols = [property_names.index("x"), property_names.index("y"), property_names.index("z")]
    points = np.loadtxt(path, skiprows=header_lines, usecols=usecols, max_rows=vertex_count, dtype=np.float32)
    return np.atleast_2d(points)


def load_binary_ply(path):
    vertex_count = None
    properties = []
    in_vertex = False
    data_offset = 0
    with open(path, "rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("invalid PLY header")
            data_offset += len(line)
            text = line.decode("ascii", errors="ignore").strip()
            if text.startswith("element vertex"):
                vertex_count = int(text.split()[2])
                in_vertex = True
            elif text.startswith("element ") and not text.startswith("element vertex"):
                in_vertex = False
            elif in_vertex and text.startswith("property"):
                parts = text.split()
                if parts[1] == "list":
                    raise RuntimeError("list properties in vertex block are unsupported")
                properties.append((parts[2], parts[1]))
            elif text == "end_header":
                break

        if vertex_count is None:
            raise RuntimeError("PLY file missing vertex count")
        prop_map = {
            "char": "i1",
            "uchar": "u1",
            "short": "i2",
            "ushort": "u2",
            "int": "i4",
            "uint": "u4",
            "float": "f4",
            "double": "f8",
        }
        dtype_fields = []
        for name, ptype in properties:
            if ptype not in prop_map:
                raise RuntimeError("unsupported PLY property type: %s" % ptype)
            dtype_fields.append((name, "<" + prop_map[ptype]))

        handle.seek(data_offset)
        data = np.frombuffer(handle.read(), dtype=np.dtype(dtype_fields), count=vertex_count)

    if not {"x", "y", "z"}.issubset({name for name, _ in properties}):
        raise RuntimeError("PLY file must contain x y z properties")
    return np.column_stack((data["x"], data["y"], data["z"])).astype(np.float32, copy=False)


def load_ply_file(path):
    with open(path, "rb") as handle:
        header = handle.read(256).decode("ascii", errors="ignore")
    if "format ascii 1.0" in header:
        return load_ascii_ply(path)
    if "format binary_little_endian 1.0" in header:
        return load_binary_ply(path)
    raise RuntimeError("unsupported PLY format")


def load_point_cloud_file(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".pcd":
        return load_pcd_file(path)
    if ext == ".ply":
        return load_ply_file(path)
    if ext in (".xyz", ".pts"):
        points = np.loadtxt(path, usecols=(0, 1, 2), dtype=np.float32)
        return np.atleast_2d(points)
    raise RuntimeError("unsupported point cloud file extension: %s" % ext)


if __name__ == "__main__":
    rospy.init_node("aloam_cloud_to_2d_map")
    AloamCloudTo2DMap()
    rospy.spin()

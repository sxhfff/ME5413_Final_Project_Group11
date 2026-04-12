#!/usr/bin/env python3
import os
import shutil
import subprocess
import tempfile
import threading

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

SUPPORTED_DIGITS = tuple(str(digit) for digit in range(1, 10))
OCR_WHITELIST = "".join(SUPPORTED_DIGITS)
DEFAULT_IMAGE_TOPIC = "/front/image_raw"
MODEL_TEMPLATE_DIGITS = tuple(str(digit) for digit in range(1, 10))
SHAPE_CANVAS_SIZE = 96


class DigitRecognizer:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.current_frame = None
        self.shape_templates = self.build_shape_templates()

        self.image_topic = rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC)
        self.vote_rate_hz = float(rospy.get_param("~vote_rate_hz", 8.0))
        self.ocr_min_score = float(rospy.get_param("~ocr_min_score", 0.34))
        self.roi_x_margin_ratio = float(rospy.get_param("~roi_x_margin_ratio", 0.06))
        self.roi_y_margin_ratio = float(rospy.get_param("~roi_y_margin_ratio", 0.08))
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.0008))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.55))
        self.min_aspect_ratio = float(rospy.get_param("~min_aspect_ratio", 0.45))
        self.max_aspect_ratio = float(rospy.get_param("~max_aspect_ratio", 3.2))
        self.max_center_offset_ratio = float(rospy.get_param("~max_center_offset_ratio", 0.50))
        self.max_candidates = int(rospy.get_param("~max_candidates", 8))
        self.candidate_padding_ratio = float(rospy.get_param("~candidate_padding_ratio", 0.16))
        self.candidate_iou_threshold = float(rospy.get_param("~candidate_iou_threshold", 0.35))
        self.close_range_crop_ratio = float(rospy.get_param("~close_range_crop_ratio", 0.78))
        self.consensus_min_detections = int(rospy.get_param("~consensus_min_detections", 1))
        self.consensus_min_avg_score = float(rospy.get_param("~consensus_min_avg_score", 0.34))
        self.consensus_min_max_score = float(rospy.get_param("~consensus_min_max_score", 0.45))

        self.tesseract_cmd = shutil.which(rospy.get_param("~tesseract_cmd", "tesseract"))
        if self.tesseract_cmd is None:
            rospy.logwarn(
                "未找到 tesseract 可执行文件。当前 OCR 简化版已接入，但识别结果会始终为空。"
            )
        else:
            rospy.loginfo("Using tesseract executable: %s", self.tesseract_cmd)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        self.best_digit = None
        self.best_score = -1.0
        self.stop_event = threading.Event()
        self.thread = None
        self.detection_counts = {}
        self.detection_scores = {}

    def build_shape_templates(self):
        templates = {}
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        for digit in MODEL_TEMPLATE_DIGITS:
            texture_path = os.path.join(
                package_root,
                "models",
                f"number{digit}",
                "materials",
                "textures",
                f"number{digit}.png",
            )
            image = cv2.imread(texture_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                rospy.logwarn("未找到数字模板贴图：%s", texture_path)
                continue

            if image.ndim == 3 and image.shape[2] == 4:
                gray = image[:, :, 3]
            elif image.ndim == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image

            _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            normalized = self.normalize_mask(binary)
            contour = self.extract_largest_contour(normalized)
            if normalized is not None and contour is not None:
                templates[digit] = {
                    "mask": normalized,
                    "contour": contour,
                }
        return templates

    def image_callback(self, img_msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except cv_bridge.CvBridgeError as exc:
            rospy.logerr("cv_bridge 转换错误：%s", exc)

    def preprocess_frame(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        height, width = gray.shape[:2]
        x_margin = int(width * self.roi_x_margin_ratio)
        y_margin = int(height * self.roi_y_margin_ratio)
        roi = gray[y_margin:height - y_margin, x_margin:width - x_margin]
        if roi.size == 0:
            roi = gray
            x_margin = 0
            y_margin = 0

        return {
            "gray": roi,
            "x_offset": x_margin,
            "y_offset": y_margin,
            "frame_width": width,
            "frame_height": height,
        }

    def order_points(self, points):
        points = np.array(points, dtype=np.float32)
        rect = np.zeros((4, 2), dtype=np.float32)
        sums = points.sum(axis=1)
        diffs = np.diff(points, axis=1)
        rect[0] = points[np.argmin(sums)]
        rect[2] = points[np.argmax(sums)]
        rect[1] = points[np.argmin(diffs)]
        rect[3] = points[np.argmax(diffs)]
        return rect

    def crop_rotated_patch(self, image, contour):
        rect = cv2.minAreaRect(contour)
        (center_x, center_y), (width, height), _ = rect
        if width < 1 or height < 1:
            return None

        box = cv2.boxPoints(rect)
        ordered_box = self.order_points(box)
        target_width = max(1, int(round(max(width, height))))
        target_height = max(1, int(round(min(width, height))))
        if target_width < target_height:
            target_width, target_height = target_height, target_width

        dst = np.array(
            [
                [0, 0],
                [target_width - 1, 0],
                [target_width - 1, target_height - 1],
                [0, target_height - 1],
            ],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(ordered_box, dst)
        warped = cv2.warpPerspective(image, transform, (target_width, target_height))
        if warped.size == 0:
            return None

        if warped.shape[0] > warped.shape[1]:
            warped = cv2.rotate(warped, cv2.ROTATE_90_CLOCKWISE)

        bbox = cv2.boundingRect(contour)
        return {
            "patch": warped,
            "bbox": bbox,
            "center": (center_x, center_y),
            "size": (width, height),
        }

    def build_candidate_masks(self, gray):
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary_dark = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        _, binary_bright = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)

        open_kernel = np.ones((3, 3), np.uint8)
        close_kernel = np.ones((5, 5), np.uint8)
        masks = []
        for mask in (binary_dark, binary_bright, edges):
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, close_kernel)
            masks.append(cleaned)
        return masks

    def candidate_penalty(self, candidate, roi_shape):
        roi_h, roi_w = roi_shape[:2]
        center_x = candidate["center"][0]
        center_offset = abs(center_x - (roi_w / 2.0)) / max(1.0, roi_w / 2.0)
        box_w, box_h = candidate["bbox"][2], candidate["bbox"][3]
        area_ratio = (box_w * box_h) / float(max(1, roi_w * roi_h))
        return center_offset + (0.15 * area_ratio)

    def iou(self, box_a, box_b):
        ax1, ay1, aw, ah = box_a
        bx1, by1, bw, bh = box_b
        ax2, ay2 = ax1 + aw, ay1 + ah
        bx2, by2 = bx1 + bw, by1 + bh

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area <= 0:
            return 0.0

        union_area = (aw * ah) + (bw * bh) - inter_area
        return inter_area / float(max(1, union_area))

    def extract_candidates(self, frame_maps):
        gray = frame_maps["gray"]
        roi_h, roi_w = gray.shape[:2]
        roi_area = float(max(1, roi_h * roi_w))
        max_center_offset_px = self.max_center_offset_ratio * (roi_w / 2.0)

        candidates = []
        for mask in self.build_candidate_masks(gray):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area <= 0:
                    continue

                area_ratio = area / roi_area
                if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
                    continue

                rect = cv2.minAreaRect(contour)
                (_, _), (width, height), _ = rect
                if width < 12 or height < 12:
                    continue

                long_side = max(width, height)
                short_side = min(width, height)
                if short_side <= 1:
                    continue

                aspect_ratio = long_side / short_side
                if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                    continue

                center_x = rect[0][0]
                if abs(center_x - (roi_w / 2.0)) > max_center_offset_px:
                    continue

                candidate = self.crop_rotated_patch(gray, contour)
                if candidate is None:
                    continue

                candidate["penalty"] = self.candidate_penalty(candidate, gray.shape)
                candidates.append(candidate)

        fallback_candidate = self.build_center_fallback_candidate(gray)
        if fallback_candidate is not None:
            candidates.append(fallback_candidate)

        candidates.sort(key=lambda item: item["penalty"])

        deduped = []
        for candidate in candidates:
            if any(self.iou(candidate["bbox"], kept["bbox"]) > self.candidate_iou_threshold for kept in deduped):
                continue
            deduped.append(candidate)
            if len(deduped) >= self.max_candidates:
                break
        return deduped

    def build_center_fallback_candidate(self, gray):
        if gray is None or gray.size == 0:
            return None

        roi_h, roi_w = gray.shape[:2]
        crop_ratio = min(0.95, max(0.4, self.close_range_crop_ratio))
        crop_w = max(24, int(round(roi_w * crop_ratio)))
        crop_h = max(24, int(round(roi_h * crop_ratio)))
        x0 = max(0, (roi_w - crop_w) // 2)
        y0 = max(0, (roi_h - crop_h) // 2)
        x1 = min(roi_w, x0 + crop_w)
        y1 = min(roi_h, y0 + crop_h)
        patch = gray[y0:y1, x0:x1]
        if patch.size == 0:
            return None

        return {
            "patch": patch,
            "bbox": (x0, y0, x1 - x0, y1 - y0),
            "center": ((x0 + x1) / 2.0, (y0 + y1) / 2.0),
            "size": (x1 - x0, y1 - y0),
            "penalty": 0.02,
        }

    def select_digit_component(self, binary_image):
        if binary_image is None or binary_image.size == 0:
            return None

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
        height, width = binary_image.shape[:2]
        image_area = float(max(1, height * width))
        center_x = width / 2.0
        center_y = height / 2.0
        best_label = None
        best_score = -1e9

        for label in range(1, num_labels):
            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            w = stats[label, cv2.CC_STAT_WIDTH]
            h = stats[label, cv2.CC_STAT_HEIGHT]
            area = stats[label, cv2.CC_STAT_AREA]
            if w < 8 or h < 12 or area <= 0:
                continue

            area_ratio = area / image_area
            cx, cy = centroids[label]
            center_penalty = np.hypot(cx - center_x, cy - center_y) / max(1.0, 0.5 * max(width, height))
            border_penalty = 0.0
            if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
                border_penalty = 0.18

            fill_ratio = area / float(max(1, w * h))
            aspect_ratio = max(w, h) / float(max(1, min(w, h)))
            score = (3.2 * area_ratio) + (1.2 * fill_ratio) - (0.85 * center_penalty) - border_penalty - (0.08 * aspect_ratio)
            if score > best_score:
                best_score = score
                best_label = label

        if best_label is None:
            return None

        component_mask = np.where(labels == best_label, 255, 0).astype(np.uint8)
        points = cv2.findNonZero(component_mask)
        if points is None:
            return None
        x, y, w, h = cv2.boundingRect(points)
        return component_mask, (x, y, w, h)

    def choose_digit_mask(self, patch):
        if patch is None or patch.size == 0:
            return None

        blurred = cv2.GaussianBlur(patch, (5, 5), 0)
        _, binary_inv = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        kernel = np.ones((3, 3), np.uint8)
        candidates = [
            cv2.morphologyEx(binary_inv, cv2.MORPH_OPEN, kernel),
            cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel),
        ]

        best_mask = None
        best_bbox = None
        best_area = -1
        for candidate in candidates:
            selected = self.select_digit_component(candidate)
            if selected is None:
                continue
            component_mask, bbox = selected
            area = cv2.countNonZero(component_mask)
            if area > best_area:
                best_area = area
                best_mask = component_mask
                best_bbox = bbox

        if best_mask is None:
            return None
        return best_mask, best_bbox

    def normalize_mask(self, binary_mask):
        if binary_mask is None or binary_mask.size == 0:
            return None
        points = cv2.findNonZero(binary_mask)
        if points is None:
            return None

        x, y, w, h = cv2.boundingRect(points)
        roi = binary_mask[y:y + h, x:x + w]
        if roi.size == 0:
            return None

        target = SHAPE_CANVAS_SIZE - 20
        scale = min(target / float(max(1, w)), target / float(max(1, h)))
        resized = cv2.resize(
            roi,
            (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
            interpolation=cv2.INTER_NEAREST,
        )
        canvas = np.zeros((SHAPE_CANVAS_SIZE, SHAPE_CANVAS_SIZE), dtype=np.uint8)
        x_offset = (SHAPE_CANVAS_SIZE - resized.shape[1]) // 2
        y_offset = (SHAPE_CANVAS_SIZE - resized.shape[0]) // 2
        canvas[y_offset:y_offset + resized.shape[0], x_offset:x_offset + resized.shape[1]] = resized
        return canvas

    def extract_largest_contour(self, binary_mask):
        if binary_mask is None or binary_mask.size == 0:
            return None
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def score_shape_against_template(self, candidate_mask, candidate_contour, template_mask, template_contour):
        if candidate_mask is None or candidate_contour is None:
            return -1.0
        corr = cv2.matchTemplate(
            candidate_mask.astype(np.float32) / 255.0,
            template_mask.astype(np.float32) / 255.0,
            cv2.TM_CCOEFF_NORMED,
        )[0][0]
        intersection = np.logical_and(candidate_mask > 0, template_mask > 0).sum()
        union = np.logical_or(candidate_mask > 0, template_mask > 0).sum()
        iou = float(intersection) / float(max(1, union))
        shape_distance = cv2.matchShapes(candidate_contour, template_contour, cv2.CONTOURS_MATCH_I1, 0.0)
        shape_score = 1.0 / (1.0 + shape_distance)
        return 0.40 * corr + 0.35 * iou + 0.25 * shape_score

    def refine_digit_with_shape(self, roi, ocr_digit, ocr_score):
        if roi is None:
            return int(ocr_digit), ocr_score

        selected = self.choose_digit_mask(roi)
        if selected is None:
            return int(ocr_digit), ocr_score
        mask, _ = selected
        normalized = self.normalize_mask(mask)
        contour = self.extract_largest_contour(normalized)
        if normalized is None or contour is None:
            return int(ocr_digit), ocr_score

        shape_scores = {}
        for digit in SHAPE_TEMPLATE_DIGITS:
            template = self.shape_templates.get(digit)
            if template is None:
                continue
            shape_scores[digit] = self.score_shape_against_template(
                normalized,
                contour,
                template["mask"],
                template["contour"],
            )

        if len(shape_scores) < 2:
            return int(ocr_digit), ocr_score

        best_shape_digit = max(shape_scores, key=shape_scores.get)
        best_shape_score = shape_scores[best_shape_digit]
        current_shape_score = shape_scores.get(str(ocr_digit), -1.0)

        if best_shape_digit != str(ocr_digit) and best_shape_score > current_shape_score + 0.08:
            refined_score = max(ocr_score, min(0.95, 0.55 + 0.4 * best_shape_score))
            rospy.loginfo(
                "Shape refinement overrides OCR digit %s -> %s (shape %.3f vs %.3f).",
                ocr_digit,
                best_shape_digit,
                best_shape_score,
                current_shape_score,
            )
            return int(best_shape_digit), refined_score

        return int(ocr_digit), ocr_score

    def recognize_patch_with_template(self, patch):
        roi = self.tighten_digit_roi(patch)
        if roi is None:
            return None

        selected = self.choose_digit_mask(roi)
        if selected is None:
            return None
        mask, _ = selected
        normalized = self.normalize_mask(mask)
        contour = self.extract_largest_contour(normalized)
        if normalized is None or contour is None:
            return None

        best_digit = None
        best_score = -1.0
        for digit, template in self.shape_templates.items():
            score = self.score_shape_against_template(
                normalized,
                contour,
                template["mask"],
                template["contour"],
            )
            if score > best_score:
                best_score = score
                best_digit = digit

        if best_digit is None or best_score < 0.33:
            return None
        return int(best_digit), min(0.95, 0.52 + 0.38 * best_score)

    def tighten_digit_roi(self, patch):
        if patch is None or patch.size == 0:
            return None

        patch = cv2.copyMakeBorder(
            patch, 8, 8, 8, 8, cv2.BORDER_CONSTANT, value=255
        )
        selected = self.choose_digit_mask(patch)
        if selected is None:
            return None

        _, (x, y, w, h) = selected
        if w < 8 or h < 12:
            return None

        pad_x = max(4, int(w * self.candidate_padding_ratio))
        pad_y = max(4, int(h * self.candidate_padding_ratio))
        x0 = max(0, x - pad_x)
        y0 = max(0, y - pad_y)
        x1 = min(patch.shape[1], x + w + pad_x)
        y1 = min(patch.shape[0], y + h + pad_y)
        roi = patch[y0:y1, x0:x1]
        if roi.size == 0:
            return None
        return roi

    def build_ocr_variants(self, roi):
        resized = cv2.resize(roi, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
        blurred = cv2.GaussianBlur(resized, (3, 3), 0)

        _, binary_inv = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        adaptive = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            11,
        )
        adaptive_bright = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            11,
        )
        thickened = cv2.dilate(binary_inv, np.ones((3, 3), np.uint8), iterations=1)

        variants = [resized, binary_inv, binary, adaptive, adaptive_bright, thickened]
        cleaned_variants = []
        for variant in variants:
            if variant.ndim == 3:
                variant = cv2.cvtColor(variant, cv2.COLOR_BGR2GRAY)
            cleaned_variants.append(variant)
        return cleaned_variants

    def run_tesseract(self, image):
        if self.tesseract_cmd is None:
            return None

        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            cv2.imwrite(temp_path, image)
            command = [
                self.tesseract_cmd,
                temp_path,
                "stdout",
                "--psm",
                "10",
                "--oem",
                "3",
                "-c",
                "tessedit_char_whitelist={}".format(OCR_WHITELIST),
                "tsv",
            ]
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                rospy.logdebug("tesseract failed: %s", result.stderr.strip())
                return None

            best_digit = None
            best_conf = -1.0
            for line in result.stdout.splitlines()[1:]:
                parts = line.split("\t")
                if len(parts) < 12:
                    continue
                text = parts[11].strip()
                if text not in SUPPORTED_DIGITS:
                    continue
                try:
                    confidence = float(parts[10])
                except ValueError:
                    continue
                if confidence > best_conf:
                    best_conf = confidence
                    best_digit = text

            if best_digit is None:
                return None
            return best_digit, max(0.0, min(1.0, best_conf / 100.0))
        finally:
            try:
                os.remove(temp_path)
            except OSError:
                pass

    def recognize_patch_with_ocr(self, patch):
        roi = self.tighten_digit_roi(patch)
        if roi is None:
            return None

        votes = {}
        for variant in self.build_ocr_variants(roi):
            result = self.run_tesseract(variant)
            if result is None:
                continue
            digit, score = result
            votes.setdefault(digit, []).append(score)

        if not votes:
            return None

        ranked = sorted(
            votes.items(),
            key=lambda item: (len(item[1]), float(np.mean(item[1])), float(max(item[1]))),
            reverse=True,
        )
        digit, scores = ranked[0]
        final_score = 0.65 * float(np.mean(scores)) + 0.35 * float(max(scores))
        if final_score < self.ocr_min_score:
            return None
        refined_digit, refined_score = self.refine_digit_with_shape(roi, digit, final_score)
        return refined_digit, refined_score

    def recognize_digit(self, cv_image):
        frame_maps = self.preprocess_frame(cv_image)
        candidates = self.extract_candidates(frame_maps)
        if not candidates:
            return None

        best_result = None
        best_total_score = -1.0
        roi_w = frame_maps["gray"].shape[1]
        frame_center_x = cv_image.shape[1] / 2.0

        for candidate in candidates:
            ocr_result = self.recognize_patch_with_ocr(candidate["patch"])
            template_result = self.recognize_patch_with_template(candidate["patch"])

            if ocr_result is None and template_result is None:
                continue

            if ocr_result is not None and template_result is not None:
                ocr_digit, ocr_score = ocr_result
                template_digit, template_score = template_result
                if ocr_digit == template_digit:
                    digit = ocr_digit
                    total_score = max(ocr_score, 0.55 * ocr_score + 0.45 * template_score)
                elif template_score > ocr_score + 0.08:
                    digit = template_digit
                    total_score = template_score
                else:
                    digit = ocr_digit
                    total_score = ocr_score
            elif ocr_result is not None:
                digit, total_score = ocr_result
            else:
                digit, total_score = template_result

            center_x = frame_maps["x_offset"] + candidate["center"][0]
            pixel_offset = center_x - frame_center_x
            center_penalty = abs(candidate["center"][0] - (roi_w / 2.0)) / max(1.0, roi_w / 2.0)
            total_score -= 0.03 * center_penalty

            if total_score > best_total_score:
                best_total_score = total_score
                best_result = (digit, total_score, pixel_offset)

        return best_result

    def recognition_loop(self):
        rate_hz = max(1.0, self.vote_rate_hz)
        rate = rospy.Rate(rate_hz)
        missing_ocr_warned = False

        while not self.stop_event.is_set():
            if self.current_frame is None:
                rate.sleep()
                continue

            if self.tesseract_cmd is None:
                if not missing_ocr_warned:
                    rospy.logwarn("OCR 未启用，因为系统中未找到 tesseract。")
                    missing_ocr_warned = True
                rate.sleep()
                continue

            frame = self.current_frame.copy()
            result = self.recognize_digit(frame)
            if result is not None:
                digit, score, pixel_offset = result
                self.detection_counts[digit] = self.detection_counts.get(digit, 0) + 1
                self.detection_scores.setdefault(digit, []).append(score)

                if score > self.best_score:
                    self.best_score = score
                    self.best_digit = digit

                rospy.loginfo(
                    "OCR 候选: 数字 %s, 分数 %.3f, 偏移 %.1f px, 命中次数 %d",
                    digit,
                    score,
                    pixel_offset,
                    self.detection_counts[digit],
                )
            rate.sleep()

    def select_consensus_digit(self):
        if not self.detection_counts:
            return None

        ranked_digits = sorted(
            self.detection_counts.keys(),
            key=lambda digit: (
                self.detection_counts[digit],
                float(np.mean(self.detection_scores.get(digit, [-1.0]))),
                float(max(self.detection_scores.get(digit, [-1.0]))),
            ),
            reverse=True,
        )

        best_digit = ranked_digits[0]
        best_scores = self.detection_scores.get(best_digit, [self.best_score])
        best_avg_score = float(np.mean(best_scores))
        best_max_score = float(max(best_scores))
        if (
            self.detection_counts[best_digit] < self.consensus_min_detections
            or best_avg_score < self.consensus_min_avg_score
            or best_max_score < self.consensus_min_max_score
        ):
            rospy.logwarn(
                "最佳候选 %s 证据不足: 命中次数 %d, 平均分 %.3f, 最高分 %.3f。",
                best_digit,
                self.detection_counts[best_digit],
                best_avg_score,
                best_max_score,
            )
            if self.detection_counts[best_digit] >= 1 and best_max_score >= self.ocr_min_score:
                rospy.logwarn(
                    "放宽兜底返回数字 %s，避免近距离场景因门槛过高被判定为未识别。",
                    best_digit,
                )
                return best_digit
            return None

        rospy.loginfo(
            "最终识别: 数字 %s, 命中次数 %d, 平均分 %.3f, 最高分 %.3f",
            best_digit,
            self.detection_counts[best_digit],
            best_avg_score,
            best_max_score,
        )
        return best_digit

    def start_recognition(self):
        self.stop_event.clear()
        self.best_digit = None
        self.best_score = -1.0
        self.detection_counts = {}
        self.detection_scores = {}
        self.thread = threading.Thread(target=self.recognition_loop)
        self.thread.start()
        rospy.loginfo("数字识别启动……")

    def stop_recognition(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        rospy.loginfo("数字识别停止。")
        return self.select_consensus_digit()

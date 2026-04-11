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


class DigitRecognizer:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.current_frame = None

        self.image_topic = rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC)
        self.vote_rate_hz = float(rospy.get_param("~vote_rate_hz", 8.0))
        self.ocr_min_score = float(rospy.get_param("~ocr_min_score", 0.45))
        self.roi_x_margin_ratio = float(rospy.get_param("~roi_x_margin_ratio", 0.10))
        self.roi_y_margin_ratio = float(rospy.get_param("~roi_y_margin_ratio", 0.12))
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.0008))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.22))
        self.min_aspect_ratio = float(rospy.get_param("~min_aspect_ratio", 0.45))
        self.max_aspect_ratio = float(rospy.get_param("~max_aspect_ratio", 1.75))
        self.max_center_offset_ratio = float(rospy.get_param("~max_center_offset_ratio", 0.38))
        self.max_candidates = int(rospy.get_param("~max_candidates", 6))
        self.candidate_padding_ratio = float(rospy.get_param("~candidate_padding_ratio", 0.10))
        self.candidate_iou_threshold = float(rospy.get_param("~candidate_iou_threshold", 0.35))
        self.consensus_min_detections = int(rospy.get_param("~consensus_min_detections", 3))
        self.consensus_min_avg_score = float(rospy.get_param("~consensus_min_avg_score", 0.45))
        self.consensus_min_max_score = float(rospy.get_param("~consensus_min_max_score", 0.55))

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

        candidates.sort(key=lambda item: item["penalty"])

        deduped = []
        for candidate in candidates:
            if any(self.iou(candidate["bbox"], kept["bbox"]) > self.candidate_iou_threshold for kept in deduped):
                continue
            deduped.append(candidate)
            if len(deduped) >= self.max_candidates:
                break
        return deduped

    def tighten_digit_roi(self, patch):
        if patch is None or patch.size == 0:
            return None

        patch = cv2.copyMakeBorder(
            patch, 8, 8, 8, 8, cv2.BORDER_CONSTANT, value=255
        )
        blurred = cv2.GaussianBlur(patch, (5, 5), 0)
        _, binary_inv = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        binary_inv = cv2.morphologyEx(
            binary_inv, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )

        points = cv2.findNonZero(binary_inv)
        if points is None:
            return None

        x, y, w, h = cv2.boundingRect(points)
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
        resized = cv2.resize(roi, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
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

        variants = [resized, binary_inv, binary, adaptive]
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
        return int(digit), final_score

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
            if ocr_result is None:
                continue

            digit, ocr_score = ocr_result
            center_x = frame_maps["x_offset"] + candidate["center"][0]
            pixel_offset = center_x - frame_center_x
            center_penalty = abs(candidate["center"][0] - (roi_w / 2.0)) / max(1.0, roi_w / 2.0)
            total_score = ocr_score - (0.10 * center_penalty)

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

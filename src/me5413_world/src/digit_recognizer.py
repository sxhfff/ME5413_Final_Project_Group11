#!/usr/bin/env python3
import os
import threading

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

CANONICAL_TEMPLATE_SIZE = 120
FRAME_MATCH_THRESHOLD = 0.34
FRAME_MATCH_MARGIN = 0.04
CONSENSUS_MIN_DETECTIONS = 2
CONSENSUS_MIN_AVG_SCORE = 0.34
CONSENSUS_MIN_MAX_SCORE = 0.36
ROI_X_MARGIN_RATIO = 0.10
ROI_Y_MARGIN_RATIO = 0.12
SCALE_SAMPLES = 28
MIN_TEMPLATE_SCALE = 0.20
MAX_TEMPLATE_SCALE = 1.80
MAX_ABS_PIXEL_OFFSET = 90.0
SUPPORTED_DIGITS = range(1, 10)


class DigitRecognizer:
    def __init__(self, templates_dir=None):
        self.templates = {}
        self.template_debug_sources = {}

        script_dir = os.path.dirname(os.path.abspath(__file__))
        package_root = os.path.dirname(script_dir)
        if templates_dir is None:
            templates_dir = os.path.join(script_dir, "templates")

        for digit in SUPPORTED_DIGITS:
            digit_key = str(digit)
            processed_template = None
            template_source = None

            candidate_paths = [
                os.path.join(templates_dir, "{}.png".format(digit)),
                os.path.join(
                    package_root,
                    "models",
                    "number{}".format(digit),
                    "materials",
                    "textures",
                    "number{}.png".format(digit),
                ),
            ]

            for template_path in candidate_paths:
                if not os.path.exists(template_path):
                    continue
                raw_template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
                processed_template = self.prepare_template(raw_template)
                if processed_template is not None:
                    template_source = template_path
                    break

            if processed_template is not None:
                self.templates[digit_key] = processed_template
                self.template_debug_sources[digit_key] = template_source
                rospy.loginfo("Loaded template %s from %s", digit_key, template_source)
            else:
                rospy.logwarn("No usable template found for digit %s", digit_key)

        if not self.templates:
            raise Exception("未加载任何模板，请检查 src/templates 或 models/number*/materials/textures 路径。")

        self.bridge = cv_bridge.CvBridge()
        self.current_frame = None
        self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)

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

    def crop_foreground(self, image):
        if image is None or image.size == 0:
            return None

        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        _, binary_inv = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        points = cv2.findNonZero(binary_inv)
        if points is None:
            return image

        x, y, w, h = cv2.boundingRect(points)
        pad = max(4, int(0.08 * max(w, h)))
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(image.shape[1], x + w + pad)
        y1 = min(image.shape[0], y + h + pad)
        cropped = image[y0:y1, x0:x1]
        return cropped if cropped.size != 0 else image

    def normalize_digit_patch(self, image):
        if image is None or image.size == 0:
            return None

        cropped = self.crop_foreground(image)
        if cropped is None or cropped.size == 0:
            return None

        side = max(cropped.shape[:2])
        square = np.full((side, side), 255, dtype=np.uint8)
        y_offset = (side - cropped.shape[0]) // 2
        x_offset = (side - cropped.shape[1]) // 2
        square[y_offset:y_offset + cropped.shape[0], x_offset:x_offset + cropped.shape[1]] = cropped
        return cv2.resize(square, (CANONICAL_TEMPLATE_SIZE, CANONICAL_TEMPLATE_SIZE), interpolation=cv2.INTER_AREA)

    def prepare_template(self, image):
        if image is None or image.size == 0:
            return None

        normalized = self.normalize_digit_patch(image)
        if normalized is None:
            return None

        _, binary_inv = cv2.threshold(normalized, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        edges = cv2.Canny(normalized, 50, 150)
        return {
            "gray": normalized,
            "binary": binary_inv,
            "edges": edges,
        }

    def preprocess_frame(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        height, width = gray.shape[:2]
        x_margin = int(width * ROI_X_MARGIN_RATIO)
        y_margin = int(height * ROI_Y_MARGIN_RATIO)
        roi = gray[y_margin:height - y_margin, x_margin:width - x_margin]
        if roi.size == 0:
            roi = gray
            x_margin = 0
            y_margin = 0

        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _, binary_inv = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binary_inv = cv2.morphologyEx(binary_inv, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        edges = cv2.Canny(blurred, 50, 150)

        return {
            "gray": roi,
            "binary": binary_inv,
            "edges": edges,
            "x_offset": x_margin,
            "y_offset": y_margin,
        }

    def match_template_family(self, frame_maps, template_maps):
        best_score = -1.0
        best_loc = None
        best_shape = None

        for scale in np.linspace(MIN_TEMPLATE_SCALE, MAX_TEMPLATE_SCALE, SCALE_SAMPLES):
            try:
                resized_binary = cv2.resize(
                    template_maps["binary"],
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_AREA,
                )
                resized_edges = cv2.resize(
                    template_maps["edges"],
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_AREA,
                )
                resized_gray = cv2.resize(
                    template_maps["gray"],
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_AREA,
                )
            except Exception:
                continue

            t_h, t_w = resized_binary.shape[:2]
            if t_h < 12 or t_w < 12:
                continue
            if frame_maps["binary"].shape[0] < t_h or frame_maps["binary"].shape[1] < t_w:
                continue

            binary_res = cv2.matchTemplate(frame_maps["binary"], resized_binary, cv2.TM_CCOEFF_NORMED)
            edge_res = cv2.matchTemplate(frame_maps["edges"], resized_edges, cv2.TM_CCOEFF_NORMED)
            gray_res = cv2.matchTemplate(frame_maps["gray"], resized_gray, cv2.TM_CCOEFF_NORMED)
            _, binary_max, _, binary_loc = cv2.minMaxLoc(binary_res)
            _, edge_max, _, _ = cv2.minMaxLoc(edge_res)
            _, gray_max, _, _ = cv2.minMaxLoc(gray_res)

            center_x = binary_loc[0] + t_w / 2.0
            image_center_x = frame_maps["binary"].shape[1] / 2.0
            center_penalty = abs(center_x - image_center_x) / max(1.0, image_center_x)
            combined_score = (0.50 * binary_max) + (0.20 * edge_max) + (0.30 * gray_max) - (0.05 * center_penalty)

            if combined_score > best_score:
                best_score = combined_score
                best_loc = binary_loc
                best_shape = (t_h, t_w)

        return best_score, best_loc, best_shape

    def recognize_digit(self, cv_image):
        frame_maps = self.preprocess_frame(cv_image)

        best_score = -1.0
        best_digit = None
        best_loc = None
        best_shape = None
        second_best_score = -1.0

        for digit, template_maps in self.templates.items():
            score, loc, shape = self.match_template_family(frame_maps, template_maps)
            if score > best_score:
                second_best_score = best_score
                best_score = score
                best_digit = digit
                best_loc = loc
                best_shape = shape
            elif score > second_best_score:
                second_best_score = score

        if best_score < FRAME_MATCH_THRESHOLD or best_digit is None or best_loc is None or best_shape is None:
            return None
        if FRAME_MATCH_MARGIN > 0.0 and best_score - second_best_score < FRAME_MATCH_MARGIN:
            rospy.logdebug(
                "最佳候选 %s 与第二名分差过小: %.3f vs %.3f",
                best_digit,
                best_score,
                second_best_score,
            )
            return None

        t_h, t_w = best_shape
        match_center_x = frame_maps["x_offset"] + best_loc[0] + t_w / 2.0
        image_center_x = cv_image.shape[1] / 2.0
        pixel_offset = match_center_x - image_center_x
        if abs(pixel_offset) > MAX_ABS_PIXEL_OFFSET:
            rospy.logdebug(
                "候选 %s 偏移过大: %.1f px (limit %.1f), 忽略",
                best_digit,
                pixel_offset,
                MAX_ABS_PIXEL_OFFSET,
            )
            return None
        return best_digit, best_score, pixel_offset

    def recognition_loop(self):
        rate = rospy.Rate(10)
        while not self.stop_event.is_set():
            if self.current_frame is None:
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
                    "识别候选: 数字 %s, 分数 %.3f, 偏移 %.1f px, 命中次数 %d",
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
                np.mean(self.detection_scores.get(digit, [-1.0])),
                max(self.detection_scores.get(digit, [-1.0])),
            ),
            reverse=True,
        )

        best_digit = ranked_digits[0]
        best_scores = self.detection_scores.get(best_digit, [self.best_score])
        best_avg_score = float(np.mean(best_scores))
        best_max_score = float(max(best_scores))
        if (
            self.detection_counts[best_digit] < CONSENSUS_MIN_DETECTIONS
            or best_avg_score < CONSENSUS_MIN_AVG_SCORE
            or best_max_score < CONSENSUS_MIN_MAX_SCORE
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

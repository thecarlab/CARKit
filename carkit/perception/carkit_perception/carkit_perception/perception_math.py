# Copyright 2026 CARKit maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


TRAFFIC_LIGHT_UNKNOWN = 0
TRAFFIC_LIGHT_RED = 1
TRAFFIC_LIGHT_YELLOW = 2
TRAFFIC_LIGHT_GREEN = 3


@dataclass(frozen=True)
class Detection2D:
    class_id: int
    class_name: str
    bbox: tuple[float, float, float, float]
    confidence: float


def clipped_bbox(
    bbox: tuple[float, float, float, float],
    width: int,
    height: int,
) -> Optional[tuple[int, int, int, int]]:
    x1, y1, x2, y2 = bbox
    left = max(0, min(width, int(math.floor(x1))))
    top = max(0, min(height, int(math.floor(y1))))
    right = max(0, min(width, int(math.ceil(x2))))
    bottom = max(0, min(height, int(math.ceil(y2))))
    if left >= right or top >= bottom:
        return None
    return left, top, right, bottom


class TrafficLightClassifier:
    def __init__(
        self,
        min_saturation: int = 80,
        min_value: int = 100,
        min_ratio: float = 0.02,
        winner_margin: float = 1.2,
    ) -> None:
        self.min_saturation = min_saturation
        self.min_value = min_value
        self.min_ratio = min_ratio
        self.winner_margin = winner_margin

    def classify(
        self,
        image: np.ndarray,
        bbox: tuple[float, float, float, float],
    ) -> int:
        height, width = image.shape[:2]
        bounds = clipped_bbox(bbox, width, height)
        if bounds is None:
            return TRAFFIC_LIGHT_UNKNOWN

        left, top, right, bottom = bounds
        crop = image[top:bottom, left:right]
        if crop.size == 0 or crop.shape[0] < 6 or crop.shape[1] < 3:
            return TRAFFIC_LIGHT_UNKNOWN

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        vivid = (saturation >= self.min_saturation) & (value >= self.min_value)

        masks = {
            TRAFFIC_LIGHT_RED: vivid & ((hue <= 10) | (hue >= 170)),
            TRAFFIC_LIGHT_YELLOW: vivid & (hue >= 15) & (hue <= 40),
            TRAFFIC_LIGHT_GREEN: vivid & (hue >= 40) & (hue <= 95),
        }
        expected_thirds = {
            TRAFFIC_LIGHT_RED: (0.0, 1.0 / 3.0),
            TRAFFIC_LIGHT_YELLOW: (1.0 / 3.0, 2.0 / 3.0),
            TRAFFIC_LIGHT_GREEN: (2.0 / 3.0, 1.0),
        }

        scores = {}
        crop_area = float(crop.shape[0] * crop.shape[1])
        for color, mask in masks.items():
            start_fraction, end_fraction = expected_thirds[color]
            start = int(crop.shape[0] * start_fraction)
            end = max(start + 1, int(crop.shape[0] * end_fraction))
            third = mask[start:end, :]
            positional_ratio = (
                float(np.count_nonzero(third)) / float(third.size)
            )
            full_ratio = float(np.count_nonzero(mask)) / crop_area
            scores[color] = positional_ratio + 0.25 * full_ratio

        ranked = sorted(
            scores.items(),
            key=lambda item: item[1],
            reverse=True,
        )
        best_color, best_score = ranked[0]
        runner_up_score = ranked[1][1]
        if best_score < self.min_ratio:
            return TRAFFIC_LIGHT_UNKNOWN
        if (
            runner_up_score > 0.0
            and best_score < runner_up_score * self.winner_margin
        ):
            return TRAFFIC_LIGHT_UNKNOWN
        return best_color

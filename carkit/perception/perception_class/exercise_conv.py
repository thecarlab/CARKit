import cv2
import numpy as np
from pathlib import Path

# -------------------------
# Exercise 4: Image filters
# -------------------------
# Goal:
# See that a filter can act like a small pattern detector or image transformer.
#
# A vertical edge filter becomes bright where the image changes
# strongly from left to right.
#
# A horizontal edge filter becomes bright where the image changes
# strongly from top to bottom.

IMAGE_PATH = "Clear_stop_sign.jpg"


def apply_filter(img, image_filter, output_name):
    output = cv2.filter2D(img, -1, image_filter)
    output_path = Path(f"exercise_conv_{output_name}.jpg")
    output_path.unlink(missing_ok=True)
    cv2.imwrite(str(output_path), output)


img = cv2.imread(IMAGE_PATH, cv2.IMREAD_GRAYSCALE)

filter_0 = np.array([
    [-1, 2, -1],
    [-1, 2, -1],
    [-1, 2, -1],
])

apply_filter(img, filter_0, "filter_0")

# TODO:
filter_1 = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
])
apply_filter(img, filter_1, "filter_1")

# TODO: 
filter_2 = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
])
apply_filter(img, filter_2, "filter_2")

# TODO:
filter_3 = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
])
apply_filter(img, filter_3, "filter_3")

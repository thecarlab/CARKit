import cv2
from pathlib import Path

# -------------------------
# Exercise 3: Color + shape stop sign detector
# -------------------------
# Goal:
# 1. Use BGR color values to find red pixels.
# 2. Use contours to group red pixels into shapes.
# 3. Keep shapes that look like an octagon.

IMAGE_PATH = "Covered_stop_sign.jpg"
MASK_OUTPUT_PATH = "exercise_3_mask.jpg"
DETECTION_OUTPUT_PATH = "exercise_3.jpg"

# Color filter settings.
# OpenCV loads color images as BGR: Blue, Green, Red.
# A pixel is treated as red if:
# 1. its red value is bright enough
# 2. red is stronger than green
# 3. red is stronger than blue
RED_MIN_VALUE = 150
RED_TO_GREEN_RATIO = 1.5
RED_TO_BLUE_RATIO = 1.5

# Size filter settings.
# Small red regions are usually noise, not a stop sign.
MIN_CONTOUR_AREA = 1000

# A stop sign is close to an octagon.
MIN_POLYGON_CORNERS = 7
MAX_POLYGON_CORNERS = 9

# Higher values make polygon approximation simpler.
# Lower values keep more detail.
POLYGON_APPROX_FACTOR = 0.04


img = cv2.imread(IMAGE_PATH)

# OpenCV stores color images in BGR order: Blue, Green, Red.
blue = img[:, :, 0]
green = img[:, :, 1]
red = img[:, :, 2]

# The mask should be True only when:
# - red is greater than RED_MIN_VALUE
# - red is greater than green times RED_TO_GREEN_RATIO
# - red is greater than blue times RED_TO_BLUE_RATIO
#
# Hint:
# A & B & C means A, B, and C must all be true.
mask = (
    (red > RED_MIN_VALUE)
    & (red > green * RED_TO_GREEN_RATIO)
    & (red > blue * RED_TO_BLUE_RATIO)
)

# Convert True/False values into image values:
# True becomes 255, False becomes 0.
mask = mask.astype("uint8") * 255

# findContours groups neighboring white pixels into shapes.
# In this exercise:
# - white pixels are red enough
# - black pixels are ignored
# - each contour is one connected red region
#
# cv2.RETR_EXTERNAL:
# Only return the outside outline of each shape.
#
# cv2.CHAIN_APPROX_SIMPLE:
# Store the outline efficiently instead of storing every edge pixel.
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    area = cv2.contourArea(contour)

    # TODO: Skip contours if area is smaller than MIN_CONTOUR_AREA.
    if False:
        continue

    perimeter = cv2.arcLength(contour, True)

    # contour:
    # The detailed outline.
    #
    # POLYGON_APPROX_FACTOR * perimeter:
    # Controls how much we simplify the outline.
    # Multiplying by perimeter makes the simplification scale with object size.
    simplification_amount = POLYGON_APPROX_FACTOR * perimeter
    polygon = cv2.approxPolyDP(contour, simplification_amount, True)

    # Check whether the polygon has about 8 corners.
    is_octagon_like = MIN_POLYGON_CORNERS <= len(polygon) <= MAX_POLYGON_CORNERS

    # Draw a rectangular box around likely stop signs.
    if is_octagon_like:
        x, y, w, h = cv2.boundingRect(polygon)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.putText(img, "stop sign", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

Path(MASK_OUTPUT_PATH).unlink(missing_ok=True)
Path(DETECTION_OUTPUT_PATH).unlink(missing_ok=True)

cv2.imwrite(MASK_OUTPUT_PATH, mask)
cv2.imwrite(DETECTION_OUTPUT_PATH, img)
